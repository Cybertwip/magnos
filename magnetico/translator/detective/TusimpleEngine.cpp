#include "TusimpleEngine.h"

TusimpleEngine::TusimpleEngine() : OnnxEngine(std::string{MODELS_PATH} + "fastroad/288x800/roadseg.onnx") {
	
	std::vector<float> row_anchor(tusimple_row_anchor, tusimple_row_anchor + sizeof(tusimple_row_anchor) / sizeof(tusimple_row_anchor[0]));
	
	cfg.img_w = 1280;
	cfg.img_h = 720;
	
	cfg.in_w = 800;
	cfg.in_h = 288;

	cfg.row_anchor = row_anchor;
	cfg.griding_num = 100;
	cfg.cls_num_per_lane = 56;
	
	channeledDataBuffer[0].resize(cfg.in_w * cfg.in_h);
	channeledDataBuffer[1].resize(cfg.in_w * cfg.in_h);
	channeledDataBuffer[2].resize(cfg.in_w * cfg.in_h);
	
	processedTensorBuffer = std::vector<float>(cfg.in_w * cfg.in_h * 3);
}

std::vector<float> TusimpleEngine::prepareInputImage(Image& image, const std::vector<float>& mean, const std::vector<float>& std) {
	
	std::vector<uint8_t>& data = image.data;
		
	// Apply preprocessing and flatten the original image into a 1D vector
	size_t counter = 0;
	size_t skipper = 0;
	size_t colorCounter[3] = {0, 0, 0};

	for(size_t i = 0; i<data.size(); ++i){
		
		if(skipper == 3 && image.channels == 4){
			skipper = 0;
			continue;
		}
		
		int color = data[i];
		float pixel_value = (static_cast<float>(color) / 255.0 - mean[counter % 3]) / std[counter%3];
		
		channeledDataBuffer[counter % 3][colorCounter[counter % 3]++] = pixel_value;
		counter++;
		skipper++;
	}

	size_t offset = 0;
	
	// Copy the data from each channel to the flattenedData
	for (size_t channel = 0; channel < channeledDataBuffer.size(); ++channel) {
		size_t channelSize = channeledDataBuffer[channel].size();
		memcpy(processedTensorBuffer.data() + offset, channeledDataBuffer[channel].data(), channelSize * sizeof(float));
		offset += channelSize;
	}

	return processedTensorBuffer;
}

std::tuple<Image, std::vector<std::vector<std::vector<int>>>, std::vector<bool>> TusimpleEngine::detectLanes(Image& image){
	// Create the input tensor
	std::vector<int64_t> input_shape = {1, 3, cfg.in_h, cfg.in_w};
	
	auto scaledImage = resizeImage(image, cfg.in_w, cfg.in_h);
	
	// Define mean and std values
	std::vector<float> mean = {0.485, 0.456, 0.406};
	std::vector<float> std = {0.229, 0.224, 0.225};
	
	auto inputTensor = prepareInputImage(scaledImage, mean, std);
	
	auto output = inference({inputTensor}, {input_shape});
	
	auto [lanes_points, lanes_detected] = process_output(output[0], cfg);
		
	return std::make_tuple(drawLanes(image, lanes_points, lanes_detected, cfg), lanes_points, lanes_detected);
	
}
void TusimpleEngine::drawFilledCircle(std::vector<std::uint8_t>& image_data, int image_width, int image_height, int channels, int center_x, int center_y, int radius, const std::vector<uint8_t>& color) {
	for (int y = -radius; y <= radius; y++) {
		for (int x = -radius; x <= radius; x++) {
			if (x * x + y * y <= radius * radius) {
				int row = center_y + y;
				int col = center_x + x;
				
				if (row >= 0 && row < image_height && col >= 0 && col < image_width) {
					int index = channels * (row * image_width + col);
					image_data[index] = color[0];
					image_data[index + 1] = color[1];
					image_data[index + 2] = color[2];
					
					if(channels == 4){
						image_data[index + 3] = color[3];
					}
				}
			}
		}
	}
}

Image TusimpleEngine::drawLanes(Image& image, const std::vector<std::vector<std::vector<int>>>& lanes_points, const std::vector<bool>& lanes_detected, const Config& cfg) {
		
	int image_width = cfg.img_w;
	int image_height = cfg.img_h;
	int image_channels = image.channels;
	
	std::vector<std::uint8_t>& image_data = image.data;
	
	for (size_t lane_num = 0; lane_num < lanes_detected.size(); ++lane_num) {
		if (lanes_detected[lane_num]) {
			for (const auto& lane_point : lanes_points[lane_num]) {
				int col = lane_point[0];
				int row = lane_point[1];
				
				// Draw a larger dot at the lane point
				drawFilledCircle(image_data, image_width, image_height, image_channels, col, row, 10, laneColors[lane_num]);
			}
		}
	}
	
	return image;
}


std::pair<std::vector<std::vector<std::vector<int>>>, std::vector<bool>> TusimpleEngine::process_output(const Ort::Value& tensor, const Config& cfg) {
	
	auto data = tensor_to_vec<float>(tensor);
		
	// Copy the data into the vector with the desired shape
	size_t index = 0;
	for (int i = 0; i < 101; ++i) {
		for (int j = 0; j < 56; ++j) {
			for (int k = 0; k < 4; ++k) {
				shaped_data[i][j][k] = data[index];
				if(i != 100){
					softmax_buffer[i][j][k] = data[index];
				}
				index++;
			}
		}
	}
	
	reverseAlongAxis(shaped_data, 1);
	
	
	auto& processed_output = shaped_data;
	
	
	std::vector<std::vector<std::vector<float>>>& output = processed_output; // 3D vector
	std::vector<int> max_indices(output[0].size()); // vector to store max indices
	
	for (size_t i = 0; i < output[0].size(); ++i) {
		int max_val = output[0][i][0];
		int max_idx = 0;
		for (size_t j = 1; j < output.size(); ++j) {
			if (output[j][i][0] > max_val) {
				max_val = output[j][i][0];
				max_idx = j;
			}
		}
		max_indices[i] = max_idx;
	}
	
	
	auto argmax = applyArgmaxToAxis0(processed_output);
	
	auto mask = createBooleanMask(argmax, cfg.griding_num, ComparisonType::EQUAL);
	
	auto softmaxed_data = applySoftmaxToAxis0(softmax_buffer);
	
	auto& processed_prob = softmaxed_data;
	
	std::vector<float> idx_stl;
	
	for (int i = 1; i <= cfg.griding_num; ++i) {
		idx_stl.push_back(static_cast<float>(i));
	}
	
	std::vector<std::vector<float>> loc(4, std::vector<float>(56, 0.0f));
	
	// Perform the weighted sum
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 56; ++j) {
			float weight = 0.0f;
			
			for (int k = 0; k < 100; ++k) {
				weight += processed_prob[k][j][i] * idx_stl[k];
			}
			loc[i][j] += weight;
			
		}
	}
	
	
	applyBooleanMask(loc, mask, 0);
	
	int start = 0;
	int end = 800 - 1;
	
	// Create a vector to store the equally spaced values
	std::vector<float> col_sample;
	
	for (int i = 0; i < cfg.griding_num; ++i) {
		float value = static_cast<float>(start + i * (end - start) / (cfg.griding_num - 1));
		col_sample.push_back(value);
	}
	
	auto col_sample_w = col_sample[1] - col_sample[0];
	
	std::vector<std::vector<std::vector<int>>> lanes_points;
	std::vector<bool> lanes_detected;
	
	auto output_shape = tensor.GetTensorTypeAndShapeInfo().GetShape();
	
	size_t max_lanes = output_shape[3];
	
	for (int lane_num = 0; lane_num < max_lanes; ++lane_num) {
		std::vector<std::vector<int>> lane_points;
		
		auto lane = loc[lane_num];
		int nonZeroCount = 0;
		
		for (int point_num = 0; point_num < output_shape[1]; ++point_num) {
			for (int i = 0; i < lane.size(); ++i) {
				if (lane[i] != 0) {
					nonZeroCount++;
				}
			}
		}
		
		if(nonZeroCount > 2){
			lanes_detected.push_back(true);
			for (int point_num = 0; point_num < output_shape[1]; ++point_num) {
				if (lane[point_num] > 0 && point_num < cfg.cls_num_per_lane) {
					int lane_point_x = static_cast<int>(lane[point_num] * col_sample_w * cfg.img_w / 800) - 1;
					int lane_point_y = static_cast<int>(cfg.img_h * (cfg.row_anchor[cfg.cls_num_per_lane - 1 - point_num] / 288)) - 1;
					lane_points.push_back({lane_point_x, lane_point_y});
				}
			}
		} else {
			lanes_detected.push_back(false);
		}
		lanes_points.push_back(lane_points);
	}
	return {lanes_points, lanes_detected};
}

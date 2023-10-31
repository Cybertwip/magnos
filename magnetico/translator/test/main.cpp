#include <algorithm>  // std::generate
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <numeric>
#include <onnxruntime_cxx_api.h>

#define STB_IMAGE_IMPLEMENTATION  // This should be defined in one of your source files.
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

#include "MLUtils.h"

struct Image {
	int width;
	int height;
	int channels;
	std::vector<float> data;
};

Image resizeImage(Image& image, int new_width, int new_height);

template <typename T>
Ort::Value vec_to_tensor(std::vector<T>& data, const std::vector<std::int64_t>& shape) {
	Ort::MemoryInfo mem_info =
	Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
	auto tensor = Ort::Value::CreateTensor<T>(mem_info, data.data(), data.size(), shape.data(), shape.size());
	return tensor;
}

template <typename T>
std::vector<T> tensor_to_vec(const Ort::Value& tensor){
	return std::vector(tensor.GetTensorData<T>(), tensor.GetTensorData<T>() + tensor.GetTensorTypeAndShapeInfo().GetElementCount());
}

class OnnxEngine {
public:
	OnnxEngine(const std::string& model_path):
	env_(Ort::Env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeEnv")),
	session_(Ort::Session(env_, model_path.c_str(), session_options_)){

	}
	
	// Perform inference and return the output tensor
protected:
	std::vector<Ort::Value> inference(const std::vector<std::vector<float>>& input_data, const std::vector<std::vector<int64_t>>& input_shape) {
		if (!session_) {
			std::cerr << "Model not loaded. Call loadModel() first." << std::endl;
			return {};
		}
		
		Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
		
		std::vector<Ort::Value> input_tensors;
		
		
		for(size_t i = 0; i<input_data.size(); ++i){
			auto& data = input_data[i];
			auto& shape = input_shape[i];
			input_tensors.push_back(vec_to_tensor(const_cast<std::vector<float>&>(data), shape));
		}

		
		try {
			
			std::vector<std::string> input_names;
			std::vector<int64_t> input_shapes;
			
			
			for (std::size_t i = 0; i < session_.GetInputCount(); i++) {
				input_names.emplace_back(session_.GetInputNameAllocated(i, allocator_).get());
				input_shapes = session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
			}
			
			std::vector<const char*> input_names_char(input_names.size(), nullptr);
			std::transform(std::begin(input_names), std::end(input_names), std::begin(input_names_char),
						   [&](const std::string& str) { return str.c_str(); });
			
			
			std::vector<std::string> output_names;
			for (std::size_t i = 0; i < session_.GetOutputCount(); i++) {
				output_names.emplace_back(session_.GetOutputNameAllocated(i, allocator_).get());
			}

			std::vector<const char*> output_names_char(output_names.size(), nullptr);
			std::transform(std::begin(output_names), std::end(output_names), std::begin(output_names_char),
						   [&](const std::string& str) { return str.c_str(); });
			
			auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, input_names_char.data(), input_tensors.data(),
			    input_names_char.size(),
				output_names_char.data(),
				output_names_char.size());
			
			return output_tensors;
		} catch (const Ort::Exception& exception) {
			std::cerr << "Error during inference: " << exception.what() << std::endl;
			return {};
		}
		
		return {};
	}
	
private:
	Ort::Env env_;
	Ort::AllocatorWithDefaultOptions allocator_;
	Ort::SessionOptions session_options_;
	
protected:
	Ort::Session session_;
};

class TusimpleEngine : public OnnxEngine {
public:
	TusimpleEngine(const std::string& model_file) : OnnxEngine(model_file) {
			
		cfg.in_w = 800;
		cfg.in_h = 288;
		
		std::vector<float> row_anchor(tusimple_row_anchor, tusimple_row_anchor + sizeof(tusimple_row_anchor) / sizeof(tusimple_row_anchor[0]));
		
		cfg.img_w = 1280;
		cfg.img_h = 720;
		
		cfg.row_anchor = row_anchor;
		cfg.griding_num = 100;
		cfg.cls_num_per_lane = 56;
	}
	
	std::vector<float> prepareInputImage(Image& image, const std::vector<float>& mean, const std::vector<float>& std) {
		
		std::vector<unsigned char> data;
		
		for(auto color : image.data){
			data.push_back(static_cast<int>(color));
		}
		
		// Apply preprocessing and flatten the original image into a 1D vector
		std::vector<float> processed_tensor;
		
		size_t counter = 0;
		for(size_t i = 0; i<data.size(); ++i){
			int color = data[i];
			float pixel_value = (static_cast<float>(color) / 255.0 - mean[counter % 3]) / std[counter%3];
			
			processed_tensor.push_back(pixel_value);
			counter++;
		}
		
		
		std::vector<std::vector<float>> channeledData(3);
		
		counter = 0;
		for(size_t i = 0; i<processed_tensor.size(); ++i){
			channeledData[counter % 3].push_back(processed_tensor[i]);
			counter++;
		}
		std::vector<std::vector<std::vector<float>>> channeledDataWithNewDim(1, channeledData);
		
		// Flatten the data
		std::vector<float> flattenedData;
		for (int channel = 0; channel < 3; ++channel) {
			for (float value : channeledData[channel]) {
				flattenedData.push_back(value);
			}
		}
		
		return flattenedData;
	}

	std::vector<float> detectLanes(Image& image){
		// Create the input tensor
		std::vector<int64_t> input_shape = {1, 3, cfg.in_h, cfg.in_w};

		auto scaledImage = resizeImage(image, cfg.in_w, cfg.in_h);
		
		// Define mean and std values
		std::vector<float> mean = {0.485, 0.456, 0.406};
		std::vector<float> std = {0.229, 0.224, 0.225};
				
		auto inputTensor = prepareInputImage(scaledImage, mean, std);
		
		auto output = inference({inputTensor}, {input_shape});

		auto [lanes_points, lanes_detected] = process_output(output[0], cfg);

		std::vector<std::uint8_t> uint8Vector;

		auto scaled_image =
		resizeImage(image, cfg.img_w, cfg.img_h);

		for (float value : scaled_image.data) {
			std::uint8_t uint8Value = static_cast<std::uint8_t>(value);
			uint8Vector.push_back(uint8Value);
		}

		
		drawLanes(uint8Vector, lanes_points, lanes_detected, cfg, true);
		
//		return visualization_image;

		return {};
	}
	
private:
	struct Config {
		unsigned int griding_num;
		int in_w;
		int in_h;
		int img_w;
		int img_h;
		int cls_num_per_lane;
		std::vector<float> row_anchor;
	};
	
	
	Config cfg;
	
	// Define the lane colors
	std::vector<std::vector<uint8_t>> laneColors = {
		{0,0,255},
		{0,255,0},
		{255,0,0},
		{0,255,255}
	};
		
	static constexpr float tusimple_row_anchor[] = {64,  68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112,
		116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164,
		168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216,
		220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268,
		272, 276, 280, 284};

	
private:
	
	void drawFilledCircle(std::vector<std::uint8_t>& image_data, int image_width, int image_height, int center_x, int center_y, int radius, const std::vector<uint8_t>& color) {
		for (int y = -radius; y <= radius; y++) {
			for (int x = -radius; x <= radius; x++) {
				if (x * x + y * y <= radius * radius) {
					int row = center_y + y;
					int col = center_x + x;
					
					if (row >= 0 && row < image_height && col >= 0 && col < image_width) {
						int index = 3 * (row * image_width + col);
						image_data[index] = color[0];
						image_data[index + 1] = color[1];
						image_data[index + 2] = color[2];
					}
				}
			}
		}
	}
	
	void drawLanes(const std::vector<std::uint8_t>& input_data, const std::vector<std::vector<std::vector<int>>>& lanes_points, const std::vector<bool>& lanes_detected, const Config& cfg, bool saveImage) {
		// Create an image with 3 channels (RGB) and allocate memory.
		int image_width = cfg.img_w;
		int image_height = cfg.img_h;
		std::vector<std::uint8_t> image_data = input_data;
		
		for (size_t lane_num = 0; lane_num < lanes_detected.size(); ++lane_num) {
			if (lanes_detected[lane_num]) {
				for (const auto& lane_point : lanes_points[lane_num]) {
					int col = lane_point[0];
					int row = lane_point[1];
					
					// Draw a larger dot at the lane point
					drawFilledCircle(image_data, image_width, image_height, col, row, 10, laneColors[lane_num]);
				}
			}
		}
		
		std::string workPath = MODELS_PATH;
		auto imagePath = workPath + "output.png";
		// Save the image as a PNG file if required.
		if (saveImage) {
			stbi_write_png(imagePath.c_str(), image_width, image_height, 3, image_data.data(), 0);
		}
	}
	
	std::pair<std::vector<std::vector<std::vector<int>>>, std::vector<bool>> process_output(const Ort::Value& tensor, const Config& cfg) {
		
		auto data = tensor_to_vec<float>(tensor);

		std::vector<std::vector<std::vector<float>>> shaped_data(101, std::vector<std::vector<float>>(56, std::vector<float>(4)));
		
		// Copy the data into the vector with the desired shape
		size_t index = 0;
		for (int i = 0; i < 101; ++i) {
			for (int j = 0; j < 56; ++j) {
				for (int k = 0; k < 4; ++k) {
					shaped_data[i][j][k] = data[index];
					
					index++;
				}
			}
		}

		reverseAlongAxis(shaped_data, 1);

		auto processed_output = shaped_data;
		shaped_data.pop_back();
		
		auto softmaxed_data = applySoftmaxToAxis0(shaped_data);
		
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


		std::vector<std::vector<std::vector<float>>> output = processed_output; // 3D vector
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
		
		auto mask = createBooleanMask(argmax, cfg.griding_num);
		
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
	
};

Image resizeImage(Image& image, int new_width, int new_height) {
	
	int width = image.width;
	int height = image.height;
	int channels = image.channels;
	auto& input_image = image.data;
	
	std::vector<unsigned char> output_buffer(new_width * new_height * channels, 0.0f);
	
	
	std::vector<unsigned char> input_buffer;
	
	for(auto color : input_image){
		input_buffer.push_back(static_cast<int8_t>(color));
	}
	
	stbir_resize_uint8_linear(input_buffer.data(), width, height, 0, output_buffer.data(), new_width, new_height, 0, (stbir_pixel_layout) channels);
	
	
	std::vector<float> resized_image;
	
	
	for(auto color : output_buffer){
		resized_image.push_back(static_cast<float>(color));
	}
	
	return Image { new_width, new_height, channels, resized_image };
}

Image load_image_data(const std::string& image_path) {
	int width, height, channels;
	stbi_uc* image_data = stbi_load(image_path.c_str(), &width, &height, &channels, STBI_rgb);
	
	if (!image_data) {
		std::cerr << "Failed to load the image." << std::endl;
		exit(EXIT_FAILURE);
	}
	
	// Create a vector to hold image data
	std::vector<float> input_data(image_data, image_data + width * height * channels);
	
	// Ensure proper memory deallocation
	stbi_image_free(image_data);
	
	return Image { width, height, channels, input_data };
}

int main() {
	std::string workPath = MODELS_PATH; // You need to define MODELS_PATH
	
	std::string model_file = workPath + "fast_roadseg-tusimple.onnx";
	
	TusimpleEngine engine(model_file);
	
	// Load an input image
	std::string image_path = workPath + "input.jpg";
	
	auto input_data = load_image_data(image_path);
	
	try {
		auto output_tensors = engine.detectLanes(input_data);
		
		std::cout << "Done!" << std::endl;
		
	} catch (const Ort::Exception& exception) {
		std::cout << "ERROR running model inference: " << exception.what() << std::endl;
		exit(-1);
	}
	
	return 0;
}

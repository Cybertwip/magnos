#include "HybridnetsEngine.h"

HybridnetsEngine::HybridnetsEngine(int channels) : OnnxEngine(std::string{MODELS_PATH} + "hybridnets/384x512/roadseg.onnx") {
			
	cfg.img_w = 1280;
	cfg.img_h = 720;
	
	cfg.in_w = 512;
	cfg.in_h = 384;
	
	shaped_data = std::vector<std::vector<std::vector<float>>> (3, std::vector<std::vector<float>>(cfg.in_h, std::vector<float>(cfg.in_w)));

	
	channeledDataBuffer[0].resize(cfg.in_w * cfg.in_h);
	channeledDataBuffer[1].resize(cfg.in_w * cfg.in_h);
	channeledDataBuffer[2].resize(cfg.in_w * cfg.in_h);
	
	processedTensorBuffer = std::vector<float>(cfg.in_w * cfg.in_h * 3);
	
	outputDataBuffer = std::make_unique<Image>(cfg.in_w, cfg.in_h, channels, std::vector<uint8_t>(cfg.in_w * cfg.in_h * 3));

}

std::vector<float> HybridnetsEngine::prepareInputImage(Image& image, const std::vector<float>& mean, const std::vector<float>& std) {
	
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

std::tuple<Image> HybridnetsEngine::detectLanes(Image& image){
	// Create the input tensor
	std::vector<int64_t> input_shape = {1, 3, cfg.in_h, cfg.in_w};
	
	auto scaledImage = resizeImage(image, cfg.in_w, cfg.in_h);
	
	// Define mean and std values
	std::vector<float> mean = {0.485, 0.456, 0.406};
	std::vector<float> std = {0.229, 0.224, 0.225};
	
	auto inputTensor = prepareInputImage(scaledImage, mean, std);
	
	auto output = inference({inputTensor}, {input_shape});
	
	auto [segmentation_map] = process_output(output[2], cfg);
	
	auto mask = createBooleanMask(segmentation_map, 0, ComparisonType::GREATER);
	
	auto appliedMask = getAppliedBooleanMaskAxis1(segmentation_map, mask);
	
	auto columns = scaledImage.rows();
	
	auto imageMask = getAppliedBooleanMaskIndicesAxis1(columns, mask);

	memset(outputDataBuffer->data.data(), 255, outputDataBuffer->data.size());
	
	memcpy(outputDataBuffer->data.data(), scaledImage.data.data(), scaledImage.data.size());

	for(size_t i = 0; i<imageMask.size(); ++i){
		auto [index_r, index_c] = imageMask[i];
		
		for(size_t channel = 0; channel < image.channels; ++channel){
			(*outputDataBuffer)[index_r][index_c + channel] =
			segmentationColors[appliedMask[i]][channel];
		}
	}
	
	auto scaledBuffer = resizeImage(*outputDataBuffer, image.width, image.height);
	
	return std::make_tuple(scaledBuffer);
}

std::tuple<std::vector<std::vector<std::size_t>>> HybridnetsEngine::process_output(const Ort::Value& tensor, const Config& cfg) {
	
	auto data = tensor_to_vec<float>(tensor);
	
	// Copy the data into the vector with the desired shape
	size_t index = 0;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < cfg.in_h; ++j) {
			for (int k = 0; k < cfg.in_w; ++k) {
				shaped_data[i][j][k] = data[index];
				index++;
			}
		}
	}
	
	auto argmax = applyArgmaxToAxis0(shaped_data);
	
	return {argmax};
}

#include "DepthEngine.h"

DepthEngine::DepthEngine(int channels) : OnnxEngine(std::string{MODELS_PATH} + "depth/384x640/estimator.onnx") {
			
	cfg.img_w = 1714;
	cfg.img_h = 914;
	
	cfg.in_w = 640;
	cfg.in_h = 384;
	
	cfg.min_depth = 0.0f;
	cfg.max_depth = 5.0f;
	
	channeledDataBuffer[0].resize(cfg.in_w * cfg.in_h);
	channeledDataBuffer[1].resize(cfg.in_w * cfg.in_h);
	channeledDataBuffer[2].resize(cfg.in_w * cfg.in_h);
	
	processedTensorBuffer = std::vector<float>(cfg.in_w * cfg.in_h * 3);
	
	outputDepthBuffer = std::make_unique<Image>(cfg.in_w, cfg.in_h, 1, std::vector<uint8_t>(cfg.in_w * cfg.in_h * 1));

	outputDataBuffer = std::make_unique<Image>(cfg.in_w, cfg.in_h, channels, std::vector<uint8_t>(cfg.in_w * cfg.in_h * channels));

}

std::vector<float> DepthEngine::prepareInputImage(Image& image) {
	
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
		float pixel_value = (static_cast<float>(color) / 255.0);
		
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

std::tuple<Image> DepthEngine::detectDepth(Image& image){
	// Create the input tensor
	std::vector<int64_t> input_shape = {1, 3, cfg.in_h, cfg.in_w};
	
	auto scaledImage = resizeImage(image, cfg.in_w, cfg.in_h);
		
	auto inputTensor = prepareInputImage(scaledImage);
	
	auto output = inference({inputTensor}, {input_shape});
	

	process_output(output[0], cfg);


	auto scaledBuffer = resizeImage(*outputDepthBuffer, image.width, image.height);
	
	return std::make_tuple(scaledBuffer);
}

void  DepthEngine::process_output(const Ort::Value& tensor, const Config& cfg) {
	
	auto data = tensor_to_vec<float>(tensor);
	
	// Copy the data into the vector with the desired shape
	size_t index = 0;
	int vertex_idx = 0;

	for (int k = 0; k < cfg.in_h; ++k) {
		for (int j = 0; j < cfg.in_w; ++j) {
			float depthValue = 255.0f * (data[index] - cfg.min_depth) / (cfg.max_depth - cfg.min_depth);
			
			if(depthValue < cfg.min_depth){
				depthValue = 0;
			}
			
			(*outputDepthBuffer)[k][j] = depthValue;
			if (_isnan(depthValue))
			{
				cloud.vertices.push_back(VertexType(0, 0, depthValue));
				continue;
			}
			float z = depthValue; // Depth value for pixel (j, k)
			float x = (j - kCx) * z / kFx;
			float y = (k - kCy) * z / kFy;

			cloud.vertices.push_back(VertexType(x, y, z));

			index++;
		}
	}
	
}

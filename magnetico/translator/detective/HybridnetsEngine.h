#pragma once

#include "OnnxEngine.h"

#include "StbUtils.h"
#include "MLUtils.h"

#include <memory>

class HybridnetsEngine : public OnnxEngine {
public:
	HybridnetsEngine(int channels);
	
	std::tuple<Image> detectLanes(Image& image);
private:
	std::vector<float> prepareInputImage(Image& image, const std::vector<float>& mean, const std::vector<float>& std);
	
	struct Config {
		int in_w;
		int in_h;
		int img_w;
		int img_h;
		float conf_thres;
		float iou_thres;
		std::vector<float> anchors;
	};
	
	std::vector<std::vector<uint8_t>> segmentationColors = {
		{0,    	0,     0, 255},
		{0,  	0,   191, 255},
		{192,   0,   0, 255}
	};
	
	Config cfg;
	
private:
	std::tuple<std::vector<std::vector<std::size_t>>> process_output(const Ort::Value& tensor, const Config& cfg);
	
private:
	std::vector<std::vector<std::vector<float>>> shaped_data;
		
	std::vector<std::vector<float>> channeledDataBuffer = std::vector<std::vector<float>>(3);
	
	std::unique_ptr<Image> outputDataBuffer;

	std::vector<float> processedTensorBuffer;
	
};

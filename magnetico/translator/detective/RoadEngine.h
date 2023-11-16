#pragma once

#include "OnnxEngine.h"

#include "StbUtils.h"
#include "MLUtils.h"

#include <memory>

class ImagePointCloud;

class RoadEngine {
public:
	RoadEngine();
	
	std::tuple<Image> detectLanes(ImagePointCloud& image);
private:
	
	struct Config {
		int img_w;
		int img_h;
	};
	
	Config cfg;
	
private:
		
	std::unique_ptr<Image> outputDataBuffer;
	
};

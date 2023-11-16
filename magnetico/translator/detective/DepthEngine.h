#pragma once

#include "OnnxEngine.h"

#include "StbUtils.h"
#include "MLUtils.h"

#include <Eigen/Eigen>

#include <memory>


typedef Eigen::Vector3d VertexType;
typedef Eigen::Vector2i PixelPos;

const int kNeighborRange = 5; // boundary pixels' neighbor range
const int kScaleFactor = 5; // scale coordinate unit in mm
const float kInfVal = 1000000; // an infinite large value used in MRF optimization

// Camera intrinsic parameters.
// All BundleFusion data uses the following parameters.
const double kFx = 720;
const double kFy = 720;
const double kCx = 640;
const double kCy = 360;

class DepthEngine : public OnnxEngine {
public:
	DepthEngine(int channels);
	
	std::tuple<Image> detectDepth(Image& image);
private:
	std::vector<float> prepareInputImage(Image& image);
	
	struct Config {
		int in_w;
		int in_h;
		int img_w;
		int img_h;
		float min_depth;
		float max_depth;
	};
		
	Config cfg;
	
private:
	void process_output(const Ort::Value& tensor, const Config& cfg);

private:

	std::vector<std::vector<float>> channeledDataBuffer = std::vector<std::vector<float>>(3);
	
	std::unique_ptr<Image> outputDepthBuffer;

	std::unique_ptr<Image> outputDataBuffer;

	std::vector<float> processedTensorBuffer;
	
};

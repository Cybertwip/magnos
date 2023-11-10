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
const double kFx = 640;
const double kFy = 384;
const double kCx = 640;
const double kCy = 384;

#if defined(__linux__) || defined(__APPLE__)
#define _isnan(x) isnan(x)
#endif

struct ImagePointCloud
{
	std::vector<VertexType> vertices; // 3D vertices
	int w, h;
	
	inline int width() const { return w; }
	inline int height() const { return h; }
	inline bool get(const int row, const int col, double &x, double &y, double &z) const {
		const int pixIdx = row * w + col;
		z = vertices[pixIdx][2];
		// Remove points with 0 or invalid depth in case they are detected as a plane
		if (z == 0 || _isnan(z)) return false;
		x = vertices[pixIdx][0];
		y = vertices[pixIdx][1];
		return true;
	}
};


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
//	@TODO
//	/************************************************************************/
//	/* For MRF optimization */
//	inline MRF::CostVal dCost(int pix, int label)
//	{
//		return pixel_boundary_flags_[pix] ? 1 : (label == plane_filter.membershipImg.at<int>(pix / kDepthWidth, pix % kDepthWidth) ? 1 : kInfVal);
//	}
//	inline MRF::CostVal fnCost(int pix1, int pix2, int i, int j)
//	{
//		int gray1 = pixel_grayval_[pix1], gray2 = pixel_grayval_[pix2];
//		return i == j ? 0 : exp(-MRF::CostVal(gray1 - gray2) * (gray1 - gray2) / 900); // 900 = sigma^2 by default
//	}
//	/************************************************************************/

private:
	ImagePointCloud cloud;
	std::vector<std::vector<int>> plane_vertices_; // vertex indices each plane contains

	std::vector<std::vector<float>> channeledDataBuffer = std::vector<std::vector<float>>(3);
	
	std::unique_ptr<Image> outputDepthBuffer;

	std::unique_ptr<Image> outputDataBuffer;

	std::vector<float> processedTensorBuffer;
	
};

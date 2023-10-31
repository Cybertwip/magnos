#pragma once

#include "OnnxEngine.h"

#include "StbUtils.h"
#include "MLUtils.h"

class TusimpleEngine : public OnnxEngine {
public:
	TusimpleEngine(const std::string& model_file);
		
	Image detectLanes(Image& image);
	
private:
	std::vector<float> prepareInputImage(Image& image, const std::vector<float>& mean, const std::vector<float>& std);

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
	
	void drawFilledCircle(std::vector<std::uint8_t>& image_data, int image_width, int image_height, int center_x, int center_y, int radius, const std::vector<uint8_t>& color);
	
	Image drawLanes(Image& image, const std::vector<std::vector<std::vector<int>>>& lanes_points, const std::vector<bool>& lanes_detected, const Config& cfg, bool saveToFile);
	
	std::pair<std::vector<std::vector<std::vector<int>>>, std::vector<bool>> process_output(const Ort::Value& tensor, const Config& cfg);
};

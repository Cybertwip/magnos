#pragma once

#include <vector>

struct Image {
	int width;
	int height;
	int channels;
	std::vector<uint8_t> data;
};

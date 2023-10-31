#pragma once

#include "Image.h"

#include <string>

Image resizeImage(Image& image, int new_width, int new_height);

Image load_image_data(const std::string& image_path);

void saveImage(Image& image, const std::string& path);

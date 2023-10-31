#include "StbUtils.h"

#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

#include <iostream>

Image resizeImage(Image& image, int new_width, int new_height) {
	
	int width = image.width;
	int height = image.height;
	int channels = image.channels;
	auto& input_image = image.data;
	
	std::vector<uint8_t> output_buffer(new_width * new_height * channels, 0.0f);
	
	stbir_resize_uint8_linear(input_image.data(), width, height, 0, output_buffer.data(), new_width, new_height, 0, (stbir_pixel_layout) channels);
	
	
	std::vector<uint8_t> resized_image;
	
	for(auto color : output_buffer){
		resized_image.push_back((color));
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
	std::vector<uint8_t> input_data(image_data, image_data + width * height * channels);
	
	// Ensure proper memory deallocation
	stbi_image_free(image_data);
	
	return Image { width, height, channels, input_data };
}

void saveImage(Image& image, const std::string& path){
	std::vector<std::uint8_t>& uint8Vector = image.data;

	stbi_write_png(path.c_str(), image.width, image.height, image.channels, uint8Vector.data(), 0);
}

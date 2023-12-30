#include "StbUtils.h"

#include "stb_image.h"

#include "stb_image_write.h"

#include "stb_image_resize2.h"

#include <iostream>


static std::vector<uint8_t> output_buffer;
static std::vector<uint8_t> input_buffer;


Image resizeImage(Image& image, int new_width, int new_height) {
	
	int width = image.width;
	int height = image.height;
	int channels = image.channels;
	auto& input_image = image.data;
	
	if(output_buffer.size() != new_width * new_height * channels){
		output_buffer.resize(new_width * new_height * channels);
	}
	
	stbir_resize_uint8_linear(input_image.data(), width, height, 0, output_buffer.data(), new_width, new_height, 0, (stbir_pixel_layout) channels);
	
	return Image { new_width, new_height, channels, output_buffer };
}


Image load_image_data(const std::string& image_path) {
	int width, height, channels;
	stbi_uc* image_data = stbi_load(image_path.c_str(), &width, &height, &channels, STBI_rgb);
	
	if (!image_data) {
		std::cerr << "Failed to load the image." << std::endl;
		exit(EXIT_FAILURE);
	}
		
	if(input_buffer.size() != width * height * channels){
		input_buffer.resize(width * height * channels);
	}
	
	memcpy(input_buffer.data(), image_data, input_buffer.size());
	
	// Ensure proper memory deallocation
	stbi_image_free(image_data);
	
	return Image { width, height, channels, input_buffer };
}

void saveImage(Image& image, const std::string& path){
	std::vector<std::uint8_t>& uint8Vector = image.data;

	stbi_write_png(path.c_str(), image.width, image.height, image.channels, uint8Vector.data(), 0);
}

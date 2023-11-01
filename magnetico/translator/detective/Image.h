#pragma once

#include <vector>

struct Image {
	int width;
	int height;
	int channels;
	
	struct Row {
		Image& parent;
		std::size_t row;
		
		Row(Image& p, size_t r) : parent(p), row(r) {}
		
		uint8_t& operator[](std::size_t col) {
			return parent.data[(row * parent.width + col) * parent.channels];
		}
	};
	
	std::vector<uint8_t> data;
	
	Image(int w, int h, int c, const std::vector<uint8_t>& input) : width(w), height(h), channels(c) {
		// Ensure data vector is properly sized (width x height x channels)
		data.resize(width * height * channels);
		
		memcpy(data.data(), input.data(), input.size());
		
		data3DBuffer = std::vector<std::vector<std::vector<uint8_t>>>(height, std::vector<std::vector<uint8_t>>(width, std::vector<uint8_t>(channels)));
		
		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				for (int k = 0; k < channels; ++k) {
					data3DBuffer[i][j][k] = data[(i * width + j) * channels + k];
				}
			}
		}
	}

	Image(const Image& other) : width(other.width), height(other.height), channels(other.channels) {
		data = other.data; // Copy the data
		data3DBuffer = other.data3DBuffer;
	}

	
	// Function to get a row
	Row operator[](std::size_t row) {
		if (row >= 0 && row < height) {
			return Row(*this, row);
		} else {
			// Handle out-of-bounds row access here
			throw std::out_of_range("Row index out of bounds");
		}
	}
	
	std::vector<std::vector<std::vector<uint8_t>>>& rows() {
		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				for (int k = 0; k < channels; ++k) {
					data3DBuffer[i][j][k] = data[(i * width + j) * channels + k];
				}
			}
		}
		return data3DBuffer;
	}
	
private:
	std::vector<std::vector<std::vector<uint8_t>>> data3DBuffer;

};

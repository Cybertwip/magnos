#pragma once

#include <string>
#include <vector>

class DataDecompressor {
public:
	DataDecompressor(const std::string& compressedFilename);
	bool decompressData(const std::string& compressedFilename);
};


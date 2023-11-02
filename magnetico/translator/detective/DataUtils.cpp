#include "DataUtils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "cereal/thirdparty/base64.hpp"

DataDecompressor::DataDecompressor(const std::string& compressedFilename) {
	decompressData(compressedFilename);
}

bool DataDecompressor::decompressData(const std::string& compressedFilename) {
	
	if(std::filesystem::exists(compressedFilename)){
		return true;
	}
	// Read the index file
	std::vector<std::pair<int, int>> index;
	std::ifstream indexFile(compressedFilename + "." + "bin");
	if (!indexFile.is_open()) {
		std::cerr << "Error: Failed to open the index file." << std::endl;
		return false;
	}
	
	std::string line;
	while (std::getline(indexFile, line)) {
		if (line.empty()) {
			continue;
		}
		
		int start, length;
		if (sscanf(line.c_str(), "%d,%d", &start, &length) == 2) {
			index.push_back(std::make_pair(start, length));
		}
	}
	indexFile.close();
	
	// Decompress and reconstruct the data
	std::ofstream outputFile(compressedFilename, std::ios::binary);
	if (!outputFile.is_open()) {
		std::cerr << "Error: Failed to create the output file." << std::endl;
		return false;
	}
	
	for (const auto& entry : index) {
		int start = entry.first;
		int length = entry.second;
		std::ifstream compressedChunkFile(compressedFilename + "." + std::to_string(start) + "." + "bin", std::ios::binary);
		if (!compressedChunkFile.is_open()) {
			std::cerr << "Error: Failed to open a compressed chunk file." << std::endl;
			return false;
		}
		
		std::string compressedData;
		compressedData.resize(length);
		compressedChunkFile.read(&compressedData[0], length);
		compressedChunkFile.close();
		
		std::string decodedData = cereal::base64::decode(compressedData);
		outputFile.write(decodedData.c_str(), decodedData.size());
	}
	
	outputFile.close();
	return true;
}

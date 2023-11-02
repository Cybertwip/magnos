#pragma once

#include "OnnxUtils.h"

#include "DataUtils.h"


class OnnxEngine {
public:
	OnnxEngine(const std::string& model_path);
	
protected:
	// Perform inference and return the output tensor
	std::vector<Ort::Value> inference(const std::vector<std::vector<float>>& input_data, const std::vector<std::vector<int64_t>>& input_shape);
	
private:
	Ort::Env env_;
	Ort::AllocatorWithDefaultOptions allocator_;
	Ort::SessionOptions session_options_;
	DataDecompressor decompressor;
	
protected:
	Ort::Session session_;
};

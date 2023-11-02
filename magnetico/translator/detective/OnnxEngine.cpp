#include "OnnxEngine.h"

#include <iostream>

OnnxEngine::OnnxEngine(const std::string& model_path):
env_(Ort::Env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeEnv")),
decompressor(model_path),
session_(Ort::Session(env_, model_path.c_str(), session_options_)){
}

std::vector<Ort::Value> OnnxEngine::inference(const std::vector<std::vector<float>>& input_data, const std::vector<std::vector<int64_t>>& input_shape) {
	if (!session_) {
		std::cerr << "Model not loaded. Call loadModel() first." << std::endl;
		return {};
	}
	
	Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
	
	std::vector<Ort::Value> input_tensors;
	
	
	for(size_t i = 0; i<input_data.size(); ++i){
		auto& data = input_data[i];
		auto& shape = input_shape[i];
		input_tensors.push_back(vec_to_tensor(const_cast<std::vector<float>&>(data), shape));
	}
	
	
	try {
		
		std::vector<std::string> input_names;
		std::vector<int64_t> input_shapes;
		
		
		for (std::size_t i = 0; i < session_.GetInputCount(); i++) {
			input_names.emplace_back(session_.GetInputNameAllocated(i, allocator_).get());
			input_shapes = session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
		}
		
		std::vector<const char*> input_names_char(input_names.size(), nullptr);
		std::transform(std::begin(input_names), std::end(input_names), std::begin(input_names_char),
					   [&](const std::string& str) { return str.c_str(); });
		
		
		std::vector<std::string> output_names;
		for (std::size_t i = 0; i < session_.GetOutputCount(); i++) {
			output_names.emplace_back(session_.GetOutputNameAllocated(i, allocator_).get());
		}
		
		std::vector<const char*> output_names_char(output_names.size(), nullptr);
		std::transform(std::begin(output_names), std::end(output_names), std::begin(output_names_char),
					   [&](const std::string& str) { return str.c_str(); });
		
		auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, input_names_char.data(), input_tensors.data(),
										   input_names_char.size(),
										   output_names_char.data(),
										   output_names_char.size());
		
		return output_tensors;
	} catch (const Ort::Exception& exception) {
		std::cerr << "Error during inference: " << exception.what() << std::endl;
		return {};
	}
	
	return {};
}

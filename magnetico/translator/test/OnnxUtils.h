#pragma once

#include <algorithm>
#include <vector>
#include <onnxruntime_cxx_api.h>

template <typename T>
Ort::Value vec_to_tensor(std::vector<T>& data, const std::vector<std::int64_t>& shape) {
	Ort::MemoryInfo mem_info =
	Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
	auto tensor = Ort::Value::CreateTensor<T>(mem_info, data.data(), data.size(), shape.data(), shape.size());
	return tensor;
}

template <typename T>
std::vector<T> tensor_to_vec(const Ort::Value& tensor){
	return std::vector(tensor.GetTensorData<T>(), tensor.GetTensorData<T>() + tensor.GetTensorTypeAndShapeInfo().GetElementCount());
}

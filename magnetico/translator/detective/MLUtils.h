#pragma once

#include <algorithm>
#include <cmath>
#include <cassert>

template <typename T>
std::size_t argmax(const std::vector<T>& vec) {
    auto max_iter = std::max_element(vec.begin(), vec.end());
    assert(max_iter != vec.end()); // Ensure the vector is not empty
    return static_cast<std::size_t>(std::distance(vec.begin(), max_iter));
}

std::vector<float> softmax(const std::vector<float>& values);

std::vector<std::vector<std::vector<float>>>
applySoftmaxToAxis0(std::vector<std::vector<std::vector<float>>>& data);

enum class ComparisonType {
	EQUAL,
	GREATER,
	LESSER
};


template<typename T>
std::vector<std::vector<bool>> createBooleanMask(
												 const std::vector<std::vector<T>>& argmax,
												 T value, ComparisonType comparisonType) {
	std::vector<std::vector<bool>> mask;
	
	for (const auto& row : argmax) {
		std::vector<bool> rowMask;
		for (const auto& val : row) {
			if(comparisonType == ComparisonType::EQUAL){
				rowMask.push_back(val == value);
			} else if(comparisonType == ComparisonType::GREATER){
				rowMask.push_back(val > value);
			} else if(comparisonType == ComparisonType::LESSER){
				rowMask.push_back(val < value);
			}
		}
		mask.push_back(rowMask);
	}
	
	return mask;
}

template<typename T>
std::vector<std::vector<bool>> createBooleanMask3D(
												 const std::vector<std::vector<T>>& argmax,
												 std::size_t value, ComparisonType comparisonType) {
	std::vector<std::vector<bool>> mask;
	
	for (const auto& x : argmax) {
		std::vector<bool> rowMask;
		for (const auto& y : x) {
			for(const auto& z : y){
				if(comparisonType == ComparisonType::EQUAL){
					rowMask.push_back(z == value);
				} else if(comparisonType == ComparisonType::GREATER){
					rowMask.push_back(z > value);
				} else if(comparisonType == ComparisonType::LESSER){
					rowMask.push_back(z < value);
				}
			}
		}
		mask.push_back(rowMask);
	}
	
	return mask;
}


template<typename T>

std::vector<std::vector<bool>> createBooleanMaskAxis1(
													  const std::vector<std::vector<T>>& argmax,
													  std::size_t value,
													  ComparisonType comparisonType) {
	std::vector<std::vector<bool>> mask(argmax[0].size(), std::vector<bool>(argmax.size()));
	
	for (size_t j = 0; j < argmax[0].size(); ++j) {
		for (size_t i = 0; i < argmax.size(); ++i) {
			if (comparisonType == ComparisonType::EQUAL) {
				mask[j][i] = (argmax[i][j] == value);
			} else if (comparisonType == ComparisonType::GREATER) {
				mask[j][i] = (argmax[i][j] > value);
			} else if (comparisonType == ComparisonType::LESSER) {
				mask[j][i] = (argmax[i][j] < value);
			}
			// Add more conditions for other comparison types if needed
		}
	}
	
	return mask;
}

template<typename T>
void applyBooleanMask(
					  std::vector<std::vector<T>>& vectorToModify,
					  const std::vector<std::vector<bool>>& mask,
					  int valueToSet) {
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[j][i]) {
				// If mask value is true, set the corresponding element to the specified value
				vectorToModify[i][j] = valueToSet;
			}
		}
	}
}



template<typename T>
void applyBooleanMaskAxis1(
					  std::vector<std::vector<T>>& vectorToModify,
					  const std::vector<std::vector<bool>>& mask,
					  int valueToSet) {
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[j][i]) {
				// If mask value is true, set the corresponding element to the specified value
				vectorToModify[i][j] = valueToSet;
			}
		}
	}
}


template <typename T>
std::vector<T> getAppliedBooleanMask(const std::vector<std::vector<T>>& vectorToModify, const std::vector<std::vector<bool>>& mask) {
	std::vector<T> result;
	
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[j][i]) {
				result.push_back(vectorToModify[i][j]);
			}
		}
	}
	
	return result;
}

template <typename T>
std::vector<T> getAppliedBooleanMaskAxis1(const std::vector<std::vector<T>>& vectorToModify, const std::vector<std::vector<bool>>& mask) {
	std::vector<T> result;
	
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[i][j]) {
				result.push_back(vectorToModify[i][j]);
			}
		}
	}
	
	return result;
}


template <typename T>
std::vector<T> getAppliedBooleanMaskAxis2(const std::vector<std::vector<T>>& vectorToModify, const std::vector<std::vector<bool>>& mask) {
	std::vector<T> result;
	
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[j][i]) {
				result.push_back(vectorToModify[i][j]);
			}
		}
	}
	
	return result;
}

template <typename T>
std::vector<std::tuple<std::size_t, std::size_t>>  getAppliedBooleanMaskIndices(const std::vector<std::vector<T>>& vectorToModify, const std::vector<std::vector<bool>>& mask) {
	std::vector<std::tuple<std::size_t, std::size_t>> result;
	
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[j][i]) {
				result.push_back(std::make_tuple(i, j));
			}
		}
	}
	
	return result;
}

template <typename T>
std::vector<std::tuple<std::size_t, std::size_t>> getAppliedBooleanMaskIndicesAxis1(const std::vector<std::vector<T>>& vectorToModify, const std::vector<std::vector<bool>>& mask) {
	std::vector<std::tuple<std::size_t, std::size_t>> result;
	
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[i][j]) {
				result.push_back(std::make_tuple(i, j));
			}
		}
	}
	
	return result;
}


template <typename T>
std::vector<std::tuple<std::size_t, std::size_t>> getAppliedBooleanMaskIndicesAxis2(const std::vector<std::vector<T>>& vectorToModify, const std::vector<std::vector<bool>>& mask) {
	std::vector<std::tuple<std::size_t, std::size_t>> result;
	
	for (std::size_t i = 0; i < vectorToModify.size(); ++i) {
		for (std::size_t j = 0; j < vectorToModify[i].size(); ++j) {
			if (mask[j][i]) {
				result.push_back(std::make_tuple(j, i));
			}
		}
	}
	
	return result;
}
std::vector<std::vector<std::size_t>> applyArgmaxToAxis0(
    std::vector<std::vector<std::vector<float>>>& data);
std::vector<std::vector<std::size_t>> applyArgmaxToAxis1(std::vector<std::vector<std::vector<float>>>& data);
std::vector<std::vector<std::size_t>> applyArgmaxToAxis2(std::vector<std::vector<std::vector<float>>>& data);

template <typename T>
void reverseAlongAxis(std::vector<std::vector<std::vector<T>>>& arr, int axis) {

    if (axis == 0) {
        std::reverse(arr.begin(), arr.end());
    } else if (axis == 1) {
        for (auto& row : arr) {
            std::reverse(row.begin(), row.end());
        }
    } else if (axis == 2) {
        for (auto& row : arr) {
            for (auto& vec : row) {
                std::reverse(vec.begin(), vec.end());
            }
        }
    }
}


template std::size_t argmax(const std::vector<int>& vec);
template std::size_t argmax(const std::vector<float>& vec);

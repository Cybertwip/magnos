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
	GREATER
};

std::vector<std::vector<bool>> createBooleanMask(
												 const std::vector<std::vector<std::size_t>>& argmax,
												 std::size_t value, ComparisonType comparisonType);
std::vector<std::vector<bool>> createBooleanMaskAxis1(
												 const std::vector<std::vector<std::size_t>>& argmax,
												 std::size_t value, ComparisonType comparisonType);


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

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

std::vector<std::vector<bool>> createBooleanMask(
    const std::vector<std::vector<std::size_t>>& argmax,
    std::size_t griding_num);

void applyBooleanMask(
    std::vector<std::vector<float>>& vectorToModify,
    const std::vector<std::vector<bool>>& mask,
    int valueToSet);

std::vector<std::vector<std::size_t>> applyArgmaxToAxis0(
    std::vector<std::vector<std::vector<float>>>& data);


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

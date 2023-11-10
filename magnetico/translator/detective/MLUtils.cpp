#include "MLUtils.h"
#include <algorithm>
#include <cmath>
#include <cassert>

std::vector<float> softmax(const std::vector<float>& values) {
    std::vector<float> softmax_values(values.size());
    float sum = 0.0f;

    for (size_t i = 0; i < values.size(); ++i) {
        softmax_values[i] = std::exp(values[i]);
        sum += softmax_values[i];
    }

    for (size_t i = 0; i < values.size(); ++i) {
        softmax_values[i] /= sum;
    }

    return softmax_values;
}

std::vector<std::vector<std::vector<float>>>
applySoftmaxToAxis0(std::vector<std::vector<std::vector<float>>>& data) {
    // Ensure the input data is not empty
    if (data.empty()) {
        return data;
    }

    // Get the dimensions of the input data
    const size_t num_i = data.size();
    const size_t num_j = data[0].size();
    const size_t num_k = data[0][0].size();

    // Initialize the output vector with zeros
    std::vector<std::vector<std::vector<float>>>
    softmaxed_data(num_i, std::vector<std::vector<float>>(num_j, std::vector<float>(num_k, 0.0f)));

    // Apply softmax to the values along axis 0
    for (size_t j = 0; j < num_j; ++j) {
        for (size_t k = 0; k < num_k; ++k) {
            // Collect the values to apply softmax to
            std::vector<float> values_to_softmax(num_i);
            for (size_t i = 0; i < num_i; ++i) {
                values_to_softmax[i] = data[i][j][k];
            }

            // Apply softmax to the values
            float sum_exp = 0.0f;
            for (size_t i = 0; i < num_i; ++i) {
                softmaxed_data[i][j][k] = std::exp(values_to_softmax[i]);
                sum_exp += softmaxed_data[i][j][k];
            }

            // Normalize using the sum of exponentials
            for (size_t i = 0; i < num_i; ++i) {
                softmaxed_data[i][j][k] /= sum_exp;
            }
        }
    }

    return softmaxed_data;
}




std::vector<std::vector<std::size_t>> applyArgmaxToAxis0(std::vector<std::vector<std::vector<float>>>& data) {
	// Ensure the input data is not empty
	if (data.empty()) {
		return {};
	}
	
	// Get the dimensions of the input data
	const size_t num_i = data.size();
	const size_t num_j = data[0].size();
	const size_t num_k = data[0][0].size();
	
	// Initialize the output vector with zeros
	std::vector<std::vector<std::size_t>>
	argmaxed_data(std::vector<std::vector<std::size_t>>(num_j, std::vector<std::size_t>(num_k, 0)));
	
	// Apply softmax to the values along axis 0
	for (size_t j = 0; j < num_j; ++j) {
		for (size_t k = 0; k < num_k; ++k) {
			// Collect the values to apply argmax to
			std::vector<float> values_to_argmax(num_i);
			for (size_t i = 0; i < num_i; ++i) {
				values_to_argmax[i] = data[i][j][k];
			}
			
			argmaxed_data[j][k] = argmax(values_to_argmax);
			
		}
	}
	
	return argmaxed_data;
}

std::vector<std::vector<std::size_t>> applyArgmaxToAxis1(std::vector<std::vector<std::vector<float>>>& data) {
	// Ensure the input data is not empty
	if (data.empty()) {
		return {};
	}
	
	// Get the dimensions of the input data
	const size_t num_i = data.size();
	const size_t num_j = data[0].size();
	const size_t num_k = data[0][0].size();
	
	// Initialize the output vector with zeros
	std::vector<std::vector<std::size_t>> argmaxed_data(std::vector<std::vector<std::size_t>>(num_k, std::vector<std::size_t>(num_j, 0)));
	
	// Apply argmax to the values along axis 1
	for (size_t k = 0; k < num_k; ++k) {
		for (size_t j = 0; j < num_j; ++j) {
			// Collect the values to apply argmax to
			std::vector<float> values_to_argmax(num_j);
			for (size_t i = 0; i < num_i; ++i) {
				values_to_argmax[j] = data[i][j][k];
			}
			
			argmaxed_data[k][j] = argmax(values_to_argmax);
		}
	}
	
	return argmaxed_data;
}

std::vector<std::vector<std::size_t>> applyArgmaxToAxis2(std::vector<std::vector<std::vector<float>>>& data) {
	// Ensure the input data is not empty
	if (data.empty()) {
		return {};
	}
	
	// Get the dimensions of the input data
	const size_t num_i = data.size();
	const size_t num_j = data[0].size();
	const size_t num_k = data[0][0].size();
	
	// Initialize the output vector with zeros
	std::vector<std::vector<std::size_t>> argmaxed_data(std::vector<std::vector<std::size_t>>(num_i, std::vector<std::size_t>(num_j, 0)));
	
	// Apply argmax to the values along axis 2
	for (size_t i = 0; i < num_i; ++i) {
		for (size_t j = 0; j < num_j; ++j) {
			// Collect the values to apply argmax to
			std::vector<float> values_to_argmax(num_k);
			for (size_t k = 0; k < num_k; ++k) {
				values_to_argmax[k] = data[i][j][k];
			}
			
			argmaxed_data[i][j] = argmax(values_to_argmax);
		}
	}
	
	return argmaxed_data;
}

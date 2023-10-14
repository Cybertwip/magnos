#include "components/MovingAverageFilter.hpp"

MovingAverageFilter::MovingAverageFilter(size_t size) : windowSize(size) {}

float MovingAverageFilter::filter(float inputValue) {
    buffer.push_back(inputValue);
    if (buffer.size() > windowSize) {
        buffer.erase(buffer.begin());
    }
    
    float sum = 0.0f;
    for (const float& value : buffer) {
        sum += value;
    }
    return sum / buffer.size();
}

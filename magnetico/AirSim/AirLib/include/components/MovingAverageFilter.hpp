#pragma once

#include <vector>
#include <cstddef>

class MovingAverageFilter {
private:
    std::vector<float> buffer;
    size_t windowSize;
    
public:
    MovingAverageFilter(size_t size);
    
    float filter(float inputValue);
};

#pragma once

#include <vector>

class MovingAverageFilter {
private:
    std::vector<float> buffer;
    size_t windowSize;
    
public:
    MovingAverageFilter(size_t size);
    
    float filter(float inputValue);
};

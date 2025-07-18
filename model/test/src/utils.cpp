#include "utils.h"
#include <vector>
#include <algorithm>

// Function to normalize input data
std::vector<float> normalizeData(const std::vector<float>& data, float min, float max) {
    std::vector<float> normalized(data.size());
    float dataMin = *std::min_element(data.begin(), data.end());
    float dataMax = *std::max_element(data.begin(), data.end());

    for (size_t i = 0; i < data.size(); ++i) {
        normalized[i] = (data[i] - dataMin) / (dataMax - dataMin) * (max - min) + min;
    }
    return normalized;
}

// Function to preprocess input data
std::vector<float> preprocessInput(const std::vector<float>& rawData) {
    // Example preprocessing steps
    std::vector<float> normalizedData = normalizeData(rawData, 0.0f, 1.0f);
    // Additional preprocessing can be added here
    return normalizedData;
}
#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>

// Utility functions for data preprocessing and normalization
std::vector<float> normalizeData(const std::vector<float>& data);
void preprocessInput(const std::vector<float>& rawInput, std::vector<float>& processedInput);

// Constants used throughout the project
const float NORMALIZATION_FACTOR = 255.0f;

#endif // UTILS_H
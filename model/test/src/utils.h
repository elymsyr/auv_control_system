#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>

// Utility functions for data preprocessing and normalization
std::vector<float> normalizeData(const std::vector<float>& data, float min, float max);
void preprocessInputData(const std::string& filePath, std::vector<float>& inputData);

// Constants used throughout the project
const float DATA_MIN = 0.0f;
const float DATA_MAX = 1.0f;

#endif // UTILS_H
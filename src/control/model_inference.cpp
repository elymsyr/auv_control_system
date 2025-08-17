#include "control/model_inference.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ModelInference::ModelInference() : scalers_loaded(false) {}

ModelInference::~ModelInference() {
    // The cv::dnn::Net object manages its own resources.
}

bool ModelInference::loadModel(const std::string& modelPath) {
    try {
        net = cv::dnn::readNet(modelPath);
        if (net.empty()) {
            throw std::runtime_error("Failed to load the model or the model file is invalid: " + modelPath);
        }

        // Check for CUDA backend availability and set it as the preferred target.
        // This is equivalent to model.to(torch::kCUDA).
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            std::cout << "CUDA is available. Setting backend to CUDA." << std::endl;
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        } else {
            std::cout << "Warning: CUDA not available. Using CPU backend." << std::endl;
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
        
        std::cout << "Model loaded successfully from: " << modelPath << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        return false;
    }
    return true;
}

// The scaler loading logic remains the same as it uses standard C++ and nlohmann/json
bool ModelInference::loadScalers(const std::string& scalerPath) {
    try {
        std::ifstream f(scalerPath);
        if (!f.is_open()) {
            throw std::runtime_error("Cannot open scaler file: " + scalerPath);
        }
        
        json scaler_data = json::parse(f);
        
        x_mean = scaler_data["x_mean"].get<std::vector<float>>();
        x_std = scaler_data["x_std"].get<std::vector<float>>();
        y_mean = scaler_data["y_mean"].get<std::vector<float>>();
        y_std = scaler_data["y_std"].get<std::vector<float>>();

        // It's good practice to validate sizes.
        if (x_mean.size() != 501 || x_std.size() != 501 || y_mean.size() != 8 || y_std.size() != 8) {
             throw std::runtime_error("Scaler dimensions do not match expected model I/O.");
        }
        
        scalers_loaded = true;
        std::cout << "Scalers loaded successfully from: " << scalerPath << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error loading scalers: " << e.what() << std::endl;
        scalers_loaded = false;
    }
    return scalers_loaded;
}

// Normalization/Denormalization logic is unchanged
std::vector<float> ModelInference::normalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std) {
    std::vector<float> normalized_data;
    normalized_data.reserve(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        normalized_data.push_back((data[i] - mean[i]) / std[i]);
    }
    return normalized_data;
}

std::vector<float> ModelInference::denormalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std) {
    std::vector<float> denormalized_data;
    denormalized_data.reserve(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        denormalized_data.push_back((data[i] * std[i]) + mean[i]);
    }
    return denormalized_data;
}


std::vector<float> ModelInference::runInference(const std::vector<float>& rawInputData) {
    if (net.empty()) {
        throw std::runtime_error("Model not loaded!");
    }
    if (!scalers_loaded) {
        throw std::runtime_error("Scalers not loaded!");
    }
     if (rawInputData.size() != 501) {
        throw std::invalid_argument("Invalid input size. Expected 501 elements, got " + std::to_string(rawInputData.size()));
    }
    
    try {
        // 1. Normalize the input data
        std::vector<float> normalized_input = normalize(rawInputData, x_mean, x_std);

        // 2. Create an input "blob" for the network from the vector.
        // The dimensions are [batch_size, channels, height, width].
        // For a flat vector like this, it's typically [1, 1, 1, num_features].
        // Or more simply, a 2D matrix of [1, num_features].
        cv::Mat inputBlob(1, normalized_input.size(), CV_32F, normalized_input.data());

        // 3. Set the input to the network
        net.setInput(inputBlob);
        
        // 4. Run forward pass and get the output
        cv::Mat output = net.forward();
        
        // 5. Convert the output cv::Mat back to a std::vector<float>
        std::vector<float> normalized_output;
        normalized_output.assign((float*)output.data, (float*)output.data + output.total());
        
        // 6. Denormalize the output
        std::vector<float> final_output = denormalize(normalized_output, y_mean, y_std);

        return final_output;
        
    } catch (const std::exception& e) {
        std::cerr << "Inference error: " << e.what() << std::endl;
        throw;
    }
}
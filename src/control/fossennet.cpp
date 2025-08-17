#include "control/fossennet.h"
#include "communication/topics.hpp" // Using the C++ structs directly
#include <vector>
#include <iostream>

// The constructor now takes paths directly, removing the need for ROS to find them.
NonlinearMPC::NonlinearMPC(const std::string& modelPath, const std::string& scalerPath)
    : model(), modelPath_(modelPath), scalerPath_(scalerPath) {}

void NonlinearMPC::initialization() {
    try {
        std::cout << "Loading model and scalers..." << std::endl;

        // Pass the full paths directly to the model loader.
        if (!model.loadScalers(scalerPath_)) {
            throw std::runtime_error("Failed to load scalers from: " + scalerPath_);
        }
        if (!model.loadModel(modelPath_)) {
            throw std::runtime_error("Failed to load model from: " + modelPath_);
        }
    } catch (const std::exception& e) {
        std::cerr << "Initialization error: " << e.what() << std::endl;
        throw; // Re-throw the exception to be handled by the caller
    }
}

// The function now accepts the plain C++ structs from 'topics.h'.
std::array<double, 8> NonlinearMPC::solve(const EnvironmentTopic& local_env, const MissionTopic& local_mission) {
    // --- The logic to flatten the data into a single vector remains the same ---
    std::vector<float> rawInputData;
    rawInputData.reserve(9 + HORIZON * 12); // Reserve space for efficiency

    // Current state (orientation and velocities)
    for (size_t i = 3; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.eta[i]));
    }
    for (size_t i = 0; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.nu[i]));
    }

    // Desired trajectory over the horizon
    auto mission_array = local_mission.get_array();
    for (size_t i = 1; i <= HORIZON; ++i) {
        // Adjust index for 0-based array access if HORIZON is the size
        const auto& point = mission_array[i-1];

        // Calculate relative position for the trajectory points
        rawInputData.push_back(static_cast<float>(point[0] - local_env.eta[0]));
        rawInputData.push_back(static_cast<float>(point[1] - local_env.eta[1]));
        rawInputData.push_back(static_cast<float>(point[2] - local_env.eta[2]));

        // Add the rest of the desired state (orientation and velocities)
        for (size_t j = 3; j < 6; ++j) {
            rawInputData.push_back(static_cast<float>(point[j]));
        }
        for (size_t j = 6; j < 12; ++j) {
            rawInputData.push_back(static_cast<float>(point[j]));
        }
    }

    // --- Inference and result handling ---
    std::vector<float> control_output_float;
    try {
        control_output_float = model.runInference(rawInputData);
    } catch (const std::exception& e) {
        std::cerr << "An error occurred during inference in solve(): " << e.what() << std::endl;
        // Return a zeroed array on error
        return {0.0};
    }

    std::array<double, 8> control_input = {0.0};
    if (control_output_float.size() == control_input.size()) {
        for (size_t i = 0; i < control_input.size(); ++i) {
            control_input[i] = static_cast<double>(control_output_float[i]);
        }
    } else {
        std::cerr << "Warning: Model output size (" << control_output_float.size()
                  << ") does not match expected control size (" << control_input.size() << ")." << std::endl;
    }

    return control_input;
}
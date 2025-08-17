#include "control/fossennet.h"
#include <torch/torch.h>
#include "control/model_inference.h"
#include <vector>
#include <optional>
#include <fstream>
#include "communication/topics.hpp"
#include <array>

NonlinearMPC::NonlinearMPC(std::string modelPath, std::string scalerPath) : modelPath(modelPath), scalerPath(scalerPath) {}

void NonlinearMPC::initialization() {
    if (torch::cuda::is_available()) {
        torch::globalContext().setBenchmarkCuDNN(true);
    }

    try {
        if (!model.loadScalers(scalerPath)) {
            throw std::runtime_error("Failed to load scalers.");
        }
        if (!model.loadModel(modelPath)) {
            throw std::runtime_error("Failed to load model.");
        }
    } catch (const std::exception& e) {
        throw;
    }
}

std::array<double, 8> NonlinearMPC::solve(const EnvironmentTopic& local_env, const MissionTopic& local_mission)
{
    std::vector<float> rawInputData;
    // Pre-allocating one element too many, but it's okay.
    // Correct size is 3 + 6 + (HORIZON * 12)
    rawInputData.reserve(12 + HORIZON * 12);

    // Add current state (env) to the input vector
    for (size_t i = 3; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.eta[i]));
    }
    for (size_t i = 0; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.nu[i]));
    }

    // ====================================================================
    // MODIFIED PART
    // ====================================================================
    // Get the trajectory in the format we need
    const auto trajectory_points = local_mission.get_trajectory_points();

    // CRITICAL FIX: Loop from 0 to HORIZON-1.
    // The original loop (i=1 to i<=HORIZON) was incorrect and would cause a buffer overflow.
    for (size_t i = 0; i < HORIZON; ++i) {
        const auto& point = trajectory_points[i]; // Access the correctly formatted point

        rawInputData.push_back(static_cast<float>(point.eta_desired[0] - local_env.eta[0]));
        rawInputData.push_back(static_cast<float>(point.eta_desired[1] - local_env.eta[1]));
        rawInputData.push_back(static_cast<float>(point.eta_desired[2] - local_env.eta[2]));
        
        for (size_t j = 3; j < 6; ++j) {
            rawInputData.push_back(static_cast<float>(point.eta_desired[j]));
        }
        for (size_t j = 0; j < 6; ++j) {
            rawInputData.push_back(static_cast<float>(point.nu_desired[j]));
        }
    }
    // ====================================================================

    std::vector<float> control_output_float;
    try {
        control_output_float = model.runInference(rawInputData);
    } catch (const std::exception& e) {
        std::cerr << "An error occurred during inference in solve(): " << e.what() << std::endl;
        return {0.0}; // Return a default/safe control input
    }

    std::array<double, 8> control_input = {0.0};
    if (control_output_float.size() == control_input.size()) {
        for (size_t i = 0; i < control_input.size(); ++i) {
            control_input[i] = static_cast<double>(control_output_float[i]);
        }
    }

    return control_input;
}
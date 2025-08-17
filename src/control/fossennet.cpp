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
    rawInputData.reserve(12 + HORIZON * 12);

    for (size_t i = 3; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.eta[i]));
    }
    for (size_t i = 0; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.nu[i]));
    }

    for (size_t i = 1; i <= HORIZON; ++i) {
        const auto& point = local_mission.trajectory[i];

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

    std::vector<float> control_output_float;
    try {
        control_output_float = model.runInference(rawInputData);
    } catch (const std::exception& e) {
        std::cerr << "An error occurred during inference in solve(): " << e.what() << std::endl;
        std::array<double, 8> error_control = {0.0};
        return error_control;
    }

    std::array<double, 8> control_input = {0.0};
    if (control_output_float.size() == control_input.size()) {
        for (size_t i = 0; i < control_input.size(); ++i) {
            control_input[i] = static_cast<double>(control_output_float[i]);
        }
    }

    return control_input;
}
#ifndef FOSSENNET_H // Renamed header guard for clarity
#define FOSSENNET_H

#include <string>
#include <vector>
#include <array>
#include "control/model_inference.h" // The new model inference class
#include "communication/topics.hpp"          // The plain C++ data structures

class NonlinearMPC {
public:
    /**
     * @brief Constructor for the Nonlinear MPC controller.
     * @param modelPath Full or relative path to the ONNX model file.
     * @param scalerPath Full or relative path to the JSON scaler file.
     */
    NonlinearMPC(const std::string& modelPath, const std::string& scalerPath);

    /**
     * @brief Initializes the controller by loading the model and scalers.
     * Throws a std::runtime_error on failure.
     */
    void initialization();

    /**
     * @brief Solves the control problem for a given state and mission.
     * @param local_env The current state of the system (from EnvironmentTopic).
     * @param local_mission The desired trajectory (from MissionTopic).
     * @return An array of 8 double values representing the calculated control inputs.
     */
    std::array<double, 8> solve(const EnvironmentTopic& local_env, const MissionTopic& local_mission);

private:
    ModelInference model_;      // The inference engine instance.
    std::string modelPath_;     // Path to the ONNX model.
    std::string scalerPath_;    // Path to the scaler data.
};

#endif // FOSSENNET_H
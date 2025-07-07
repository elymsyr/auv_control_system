#ifndef MOTION_H
#define MOTION_H

#include "system/subsystem.h"
#include "communication/topics.hpp"
#include "communication/communication_methods.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>
#include <mutex>
// #include <opencv4/opencv2/dnn.hpp>
// #include <opencv4/opencv2/core.hpp>
#include <vector>
#include <stdexcept>
#include <casadi/casadi.hpp>
#include "control/nlmpc.h"
#include "control/vehicle_model.h"

// class Model {
//     cv::dnn::Net net;

// public:
//     Model(const std::string& model_path = "model.onnx") {
//         net = cv::dnn::readNetFromONNX(model_path);
//         if (net.empty()) {
//             throw std::runtime_error("Failed to load ONNX model: " + model_path);
//         }
//         // Optional: Set backend (e.g., CPU/OpenVINO)
//         net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
//         net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
//     }

//     std::vector<float> predict(const std::vector<float>& current_state, const std::vector<float>& desired_state) {
//         if (current_state.size() != 12 || desired_state.size() != 12) {
//             throw std::invalid_argument("State vectors must have size 12");
//         }

//         // Combine inputs into a single vector
//         std::vector<float> input(24);
//         std::copy(current_state.begin(), current_state.end(), input.begin());
//         std::copy(desired_state.begin(), desired_state.end(), input.begin() + 12);

//         // Create a 4D blob (batch_size=1, channels=1, height=1, width=24)
//         cv::Mat blob = cv::Mat(input).reshape(1, {1, 1, 1, 24});

//         // Set input and run inference
//         net.setInput(blob);
//         cv::Mat output = net.forward();

//         // Convert output to vector
//         return std::vector<float>(output.ptr<float>(), output.ptr<float>() + output.total());
//     }
// };

class MotionSystem : public Subsystem {
    Publisher<MotionTopic> motion_pub_;
    Subscriber<MissionTopic> mission_sub_;
    Subscriber<EnvironmentTopic> env_sub_;
    
public:
    MotionTopic motion_state;
    MissionTopic mission_state;
    EnvironmentTopic env_state;
    NonlinearMPC mpc;

    MotionSystem(std::string name = "Motion", int runtime = 200, unsigned int system_code = 2);
    void init_() override;
    void halt() override;

protected:
    void function() override;
    void publish() override;
    std::mutex mission_mtx, env_mtx;

private:
    casadi::DM x0;
    casadi::DM x_ref;

};

#endif // MOTION_H
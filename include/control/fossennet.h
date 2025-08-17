#ifndef NLMPC_H
#define NLMPC_H

#include <vector>
#include <optional>
#include <string>
#include "mapping/config.h"
#include "control/model_inference.h"
#include <array>
#include "communication/topics.hpp"

class NonlinearMPC {
public:
    NonlinearMPC(std::string modelPath = "/models/fossen_net_1/fossen_net_scripted.pt", std::string scalerPath = "/models/fossen_net_1/scalers.json");

    std::array<double, 8> solve(const EnvironmentTopic& local_env, const MissionTopic& local_mission);

    void initialization();
private:
    ModelInference model;
    std::string modelPath;
    std::string scalerPath;
};

#endif // NLMPC_H
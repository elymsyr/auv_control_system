// mission.h
#pragma once
#include <casadi/casadi.hpp>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <any>
#include "environment_map.h"

class MissionSystem;

enum MissionReq {
    SONAR,
    CAM,
    LIGHT,
    GPS
};

class Mission {
public:
    virtual ~Mission() = default;
    
    std::vector<MissionReq> mission_req_;

    // Main mission interface
    virtual void initialize(MissionSystem& system) = 0;
    virtual void terminate() = 0;
    virtual void report() = 0;

    // Flexible trajectory generation with custom arguments
    virtual casadi::DM generate_reference_trajectory(
        const std::array<double, 12>& current_state,
        std::any reference) = 0;  // std::any allows different reference types
    
    // Mission metadata
    std::string name_;
    int id_;
    
    // Map management
    void create_map(int width, int height, int N) {
        map_ = std::make_unique<EnvironmentMap>(width, height, N);
    }
    
    EnvironmentMap* map() { return map_.get(); }

protected:
    std::unique_ptr<EnvironmentMap> map_;
    int state_ = 0;
};
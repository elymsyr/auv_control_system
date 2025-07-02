#ifndef MISSION_H
#define MISSION_H

#include <casadi/casadi.hpp>
#include <vector>
#include <string>
#include <array>
#include <any>
#include <functional>
#include "environment_map.h"
#include <mutex>

class Mission {
public:
    virtual ~Mission() = default;
    
    std::vector<MissionReq> mission_req_;

    // Mission lifecycle methods
    virtual void initialize() {}
    virtual std::array<double, 12> step(const std::array<double, 12>& current_state, float2 reference) {}
    virtual void terminate() {}
    virtual void report() {}

    // Mission metadata
    std::string name_;
    int id_;
    
    // Map management
    void create_map(int width = 129, int height = 129, int N = 40) {
        map_ = std::make_unique<EnvironmentMap>(width, height, N);
    }
    
    EnvironmentMap* map() { return map_.get(); }

protected:
    std::unique_ptr<EnvironmentMap> map_;
};

#endif // MISSION_H
#ifndef MISSION_SYSTEM_H
#define MISSION_SYSTEM_H

#include <functional>
#include <unordered_map>
#include "vehicle_model.h"
#include "environment_state.h"

class MissionSystem {
public:
    enum class MissionMode {
        WAIT,
        TEST_IMPLEMENTATION
    };

    MissionSystem();
    void set_mission(MissionMode mode);
    MissionMode check_mission() const;

private:
    MissionMode mission_mode = MissionMode::WAIT;
    int mission_state = 0;

    VehicleModel vehicle_model;

    void mission_step(EnvironmentState state);
    void mission_report();
    void mission_test();
    void mission_control();
    
    void test_implementation(EnvironmentState state);
};

#endif // MISSION_SYSTEM_H
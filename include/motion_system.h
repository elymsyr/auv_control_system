#ifndef MOTION_SYSTEM_H
#define MOTION_SYSTEM_H

#include "subsystem.h"
#include "environment_state.h"
#include "shared_data.h"
#include "mission_system.h"

using Mission = MissionTemplate<VehicleModel, EnvironmentState>;

class MotionSystem : public Subsystem
{
public:
    MotionSystem(std::string name, int runtime, SystemData &system, int order, EnvironmentState &envState, SharedSignalData &signalData);
    SharedSignalData &signalData;
    bool midInit() override;
    void liveLoop() override;

private:
    std::unordered_map<int, Mission> missions;
    EnvironmentState &envState;
    void motionControlLoop();
};

#endif // MOTION_SYSTEM_H
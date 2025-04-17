#ifndef MOTION_SYSTEM_H
#define MOTION_SYSTEM_H

#include "subsystem.h"
#include "environment_state.h"
#include "shared_data.h"

class MotionSystem : public Subsystem {
public:
    MotionSystem(std::string name, int runtime, SystemData& system, int order, EnvironmentState& envState, SharedSignalData& signalData);
    SharedSignalData& signalData;
    bool midInit() override;
    // void midSuspend() override;
    // void midHalt() override;
    // void midResume() override;
    void liveLoop() override;
    
    private:
    EnvironmentState& envState;
    void motionControlLoop();
    void calculateMotionCommands();
};

#endif // MOTION_SYSTEM_H
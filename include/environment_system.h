#ifndef ENVIRONMENT_SYSTEM_H
#define ENVIRONMENT_SYSTEM_H

#include "subsystem.h"
#include "shared_data.h"
#include "environment_state.h"

class EnvironmentSystem : public Subsystem {
public:
    EnvironmentSystem(std::string name, int runtime, SystemData& system, int order, SharedSensorData& sensorData, EnvironmentState& envState);
    bool midInit() override;
    // void midSuspend() override;
    // void midHalt() override;
    // void midResume() override;
    void liveLoop() override;
    
private:
    SharedSensorData& sharedSensorData;
    EnvironmentState& envState;
    std::atomic<bool> processing{false};
    
    void updateStateEstimation();
    void generateControlSignals();
};

#endif // ENVIRONMENT_SYSTEM_H
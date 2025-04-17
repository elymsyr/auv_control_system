#include "environment_system.h"
#include "shared_data.h"
#include "environment_state.h"
#include <thread>
#include <chrono>

EnvironmentSystem::EnvironmentSystem(std::string name, int runtime, SystemData& system, int order, SharedSensorData& sensorData, EnvironmentState& envState)
    : Subsystem(name, runtime, system, order), sharedSensorData(sensorData), envState(envState) {}

bool EnvironmentSystem::midInit() {
    return true;
}

void EnvironmentSystem::liveLoop() {
    while(initialized.load()){
        while(live.load()) {
            reset_timer();
            updateStateEstimation();
            generateControlSignals();
            updateHeartbeat();
            sleep_til();
        }
        updateHeartbeat();
    }
}

void EnvironmentSystem::updateStateEstimation() {
    if (auto opt = sharedSensorData.read(); opt) {
        SensorData data = *opt;
        envState.updateState(data);
    } else {
        return;
    }
}

void EnvironmentSystem::generateControlSignals() {
    // Control signal generation implementation
}
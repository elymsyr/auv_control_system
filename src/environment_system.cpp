#include "environment_system.h"
#include "shared_data.h"
#include "environment_state.h"
#include <thread>
#include <chrono>

EnvironmentSystem::EnvironmentSystem(std::string name, int runtime, SystemData& system, int order, EnvironmentState& envState)
    : Subsystem(name, runtime, system, order), envState(envState) {}

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

}

void EnvironmentSystem::generateControlSignals() {

}
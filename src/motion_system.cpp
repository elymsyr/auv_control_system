#include "motion_system.h"
#include "shared_data.h"
#include <thread>
#include <chrono>
#include <iostream>

MotionSystem::MotionSystem(std::string name, int runtime, SystemData& system, int order, EnvironmentState& envState, SharedSignalData& signalData)
    : Subsystem(name, runtime, system, order), envState(envState), signalData(signalData) {}

bool MotionSystem::midInit() {
    return true;
}

void MotionSystem::liveLoop() {
    while(initialized.load()) {
        while(live.load()) {
            reset_timer();
            motionControlLoop();
            updateHeartbeat();
            sleep_til();
        }
        updateHeartbeat();
    }
}

void MotionSystem::motionControlLoop() {
    // auto current = envState.getCurrentState();
    // auto desired = envState.getDesiredState();
    // // std::cout << "Current Position: " 
    // //           << current.temperature << ", " 
    // //           << current.depth << std::endl;
    // calculateMotionCommands();
}

void MotionSystem::calculateMotionCommands() {
    // Implement motion control algorithms
}
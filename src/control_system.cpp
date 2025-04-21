#include "control_system.h"

ControlSystem::ControlSystem(std::string name = "Control", int runtime = 100, unsigned int system_code = 3) 
    : Subsystem(name, runtime, system_code) {}

void ControlSystem::init() {
    motion_sub_.connect("tcp://localhost:5563");
    signal_sub_.connect("tcp://localhost:5562");
    std::cout << name << " initialized\n";
}

void ControlSystem::refresh_received() {
    std::lock_guard lk(mtx);
    try {
        motion_state.set(motion_sub_.receive());
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to receive motion state\n";
    }
    try {
        signal_state.set(signal_sub_.receive());
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to receive signal state\n";
    }
}

void ControlSystem::function() {
    // notify if any error occured
}

void ControlSystem::publish() {}
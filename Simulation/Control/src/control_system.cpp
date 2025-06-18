#include "control_system.h"

ControlSystem::ControlSystem(std::string name, int runtime, unsigned int system_code) 
    : motion_sub_(motion_state, motion_mtx), signal_sub_(signal_state, signal_mtx), Subsystem(name, runtime, system_code) {}

void ControlSystem::init_() {
    try
    {
        for (int i = 0; i < 20 && !motion_sub_.is_running() ; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            motion_sub_.connect("tcp://localhost:5563");
        }
        for (int i = 0; i < 20 && !signal_sub_.is_running() ; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            signal_sub_.connect("tcp://localhost:5562");
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    motion_state.set();
    signal_state.set();
}

void ControlSystem::function() {
}

void ControlSystem::publish() {}

void ControlSystem::halt() {
    motion_sub_.close();
    signal_sub_.close();
    initialized = false;
}

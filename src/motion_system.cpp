#include "motion_system.h"

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : Subsystem(name, runtime, system_code) 
{
    motion_pub_.bind("tcp://*:5563");
}

void MotionSystem::init() {
    mission_sub_.connect("tcp://localhost:5561");
    env_sub_.connect("tcp://localhost:5560");
    std::cout << name << " initialized\n";
}

void MotionSystem::refresh_received() {
    std::lock_guard lk(mtx);
    try {
        mission_state.set(mission_sub_.receive());
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to receive mission state\n";
    }
    try {
        env_state.set(env_sub_.receive());
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to receive environment state\n";
    }
}

void MotionSystem::function() {
    // notify if any error occured
    motion_state.set();
}

void MotionSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    motion_pub_.publish(motion_state);
}

#include "motion_system.h"

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : mission_sub_(mission_state, mission_mtx), env_sub_(env_state, env_mtx), Subsystem(name, runtime, system_code) 
{
    motion_pub_.bind("tcp://*:5563");
}

void MotionSystem::init() {
    mission_sub_.connect("tcp://localhost:5561");
    env_sub_.connect("tcp://localhost:5560");
    std::cout << name << " initialized\n";
}

void MotionSystem::function() {
    // notify if any error occured
    std::array<double, 6> propeller = {static_cast<double>(rand() % 10), static_cast<double>(rand() % 10), static_cast<double>(rand() % 10), static_cast<double>(rand() % 10), static_cast<double>(rand() % 8), static_cast<double>(rand() % 8)};
    MotionTopic test;
    test.set(propeller);
    motion_state.set();
}

void MotionSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    motion_pub_.publish(motion_state);
}

#include "motion_system.h"

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : mission_sub_(mission_state, mission_mtx), env_sub_(env_state, env_mtx), Subsystem(name, runtime, system_code) 
{
    for (int i = 0; i < 20 && !motion_pub_.is_bound() ; i++) {
        motion_pub_.bind("tcp://localhost:5563");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MotionSystem::init_() {
    for (int i = 0; i < 20 && !mission_sub_.is_running() ; i++) {
        mission_sub_.connect("tcp://localhost:5561");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (int i = 0; i < 20 && !env_sub_.is_running() ; i++) {
        env_sub_.connect("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MotionSystem::function() {
    // notify if any error occured
    std::array<double, 6> propeller = {static_cast<double>(rand() % 10), static_cast<double>(rand() % 10), static_cast<double>(rand() % 10), static_cast<double>(rand() % 10), static_cast<double>(rand() % 8), static_cast<double>(rand() % 8)};
    MotionTopic test;
    test.set(propeller);
    std::lock_guard lk(mtx);
    motion_state.set(test);
}

void MotionSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    motion_pub_.publish(motion_state);
}

void MotionSystem::halt() {
    motion_pub_.close();
    mission_sub_.close();
    env_sub_.close();
    initialized = false;
}

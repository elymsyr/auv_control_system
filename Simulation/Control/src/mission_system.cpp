#include "mission_system.h"
#include "environment.h"

MissionSystem::MissionSystem(std::string name, int runtime, unsigned int system_code)
    : env_sub_(env_state, env_mtx), Subsystem(name, runtime, system_code) 
{
    for (int i = 0; i < 20 && !mission_pub_.is_bound() ; i++) {
        mission_pub_.bind("tcp://localhost:5561");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (int i = 0; i < 20 && !signal_pub_.is_bound() ; i++) {
        signal_pub_.bind("tcp://localhost:5562");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    map_ = new EnvironmentMap(129, 129);
    env_state.set();
}

void MissionSystem::init_() {
    for (int i = 0; i < 20 && !env_sub_.is_running() ; i++) {
        env_sub_.connect("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MissionSystem::function() {
    std::lock_guard lk(mtx);
    signal_state.set();
    mission_state.set();
}

void MissionSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    mission_pub_.publish(mission_state);
    signal_pub_.publish(signal_state);
}

void MissionSystem::halt() {
    mission_pub_.close();
    signal_pub_.close();
    env_sub_.close();
    initialized = false;
}
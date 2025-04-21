#include "mission_system.h"

MissionSystem::MissionSystem(std::string name, int runtime, unsigned int system_code)
    : env_sub_(env_state, env_mtx), Subsystem(name, runtime, system_code) 
{
    mission_pub_.bind("tcp://*:5561");
    signal_pub_.bind("tcp://*:5562");
}

void MissionSystem::init() {
    env_sub_.connect("tcp://localhost:5560");
    std::cout << name << " initialized\n";
}

void MissionSystem::function() {
    // notify if any error occured
    signal_state.set();
    mission_state.set();
}

void MissionSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    mission_pub_.publish(mission_state);
    signal_pub_.publish(signal_state);
}

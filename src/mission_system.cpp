#include "mission_system.h">

MissionSystem::MissionSystem(std::string name = "Mission", int runtime = 100, unsigned int system_code = 1)
    : Subsystem(name, runtime, system_code) 
{
    mission_pub_.bind("tcp://*:5561");
    signal_pub_.bind("tcp://*:5562");
}

void MissionSystem::init() {
    env_sub_.connect("tcp://localhost:5560");
    std::cout << name << " initialized\n";
}

void MissionSystem::refresh_received() {
    std::lock_guard lk(mtx);
    try {
        env_state.set(env_sub_.receive());
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to set env state\n";
    }
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

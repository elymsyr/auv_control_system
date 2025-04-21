#include "environment_system.h"

EnvironmentSystem::EnvironmentSystem(std::string name, int runtime, unsigned int system_code)
    : Subsystem(name, runtime, system_code)
{
    env_pub_.bind("tcp://*:5560");
}

void EnvironmentSystem::init() {
    std::cout << name << " initialized\n";
}

void EnvironmentSystem::function() {
    // notify if any error occured
    env_state.set();
}

void EnvironmentSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    env_pub_.publish(env_state);
}

#include "environment_system.h"

EnvironmentSystem::EnvironmentSystem(std::string name, int runtime, unsigned int system_code)
    : Subsystem(name, runtime, system_code)
{
    for (int i = 0; i < 20 && !env_pub_.is_bound(); ++i) {
        env_pub_.bind("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void EnvironmentSystem::init_() {
}

void EnvironmentSystem::function() {
    std::lock_guard lk(mtx);
    env_state.set();
}

void EnvironmentSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    env_pub_.publish(env_state);
}

void EnvironmentSystem::halt() {
    try {
        env_pub_.close();
        initialized = false;
    } catch (const std::exception& e) {
        std::cerr << "Halt failed for " << name << ": " << e.what() << "\n";
    }
}
#include "system/environment_system.h"
#include "control/vehicle_model.h"
#include <casadi/casadi.hpp>
#include <algorithm>

using namespace casadi;

EnvironmentSystem::EnvironmentSystem(std::string name, int runtime, unsigned int system_code)
    : motion_sub_(motion_state, motion_mtx), Subsystem(name, runtime, system_code)
{
    env_state.set();
    motion_state.set();
    for (int i = 0; i < 20 && !env_pub_.is_bound(); ++i) {
        env_pub_.bind("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void EnvironmentSystem::init_() {
    for (int i = 0; i < 20 && !motion_sub_.is_running() ; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        motion_sub_.connect("tcp://localhost:5563");
    }
}

void EnvironmentSystem::function() {
    // Copy propeller values safely
    std::vector<double> propeller_copy;
    std::vector<double> x_next_copy;
    {
        std::lock_guard<std::mutex> motion_lock(motion_mtx);
        motion_state.get(propeller_copy, x_next_copy);
    }
    {
        std::lock_guard lk(mtx);
        for (int i = 0; i < 6; i++) {
            env_state.eta[i] = x_next_copy[i];
            env_state.nu[i] = x_next_copy[i+6];
        }
    }
}

void EnvironmentSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    env_pub_.publish(env_state);
}

void EnvironmentSystem::halt() {
    try {
        motion_sub_.close();
        env_pub_.close();
        initialized = false;
    } catch (const std::exception& e) {
        std::cerr << "Halt failed for " << name << ": " << e.what() << "\n";
    }
}
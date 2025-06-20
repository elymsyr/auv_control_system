#include "motion_system.h"
#include <casadi/casadi.hpp>

using namespace casadi;

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : vehicle_model("/home/eren/GitHub/ControlSystem/Simulation/Control/config.json"), mpc(vehicle_model), mission_sub_(mission_state, mission_mtx), env_sub_(env_state, env_mtx), Subsystem(name, runtime, system_code) 
{
    for (int i = 0; i < 20 && !motion_pub_.is_bound() ; i++) {
        motion_pub_.bind("tcp://localhost:5563");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mission_state.set();
    env_state.set();
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
    DM x0 = env_state.get_dm();
    DM x_ref = mission_state.get_dm();
    auto solution = NonlinearMPC::solve(x0, x_ref);
    DM propeller = solution.first;
    {
        std::lock_guard<std::mutex> lk(mtx);
        motion_state.set(propeller);
    }
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

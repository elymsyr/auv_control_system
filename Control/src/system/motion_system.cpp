#include "system/motion_system.h"
#include <casadi/casadi.hpp>

using namespace casadi;

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : Subsystem(name, runtime, system_code),
      mpc("/home/eren/GitHub/Simulation/config.json", 40, 0.1),
      mission_sub_(mission_state, mission_mtx),
      env_sub_(env_state, env_mtx),
      x0(DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})),
      x_ref(DM::zeros(12, 41))
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
    mpc.initialization();
}

void MotionSystem::function() {
    try {
        {
            std::lock_guard<std::mutex> lk(mtx);
            // std::array<double, 12> x_current = env_state.get_array();
            env_state.get_dm(x0);
            mission_state.get_dm(x_ref);
        }
        
        // Check for valid inputs
        if (x0.is_empty() || x_ref.is_empty()) {
            throw std::runtime_error("Empty state received");
        }
        auto solution = mpc.solve(x0, x_ref);
        DM propeller = solution.first;
        DM x_opt = solution.second;
        DM x_next = x_opt(Slice(), 1);
        
        {
            std::lock_guard<std::mutex> lk(mtx);
            motion_state.set(propeller, x_next);
        }
    } catch (const std::exception& e) {
        std::cerr << "MotionSystem error: " << e.what() << std::endl;
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

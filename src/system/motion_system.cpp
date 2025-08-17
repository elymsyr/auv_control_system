#include "system/motion_system.h"
#include "communication/topics.hpp"
#include "communication/communication_methods.h"

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : Subsystem(name, runtime, system_code),
      mpc(),
      mission_sub_(mission_state, mission_mtx),
      env_sub_(env_state, env_mtx)
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
            x0 = env_state;
            x_ref = mission_state;
        }
        
        std::array<double, 8> propeller = mpc.solve(x0, x_ref);
        std::array<double, 12> next_state = vehicle_model_.calculate_next_state(x0, propeller, 0.1);
        {
            std::lock_guard<std::mutex> lk(mtx);
            motion_state.set(propeller, next_state);
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

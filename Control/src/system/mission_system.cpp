#include "system/mission_system.h"
#include "mapping/environment.h"
#include <casadi/casadi.hpp>
#include "mission/mission_sonar_imp.h"

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
    env_state.set();
    mission_state.set();
    signal_state.set();
}

void MissionSystem::init_() {
    for (int i = 0; i < 20 && !env_sub_.is_running() ; i++) {
        env_sub_.connect("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MissionSystem::function() {
    std::array<double, 12> current_state;
    {
        std::lock_guard lk(env_mtx);
        current_state = env_state.get_array();
    }

    std::array<std::array<double, 12>, HORIZON> ref_path;
    if (mission_running_) {
        ref_path = active_mission_->step(current_state);
    } else {
        for (size_t i = 0; i < HORIZON; i++) {
            std::array<double, 12> ref_row;
            ref_row.fill(0.0);
            // Directly edit the desired point in ref_path
            ref_row[0] = current_state[0];
            ref_row[1] = current_state[1];
            ref_row[2] = -1;
            ref_path[i] = ref_row;
        }
    }

    {
        std::lock_guard lk(mtx);
        mission_state.set(ref_path);
    }
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

void MissionSystem::setMission(MissionIMP mission_imp) {
    if (mission_running_) {
        stopMission();
    }

    // Create the mission based on the mission_imp
    if (mission_imp == MissionIMP::SonarMisTest) {
        active_mission_ = std::make_unique<SonarMission>();
    } else {
        active_mission_ = nullptr;
        return;
    }

    if (active_mission_) {
        active_mission_->initialize();
    }
}

void MissionSystem::startMission() {
    if (active_mission_) {
        mission_running_ = true;
    }
}

void MissionSystem::stopMission() {
    if (active_mission_ && mission_running_) {
        active_mission_->terminate();
        mission_running_ = false;
    }
}

void MissionSystem::reportMission() {
    if (active_mission_) {
        active_mission_->report();
    }
};
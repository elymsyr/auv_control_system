#include "mission_system.h"
#include "environment.h"
#include <casadi/casadi.hpp>
#include <mission_imp.h>
#include <mission_sonar_imp.h>

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
    x_ref = casadi::DM::repmat(casadi::DM::vertcat({20, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, map_->N+1);
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
    double* degree;
    double* detections;
    std::array<double, 12> state;
    {
        std::lock_guard lk(testsonar_mtx);
        degree = testsonar_state.degree;
        detections = testsonar_state.detection;
    }
    {
        std::lock_guard lk(env_mtx);
        state = env_state.get_array();
    }
    map_->slide(state[0], state[1]);
    float3* obstacle_world = convert_obs_to_world(state, degree, detections);
    for (int i = 0 ; i < 10 ; i ++) {
        if (obstacle_world[i].x < 0.2f && obstacle_world[i].y < 0.2f) {
            continue;
        }
        map_->updateSinglePoint(obstacle_world[i].x, obstacle_world[i].y, 255.0f);
    }
    delete[] obstacle_world;
    
    Path path = map_->findPath(20.0f, 16.0f);

    for (int k = 0; k <= map_->N; k++) {
        float2 position = path.trajectory[k];
        float yaw = path.angle[k];
        
        // Use in controller
        x_ref(0, k) = position.x;
        x_ref(1, k) = position.y;
        x_ref(5, k) = yaw;
        if (step % 20 == 0) map_->updateSinglePoint(position.x, position.y, 160.0f);
    }
    {
        std::lock_guard lk(mtx);
        mission_state.set(x_ref);
    }
    map_->updateSinglePoint(state[0], state[1], 55.0f);
    if (step % 20 == 0) map_->save(std::to_string(step));
    std::cout << "Step: " << step << "\n";
    step++;
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
    testsonar_sub_.close();
    initialized = false;
}

void MissionSystem::setMission(MissionIMP mission_imp) {
    if (mission_running_) {
        stopMission();
    }

    // Create the mission based on the mission_imp
    if (mission_imp == MissionIMP::SonarMis) {
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
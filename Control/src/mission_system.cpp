#include "mission_system.h"
#include "environment.h"
#include <casadi/casadi.hpp>

MissionSystem::MissionSystem(std::string name, int runtime, unsigned int system_code)
    : testsonar_sub_(testsonar_state, testsonar_mtx), env_sub_(env_state, env_mtx), Subsystem(name, runtime, system_code) 
{
    for (int i = 0; i < 20 && !mission_pub_.is_bound() ; i++) {
        mission_pub_.bind("tcp://localhost:5561");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (int i = 0; i < 20 && !signal_pub_.is_bound() ; i++) {
        signal_pub_.bind("tcp://localhost:5562");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    map_ = new EnvironmentMap(129, 129, 40);
    x_ref = casadi::DM::repmat(casadi::DM::vertcat({20, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, map_->N+1);
    env_state.set();
    mission_state.set();
    signal_state.set();
    testsonar_state.set();
}

void MissionSystem::init_() {
    for (int i = 0; i < 20 && !env_sub_.is_running() ; i++) {
        env_sub_.connect("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (int i = 0; i < 20 && !testsonar_sub_.is_running() ; i++) {
        testsonar_sub_.connect("tcp://localhost:7778");
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

float3* MissionSystem::convert_obs_to_world(std::array<double, 12> state, double* degree, double* detections) {
    float3* obstacle_world = new float3[10];
    // Extract robot's position and orientation
    double robot_x = state[0];
    double robot_y = state[1];
    double robot_z = state[2];
    double yaw = state[5];
    
    const double deg2rad = M_PI / 180.0;
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    for (int i = 0; i < 10; i++) {
        if (detections[i] < 0.1f) {
            obstacle_world[i] = {0.0f, 0.0f, 0.0f};
            continue;
        }

        // Convert angle from degrees to radians
        double angle_rad = degree[i] * deg2rad;
        
        // Convert sonar reading to Cartesian in robot's local frame
        double local_x = detections[i] * std::cos(angle_rad);
        double local_y = detections[i] * std::sin(angle_rad);
        
        // Rotate and translate to world coordinates
        double world_x = local_x * cos_yaw - local_y * sin_yaw + robot_x;
        double world_y = local_x * sin_yaw + local_y * cos_yaw + robot_y;
        double world_z = robot_z;
        
        // Store as float3
        obstacle_world[i] = {static_cast<float>(world_x), static_cast<float>(world_y), static_cast<float>(world_z)};
    }
    return obstacle_world;
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
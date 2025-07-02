#ifndef SONAR_MISSION_H
#define SONAR_MISSION_H

#include "mission.h"
#include <cmath>
#include "topics.hpp"
#include "communication_system.h"
#include <zmq_addon.hpp>
#include <mutex>

class SonarMission : public Mission {
public:
    SonarMission() : testsonar_sub_(testsonar_state, testsonar_mtx) {
        for (int i = 0; i < 20 && !testsonar_sub_.is_running() ; i++) {
            testsonar_sub_.connect("tcp://localhost:7778");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        testsonar_state.set();
        name_ = "Sonar Obstacle Avoidance";
        id_ = 1;
        mission_req_ = {};
        create_map(129, 129, 40);
    }

    void initialize() override {

    }

    std::array<double, 12> step(const std::array<double, 12>& current_state) override {
    }

    void terminate() override {
    }

    void report() override {
    }

private:
    Subscriber<TestSonarTopic> testsonar_sub_;
    TestSonarTopic testsonar_state;

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

protected:
    std::mutex testsonar_mtx;
};

#endif // SONAR_MISSION_H
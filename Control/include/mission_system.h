#ifndef MISSION_H
#define MISSION_H

#include "subsystem.h"
#include "environment.h"
#include "topics.hpp"
#include "communication_system.h"
#include <iostream>
#include <casadi/casadi.hpp>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>
#include <mutex>

class MissionSystem : public Subsystem {
    Publisher<MissionTopic> mission_pub_;
    Publisher<SignalTopic> signal_pub_;
    Subscriber<EnvironmentTopic> env_sub_;
    Subscriber<TestSonarTopic> testsonar_sub_;
    
public:
    MissionTopic mission_state;
    SignalTopic signal_state;
    EnvironmentTopic env_state;
    TestSonarTopic testsonar_state;

    MissionSystem(std::string name = "Mission", int runtime = 100, unsigned int system_code = 1);
    void init_() override;
    void halt() override;

private:
    EnvironmentMap* map_ = nullptr;
    casadi::DM x_ref;
    int step = 0;
    float3* convert_obs_to_world(std::array<double, 12> state, double* degree, double* detections);

protected:
    void function() override;
    void updateMap();
    void updateMissionState();
    void updateSignalState();
    void publish() override;
    std::mutex env_mtx, testsonar_mtx;
};

#endif // MISSION_H
#ifndef SONAR_MISSION_H
#define SONAR_MISSION_H

#include "mission/mission.h"
#include <cmath>
#include "communication/topics.hpp"
#include "communication/communication_methods.h"
#include <zmq_addon.hpp>
#include <mutex>
#include "mapping/config.h"
#include <memory>
#include <functional>
#include <thread>

class SonarMission : public Mission {
public:
    SonarMission();
    void initialize() override;
    void terminate() override;
    void report() override;

private:
    Subscriber<TestSonarTopic> testsonar_sub_;
    TestSonarTopic testsonar_state;

    float3* convert_obs_to_world(std::array<double, 12> state, double* degree, double* detections);
    void state_initial();
    void updateMap(const std::array<double, 12>& current_state) override;
    void state_0(const std::array<double, 12>& current_state);
    void state_1(const std::array<double, 12>& current_state);
    void state_2(const std::array<double, 12>& current_state);
    void state_3(const std::array<double, 12>& current_state);
    void state_4(const std::array<double, 12>& current_state);
    void state_5(const std::array<double, 12>& current_state);
    void state_6(const std::array<double, 12>& current_state);
    void setState(int new_state);
    void set_state_list();

protected:
    std::mutex testsonar_mtx;
};

#endif // SONAR_MISSION_H
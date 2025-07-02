#ifndef MISSION_SYSTEM_H
#define MISSION_SYSTEM_H

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
#include <mission_imp.h>

class MissionSystem : public Subsystem {
public:
    Publisher<MissionTopic> mission_pub_;
    Publisher<SignalTopic> signal_pub_;
    Subscriber<EnvironmentTopic> env_sub_;
    
    MissionTopic mission_state;
    SignalTopic signal_state;
    EnvironmentTopic env_state;

    MissionSystem(std::string name = "Mission", int runtime = 100, unsigned int system_code = 1);
    void init_() override;
    void halt() override;

    void setMission(MissionIMP mission_imp)
    void startMission();
    void stopMission();
    void getMissionReport();

private:
    casadi::DM x_ref;
    int step = 0;
    std::unique_ptr<Mission> active_mission_;
    bool mission_running_ = false;

protected:
    void function() override;
    void publish() override;
    std::mutex env_mtx;
};

#endif // MISSION_SYSTEM_H
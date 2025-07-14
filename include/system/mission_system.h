#ifndef MISSION_SYSTEM_H
#define MISSION_SYSTEM_H

#include "system/subsystem.h"
#include "mapping/environment.h"
#include "communication/topics.hpp"
#include "communication/communication_methods.h"
#include <iostream>
#include <zmq_addon.hpp>
#include <any>
#include <mutex>
#include "mission/mission_imp.h"
#include "mission/mission.h"
#include "mapping/config.h"

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

    void setMission(MissionIMP mission_imp);
    void startMission();
    void stopMission();
    void reportMission();

    std::string getMission();

private:
    std::unique_ptr<Mission> active_mission_;
    bool mission_running_ = false;

protected:
    void function() override;
    void publish() override;
    std::mutex env_mtx;
};

#endif // MISSION_SYSTEM_H
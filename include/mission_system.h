#ifndef MISSION_H
#define MISSION_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include "EnvironmentMap.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>
#include <mutex>

class MissionSystem : public Subsystem {
    Publisher<MissionTopic> mission_pub_;
    Publisher<SignalTopic> signal_pub_;
    Subscriber<EnvironmentTopic> env_sub_;
    
public:
    MissionTopic mission_state;
    SignalTopic signal_state;
    EnvironmentTopic env_state;

    MissionSystem(std::string name = "Mission", int runtime = 100, unsigned int system_code = 1);
    void init_() override;
    void halt() override;

private:
    EnvironmentMap* map_ = nullptr;

protected:
    void function() override;
    void updateMap();
    void updateMissionState();
    void updateSignalState();
    void publish() override;
    std::mutex env_mtx;
};

#endif // MISSION_H
#ifndef MISSION_H
#define MISSION_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>

class MissionSystem : public Subsystem {
    Publisher<MissionTopic> mission_pub_;
    Publisher<SignalTopic> signal_pub_;
    Subscriber<EnvironmentTopic> env_sub_;
    
public:
    MissionTopic mission_state;
    SignalTopic signal_state;
    EnvironmentTopic env_state;

    MissionSystem(std::string name = "Mission", int runtime = 100, unsigned int system_code = 1);

    void init() override;

protected:
    void refresh_received() override;

    void function() override;

    void publish() override;
};

#endif // MISSION_H
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

    MissionSystem(std::string name = "Mission", int runtime = 100, unsigned int system_code = 1)
        : Subsystem(name, runtime, system_code) 
    {
        mission_pub_.bind("tcp://*:5561");
        signal_pub_.bind("tcp://*:5562");
    }

    void init() override {
        env_sub_.connect("tcp://localhost:5560");
        std::cout << name << " initialized\n";
    }

protected:
    void function() override {
        {
            std::lock_guard lk(mtx);
            env_state.set(env_sub_.receive());
            system_state.set();
            signal_state.set();
            mission_state.set();
        }
    }

    void postState() override {
        std::shared_lock lock(topic_read_mutex);
        state_pub_.publish(system_state);
        mission_pub_.publish(mission_state);
        signal_pub_.publish(signal_state);
    }
};

#endif // MISSION_H
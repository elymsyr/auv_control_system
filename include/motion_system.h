#ifndef MOTION_H
#define MOTION_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>

class MotionSystem : public Subsystem {
    Publisher<MotionTopic> motion_pub_;
    Subscriber<MissionTopic> mission_sub_;
    
public:
    MotionTopic motion_state;
    MissionTopic mission_state;

    MotionSystem(std::string name = "Motion", int runtime = 100, unsigned int system_code = 2) 
        : Subsystem(name, runtime, system_code) 
    {
        motion_pub_.bind("tcp://*:5563");
    }

    void init() override {
        mission_sub_.connect("tcp://localhost:5561");
        std::cout << name << " initialized\n";
    }

protected:
    void function() override {
        {
            std::lock_guard lk(mtx);
            mission_state.set(mission_sub_.receive());
            system_state.set();
            motion_state.set();
        }
    }

    void postState() override {
        std::shared_lock lock(topic_read_mutex);
        state_pub_.publish(system_state);
        motion_pub_.publish(motion_state);
    }
};

#endif // MOTION_H
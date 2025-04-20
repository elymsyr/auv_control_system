#ifndef CONTROL_H
#define CONTROL_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>

class ControlSystem : public Subsystem {
    Subscriber<MotionTopic> motion_sub_;
    Subscriber<SignalTopic> signal_sub_;
    
public:
    MotionTopic motion_state;
    SignalTopic signal_state;

    ControlSystem(std::string name = "Control", int runtime = 100, unsigned int system_code = 3) 
        : Subsystem(name, runtime, system_code) 
    {}

    void init() override {
        motion_sub_.connect("tcp://localhost:5563");
        signal_sub_.connect("tcp://localhost:5562");
        std::cout << name << " initialized\n";
    }

protected:
    void function() override {
        {
            std::lock_guard lk(mtx);
            motion_state.set(motion_sub_.receive());
            signal_state.set(signal_sub_.receive());
            system_state.set();
        }
    }

    void postState() override {
        std::shared_lock lock(topic_read_mutex);
        state_pub_.publish(system_state);
    }
};

#endif // CONTROL_H
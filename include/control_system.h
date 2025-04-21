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

    ControlSystem(std::string name = "Control", int runtime = 100, unsigned int system_code = 3);

    void init() override;

protected:
    void refresh_received() override;

    void function() override;

    void publish() override;
};

#endif // CONTROL_H
#ifndef CONTROL_H
#define CONTROL_H

#include "system/subsystem.h"
#include "communication/topics.hpp"
#include "communication/communication_methods.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>
#include <mutex>

class ControlSystem : public Subsystem {
    Subscriber<MotionTopic> motion_sub_;
    Subscriber<SignalTopic> signal_sub_;
    
public:
    MotionTopic motion_state;
    SignalTopic signal_state;

    ControlSystem(std::string name = "Control", int runtime = 100, unsigned int system_code = 3);
    void init_() override;
    void halt() override;

protected:
    void function() override;
    void publish() override;
    std::mutex motion_mtx, signal_mtx;
};

#endif // CONTROL_H
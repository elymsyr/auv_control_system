#ifndef MOTION_H
#define MOTION_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>
#include <mutex>

class MotionSystem : public Subsystem {
    Publisher<MotionTopic> motion_pub_;
    Subscriber<MissionTopic> mission_sub_;
    Subscriber<EnvironmentTopic> env_sub_;
    
public:
    MotionTopic motion_state;
    MissionTopic mission_state;
    EnvironmentTopic env_state;

    MotionSystem(std::string name = "Motion", int runtime = 200, unsigned int system_code = 2);
    void init() override;

protected:
    void function() override;
    void publish() override;
    std::mutex mission_mtx, env_mtx;
};

#endif // MOTION_H
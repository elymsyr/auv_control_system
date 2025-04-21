#ifndef MAIN_H
#define MAIN_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include "environment_system.h"
#include "mission_system.h"
#include "motion_system.h"
#include "control_system.h"
#include <iostream>
#include <memory>
#include <mutex>

enum class SystemID {
    ENVIRONMENT,
    MISSION,
    MOTION,
    CONTROL,
};

enum class Operation {
    INIT,
    START,
    STOP,
    HALT,
};

class MainSystem : public Subsystem {
    Subscriber<CommandTopic> command_sub_;
    Subscriber<StateTopic> system_sub_;
    
    std::thread proxy_thread;
    zmq::context_t proxy_ctx;

    std::unordered_map<SystemID, std::unique_ptr<Subsystem>> systems_;
    std::unordered_map<SystemID, int> system_configs_;

    MissionTopic mission_state;
    EnvironmentTopic env_state;
    CommandTopic command_received;

public:
    MainSystem(std::string name = "Main", int runtime = 200, unsigned int system_code = 0, std::unordered_map<SystemID, int> system_configs = { {SystemID::MISSION, 100}, {SystemID::CONTROL, 100}, {SystemID::MOTION, 100}, {SystemID::ENVIRONMENT, 50} });
    ~MainSystem();
    void init_() override;
    bool proxy_running_ = true;
    void start_test();

private:
    void start_proxy();

protected:
    void function() override;
    void publish() override;
    std::mutex command_mtx, system_mtx;
};

#endif // MAIN_H
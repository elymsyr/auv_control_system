#ifndef MAIN_H
#define MAIN_H

#include "system/subsystem.h"
#include "communication/topics.hpp"
#include "communication/communication_methods.h"
#include "system/environment_system.h"
#include "system/mission_system.h"
#include "system/motion_system.h"
#include "system/control_system.h"
#include <iostream>
#include <memory>
#include <mutex>
#include <zmq.hpp>

enum class SystemID {
    MAIN,
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
    SubscriberMain command_sub_;
    bool is_new_ = false;
    
    std::unordered_map<SystemID, std::unique_ptr<Subsystem>> systems_;
    std::unordered_map<SystemID, int> system_configs_;

    MissionTopic mission_state;
    EnvironmentTopic env_state;
    CommandTopic command_received;

public:
    MainSystem(std::string name = "Main", int runtime = 200, unsigned int system_code = 0, std::unordered_map<SystemID, int> system_configs = { {SystemID::MISSION, 2000}, {SystemID::CONTROL, 2000}, {SystemID::MOTION, 2000}, {SystemID::ENVIRONMENT, 2000} });
    ~MainSystem();
    void init_() override;
    void start_test();
    void waitForDestruction();

private:
    MissionSystem* mission_system;
    void parse_command(int system, int operation); 
    void halt() override;

protected:
    void function() override;
    void publish() override;
    std::mutex command_mtx, system_mtx, keep_alive_mtx_;
    std:: condition_variable keep_alive_cv_;
};

#endif // MAIN_H
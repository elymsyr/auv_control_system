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

struct System {
    Subsystem& system;
    Subscriber<StateTopic>& subscriber;
    unsigned int order: 3;

public:
    System(Subsystem& sys, Subscriber<StateTopic>& sub, unsigned int ord);
};

class MainSystem : public Subsystem {
    Subscriber<MissionTopic> mission_sub_;
    Subscriber<EnvironmentTopic> env_sub_;
    Subscriber<CommandTopic> command_sub_;

    std::unordered_map<SystemID, std::unique_ptr<System>> systems_;
    std::unordered_map<SystemID, int> system_configs_;

    MissionTopic mission_state;
    EnvironmentTopic env_state;
    CommandTopic command_received;

public:
    MainSystem(std::string name = "Main", int runtime = 200, unsigned int system_code = 0, std::unordered_map<SystemID, int> system_configs = { {SystemID::MISSION, 100}, {SystemID::CONTROL, 100}, {SystemID::MOTION, 100}, {SystemID::ENVIRONMENT, 50} });

    void init() override;

protected:
    void function() override;

    void publish() override;

private:
    void manage_systems(const std::vector<SystemID>& systems = {}, Operation operation);

    void sub_system_receive();

    void sub_system_connect();

    void bind_system_pub() override;
};

#endif // MAIN_H
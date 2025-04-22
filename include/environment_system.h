#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>

class EnvironmentSystem : public Subsystem {
    Publisher<EnvironmentTopic> env_pub_;
    
public:
    EnvironmentTopic env_state;

    EnvironmentSystem(std::string name = "Environment", int runtime = 50, unsigned int system_code = 0);
    void init() override;
    void halt() override;

protected:
    void function() override;
    void publish() override;
};

#endif // ENVIRONMENT_H
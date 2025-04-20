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
    void set_env_publish(bool publish = false) {
        std::lock_guard lk(mtx);
        env_publish = publish;
    }

    EnvironmentSystem(std::string name = "Environment", int runtime = 50, unsigned int system_code = 0)
        : Subsystem(name, runtime, system_code)
    {
        env_pub_.bind("tcp://*:5560");
    }

    void init() override {
        std::cout << name << " initialized\n";
    }

protected:
    bool env_publish = true;
    void function() override {
        {
            std::lock_guard lk(mtx);
            system_state.set();
            env_state.set();
        }
    }

    void postState() override {
        std::shared_lock lock(topic_read_mutex);
        state_pub_.publish(system_state);
        if (env_publish) { env_pub_.publish(env_state); }
    }
};

#endif // ENVIRONMENT_H
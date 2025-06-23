#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "subsystem.h"
#include "topics.hpp"
#include "communication_system.h"
#include "vehicle_model.h"
#include <iostream>
#include <chrono>
#include <zmq_addon.hpp>
#include <any>
#include <casadi/casadi.hpp>
#include <array> // Include array header

using namespace casadi;

class EnvironmentSystem : public Subsystem {
    Publisher<EnvironmentTopic> env_pub_;
    Subscriber<MotionTopic> motion_sub_;
    
public:
    EnvironmentTopic env_state;
    MotionTopic motion_state;
    VehicleModel vehicle_model;
    Function dyn_func;
    
    EnvironmentSystem(std::string name = "Environment", int runtime = 50, unsigned int system_code = 0);
    void init_() override;
    void halt() override;

protected:
    void function() override;
    void publish() override;
    std::mutex motion_mtx;

private:
    MX eta_sym = MX::sym("eta", 6);
    MX nu_sym   = MX::sym("nu", 6);
    MX u_sym    = MX::sym("u", 8);
    double dt;
};

#endif // ENVIRONMENT_H
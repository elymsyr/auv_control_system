#include "environment_system.h"
#include "vehicle_model.h"
#include <casadi/casadi.hpp>
#include <algorithm>

using namespace casadi;

EnvironmentSystem::EnvironmentSystem(std::string name, int runtime, unsigned int system_code)
    : vehicle_model("/home/eren/GitHub/Simulation/Control/config.json"), motion_sub_(motion_state, motion_mtx), Subsystem(name, runtime, system_code)
{
    // Create dynamics function during construction
    auto dyn_pair = vehicle_model.dynamics(eta_sym, nu_sym, u_sym);
    dyn_func = Function("dyn_func", {eta_sym, nu_sym, u_sym}, 
                       {dyn_pair.first, dyn_pair.second});

    env_state.set();
    motion_state.set();
    for (int i = 0; i < 20 && !env_pub_.is_bound(); ++i) {
        env_pub_.bind("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    dt = runtime / 1000.0;
}

void EnvironmentSystem::init_() {
    for (int i = 0; i < 20 && !motion_sub_.is_running() ; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        motion_sub_.connect("tcp://localhost:5563");
    }
}

void EnvironmentSystem::function() {
    {
        std::lock_guard<std::mutex> motion_lock(motion_mtx);
        std::cout << "Propeller values: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << motion_state.propeller[i] << " ";
        }
        std::cout << std::endl;
    }
    {
        std::lock_guard<std::mutex> env_lock(mtx);
        std::cout << "Environment eta: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << env_state.eta[i] << " ";
        }
        std::cout << "\nEnvironment nu: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << env_state.nu[i] << " ";
        }
        std::cout << "\nEnvironment nu_dot: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << env_state.nu_dot[i] << " ";
        }
        std::cout << std::endl;
    }
    std::lock_guard lk(mtx);

    // Copy propeller values safely
    std::array<double, 6> propeller_copy;
    {
        std::lock_guard<std::mutex> motion_lock(motion_mtx);
        std::copy(std::begin(motion_state.propeller), 
                 std::end(motion_state.propeller),
                 propeller_copy.begin());
    }

    // Prepare input vectors
    std::vector<double> eta_vec(std::begin(env_state.eta), std::end(env_state.eta));
    std::vector<double> nu_vec(std::begin(env_state.nu), std::end(env_state.nu));
    
    // Pad propeller array to 8 elements if needed
    std::vector<double> prop_vec(8, 0.0);
    std::copy(propeller_copy.begin(), propeller_copy.end(), prop_vec.begin());

    // Evaluate dynamics
    std::vector<DM> args = {DM(eta_vec), DM(nu_vec), DM(prop_vec)};
    std::vector<DM> res = dyn_func(args);

    // Extract derivatives
    std::vector<double> eta_dot_vec = res[0].get_elements();
    std::vector<double> nu_dot_vec = res[1].get_elements();

    // Update state using Euler integration
    for (int i = 0; i < 6; i++) {
        env_state.eta[i] += eta_dot_vec[i] * dt;
        env_state.nu[i] += nu_dot_vec[i] * dt;
    }
}

void EnvironmentSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    env_pub_.publish(env_state);
}

void EnvironmentSystem::halt() {
    try {
        motion_sub_.close();
        env_pub_.close();
        initialized = false;
    } catch (const std::exception& e) {
        std::cerr << "Halt failed for " << name << ": " << e.what() << "\n";
    }
}
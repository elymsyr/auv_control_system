#include "system/motion_system.h"
#include "communication/topics.hpp"
#include "communication/communication_methods.h"

MotionSystem::MotionSystem(std::string name, int runtime, unsigned int system_code) 
    : Subsystem(name, runtime, system_code),
      mpc(),
      mission_sub_(mission_state, mission_mtx),
      vehicle_model_("../config.json"),
      env_sub_(env_state, env_mtx)
{
    for (int i = 0; i < 20 && !motion_pub_.is_bound() ; i++) {
        motion_pub_.bind("tcp://localhost:5563");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mission_state.set();
    env_state.set();
}

void MotionSystem::init_() {
    for (int i = 0; i < 20 && !mission_sub_.is_running() ; i++) {
        mission_sub_.connect("tcp://localhost:5561");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (int i = 0; i < 20 && !env_sub_.is_running() ; i++) {
        env_sub_.connect("tcp://localhost:5560");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mpc.initialization();
}

void MotionSystem::function() {
    const float dt = 0.1f;
    try {
        {
            std::lock_guard<std::mutex> lk(mtx);
            // x0 is the current state from the environment
            x0 = env_state;
            // x_ref is the desired trajectory from the mission
            x_ref = mission_state;
        }

        // mpc.solve returns the optimal propeller forces (tau_p)
        std::array<double, 8> propeller = mpc.solve(x0, x_ref);

        // --- Calculate the next state ---

        // 1. Get current state variables from x0
        casadi::DM eta_current_dm = casadi::DM(std::vector<double>(std::begin(x0.eta), std::end(x0.eta)));
        casadi::DM nu_current_dm = casadi::DM(std::vector<double>(std::begin(x0.nu), std::end(x0.nu)));

        // Convert propeller forces to CasADi type
        casadi::DM tau_p_dm = casadi::DM(std::vector<double>(propeller.begin(), propeller.end()));

        // Convert DM to MX for the dynamics function if needed, or adapt dynamics to take DM
        casadi::MX eta_current_mx = casadi::MX(eta_current_dm);
        casadi::MX nu_current_mx = casadi::MX(nu_current_dm);
        casadi::MX tau_p_mx = casadi::MX(tau_p_dm);

        // 2. Call the dynamics function to get the derivatives
        std::pair<casadi::MX, casadi::MX> derivatives = vehicle_model_.dynamics(eta_current_mx, nu_current_mx, tau_p_mx);
        casadi::MX eta_dot_mx = derivatives.first;
        casadi::MX nu_dot_mx = derivatives.second;

        // 3. Perform forward Euler integration to find the next state
        casadi::MX eta_next_mx = eta_current_mx + dt * eta_dot_mx;
        casadi::MX nu_next_mx = nu_current_mx + dt * nu_dot_mx;

        // 4. Convert the result back to a usable C++ type
        casadi::DM eta_next_dm = casadi::DM(eta_next_mx);
        casadi::DM nu_next_dm = casadi::DM(nu_next_mx);
        std::vector<double> eta_next_vec = eta_next_dm.get_elements();
        std::vector<double> nu_next_vec = nu_next_dm.get_elements();

        std::array<double, 12> next_state;
        std::copy(eta_next_vec.begin(), eta_next_vec.end(), next_state.begin());
        std::copy(nu_next_vec.begin(), nu_next_vec.end(), next_state.begin() + 6);

        {
            std::lock_guard<std::mutex> lk(mtx);
            // Now you can use the calculated next_state
            motion_state.set(propeller, next_state);
        }
    } catch (const std::exception& e) {
        std::cerr << "MotionSystem error: " << e.what() << std::endl;
    }
}

void MotionSystem::publish() {
    std::shared_lock lock(topic_read_mutex);
    motion_pub_.publish(motion_state);
}

void MotionSystem::halt() {
    motion_pub_.close();
    mission_sub_.close();
    env_sub_.close();
    initialized = false;
}

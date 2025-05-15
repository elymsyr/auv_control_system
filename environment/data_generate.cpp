#include <random>
#include <casadi/casadi.hpp>
#include <iostream>
#include "casadi_mpc.hpp"
using namespace casadi;

// --- Global random number generator ---
std::random_device rd;
std::mt19937 gen(rd());

// --- Helper functions for random numbers ---
double rand_uniform(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

double rand_near_zero(double abs_max) {
    return rand_uniform(-abs_max, abs_max);
}

// --- Generate X_current (eta=0, nu/nu_dot randomized) ---
DM generate_X_current() {
    // Parameters for randomization
    const double nu_data_min = -5.0;   // Adjust based on your AUV specs
    const double nu_data_max = 5.0;
    DM eta = DM::vertcat({
        0,0,0,
        rand_near_zero(M_PI/2),        // phi (roll)
        rand_near_zero(M_PI/30),        // theta (pitch)
        rand_uniform(-M_PI/2, M_PI/2)      // psi (yaw)
    });

    // Generate nu (linear and angular velocities)
    DM nu = DM::vertcat({
        rand_uniform(nu_data_min, nu_data_max),  // nu_x
        rand_uniform(nu_data_min, nu_data_max),  // nu_y
        rand_uniform(nu_data_min, nu_data_max),  // nu_z
        rand_near_zero(0.05),                    // nu_mx (close to 0)
        rand_near_zero(0.05),                    // nu_my (close to 0)
        rand_uniform(-0.1, 0.1)                  // nu_mz
    });

    return DM::vertcat({eta, nu});
}

// --- Generate X_desired (nu/nu_dot mxmy=0) ---
DM generate_X_desired() {
    // Desired eta (randomized within operational range)
    const double nu_data_min = -5.0;   // Adjust based on your AUV specs
    const double nu_data_max = 5.0;
    DM eta_d = DM::vertcat({
        rand_uniform(-10.0, 10.0),  // nu_x
        rand_uniform(-10.0, 10.0),  // nu_y
        rand_uniform(-10.0, 10.0),  // nu_z
        0,0,
        rand_uniform(-M_PI/2, M_PI/2)
    });

    // Desired nu (mx/my = 0)
    DM nu_d = DM::vertcat({
        rand_uniform(nu_data_min, nu_data_max),  // nu_x
        rand_uniform(nu_data_min, nu_data_max),  // nu_y
        rand_uniform(nu_data_min, nu_data_max),  // nu_z
        0.0,                      // nu_mx
        0.0,                      // nu_my
        rand_uniform(-1.0, 1.0)   // nu_mz
    });

    return DM::vertcat({eta_d, nu_d});
}

// --- Serialize DM to CSV string ---
std::string serialize_dm(const DM& m) {
    std::stringstream ss;
    for (casadi_int i = 0; i < m.size1(); ++i) {
        ss << static_cast<double>(m(i));
        if (i != m.size1() - 1) ss << ",";
    }
    return ss.str();
}

int main() {
    try {
        // Initialize vehicle model
        VehicleModel model("config.json");

        // Initialize MPC controller
        NonlinearMPC mpc(model);

        // Open data file
        std::ofstream data_file("mpc_data.csv");
        data_file << "x_current,x_desired,u_opt,x_opt\n";
        DM x_next = DM::zeros(12, 1);
        // Main data collection loop
        for (int test = 0; test < 10000000; ++test) {
            DM x0 = generate_X_current();
            DM x_desired = generate_X_desired();
            DM x_ref = DM::repmat(x_desired, 1, 11);
            for (int step = 0; step < 10; ++step) {
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                x_next = x_opt(Slice(), 1);
                data_file << "'" << serialize_dm(x0) << "',"
                          << "'" << serialize_dm(x_desired) << "',"
                          << "'" << serialize_dm(u_opt) << "',"
                          << "'" << serialize_dm(x_next) << "'\n";
                x0 = x_next;
            }
        }
        data_file.close();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

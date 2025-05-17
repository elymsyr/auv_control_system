// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" data_generate.cpp -lhdf5 -lhdf5_cpp -lz -ldl -lm -lcasadi -lipopt -lzmq -o data_generate

#include <H5Cpp.h>
#include <random>
#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include "casadi_mpc.hpp"
using namespace casadi;
using namespace H5;

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
    const double nu_data_min = -2.5;   // Adjust based on your AUV specs
    const double nu_data_max = 2.5;
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

std::vector<double> dm_to_vector(const DM& m) {
    std::vector<double> vec;
    for (casadi_int i = 0; i < m.size1(); ++i) {
        vec.push_back(static_cast<double>(m(i)));
    }
    return vec;
}

int main() {
    try {
        // Initialize vehicle model and MPC
        VehicleModel model("config.json");
        NonlinearMPC mpc(model);

        // Data buffers
        std::vector<double> x_current_buf, x_desired_buf, u_opt_buf, x_opt_buf;

        // Main data collection loop
        for (int test = 0; test < 10000; ++test) {
            DM x_desired = generate_X_desired();
            DM x_ref = DM::repmat(x_desired, 1, 11);
            DM x0 = generate_X_current();

            for (int step = 0; step < 100; ++step) {
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                DM x_next = x_opt(Slice(), 1);

                // Store in buffers
                auto x0_vec = dm_to_vector(x0);
                auto xd_vec = dm_to_vector(x_desired);
                auto u_vec = dm_to_vector(u_opt);
                auto xn_vec = dm_to_vector(x_next);

                x_current_buf.insert(x_current_buf.end(), x0_vec.begin(), x0_vec.end());
                x_desired_buf.insert(x_desired_buf.end(), xd_vec.begin(), xd_vec.end());
                u_opt_buf.insert(u_opt_buf.end(), u_vec.begin(), u_vec.end());
                x_opt_buf.insert(x_opt_buf.end(), xn_vec.begin(), xn_vec.end());

                x0 = x_next;
            }
        }

        // Create HDF5 file
        H5File file("mpc_data.h5", H5F_ACC_TRUNC);

        // Define dataset dimensions (10 samples x N features)
        const hsize_t n_samples = 10;
        const hsize_t xdims[2] = {n_samples, 12};  // For 12D states
        const hsize_t udims[2] = {n_samples, 6};   // For 6D controls

        // Create dataspace
        DataSpace x_space(2, xdims);
        DataSpace u_space(2, udims);

        // Create datasets
        DataSet ds_xcurr = file.createDataSet("x_current", 
                            PredType::NATIVE_DOUBLE, x_space);
        DataSet ds_xdes = file.createDataSet("x_desired", 
                           PredType::NATIVE_DOUBLE, x_space);
        DataSet ds_uopt = file.createDataSet("u_opt", 
                           PredType::NATIVE_DOUBLE, u_space);
        DataSet ds_xnext = file.createDataSet("x_opt", 
                            PredType::NATIVE_DOUBLE, x_space);

        // Write data
        ds_xcurr.write(x_current_buf.data(), PredType::NATIVE_DOUBLE);
        ds_xdes.write(x_desired_buf.data(), PredType::NATIVE_DOUBLE);
        ds_uopt.write(u_opt_buf.data(), PredType::NATIVE_DOUBLE);
        ds_xnext.write(x_opt_buf.data(), PredType::NATIVE_DOUBLE);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

#include <H5Cpp.h>
#include <random>
#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include "casadi_mpc.hpp"
#include <csignal>
#include <atomic>
using namespace casadi;
using namespace H5;

std::atomic<bool> shutdown_requested(false);
// --- Signal handler ---
void sigint_handler(int) {
    std::cout << "\nShutdown requested, finishing current work...\n";
    shutdown_requested = true;
}

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

// --- Configuration ---
const int CHUNK_SIZE = 1000;      // Write every 1000 samples
const hsize_t MAX_DIMS[2] = {H5S_UNLIMITED, 12};  // Extendable dimensions
const hsize_t CHUNK_DIMS[2] = {1000, 12};         // Chunk size for HDF5
const hsize_t U_CHUNK[2] = {1000, 6};             // Control chunk size

// --- Global HDF5 handles ---
H5File* file = nullptr;
DataSet* ds_xcurr = nullptr;
DataSet* ds_xdes = nullptr;
DataSet* ds_uopt = nullptr;
DataSet* ds_xnext = nullptr;
hsize_t current_size[2] = {0, 12};  // Current dataset dimensions

// --- Data buffers ---
std::vector<double> x_current_buf, x_desired_buf, u_opt_buf, x_opt_buf;

void write_chunk() {
    try {
        if (x_current_buf.empty()) return;

        // Determine number of new samples
        const hsize_t n_new = x_current_buf.size() / 12;
        
        // Extend datasets
        current_size[0] += n_new;
        ds_xcurr->extend(current_size);
        ds_xdes->extend(current_size);
        hsize_t u_new_size[2] = {current_size[0], 6};
        ds_uopt->extend(u_new_size);
        ds_xnext->extend(current_size);

        // Select hyperslab
        hsize_t offset[2] = {current_size[0] - n_new, 0};
        hsize_t count[2] = {n_new, 12};
        DataSpace mem_space(2, count);
        
        // Write x_current
        DataSpace file_space = ds_xcurr->getSpace();
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        ds_xcurr->write(x_current_buf.data(), PredType::NATIVE_DOUBLE, 
                       mem_space, file_space);

        // Write x_desired
        file_space = ds_xdes->getSpace();
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        ds_xdes->write(x_desired_buf.data(), PredType::NATIVE_DOUBLE, 
                       mem_space, file_space);

        // Write u_opt
        hsize_t u_count[2] = {n_new, 6};
        DataSpace u_mem_space(2, u_count);
        file_space = ds_uopt->getSpace();
        file_space.selectHyperslab(H5S_SELECT_SET, u_count, offset);
        ds_uopt->write(u_opt_buf.data(), PredType::NATIVE_DOUBLE, 
                       u_mem_space, file_space);

        // Write x_opt
        file_space = ds_xnext->getSpace();
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        ds_xnext->write(x_opt_buf.data(), PredType::NATIVE_DOUBLE, 
                       mem_space, file_space);

        // Clear buffers
        x_current_buf.clear();
        x_desired_buf.clear();
        u_opt_buf.clear();
        x_opt_buf.clear();

    } catch (const Exception& e) {
        std::cerr << "HDF5 Write Error: " << e.getCDetailMsg() << "\n";
        exit(1);
    }
}

void cleanup_hdf5() {
    try {
        // Write any remaining data first
        if (!x_current_buf.empty()) {
            write_chunk();
        }
        
        // Explicitly flush before closing
        if (file) {
            H5Fflush(file->getId(), H5F_SCOPE_GLOBAL);
        }

        // Then delete resources
        if (ds_xnext) { delete ds_xnext; ds_xnext = nullptr; }
        if (ds_uopt)  { delete ds_uopt;  ds_uopt  = nullptr; }
        if (ds_xdes)  { delete ds_xdes;  ds_xdes  = nullptr; }
        if (ds_xcurr) { delete ds_xcurr; ds_xcurr = nullptr; }
        if (file)     { delete file;     file     = nullptr; }
        
    } catch (...) {
        std::cerr << "Error during final cleanup\n";
    }
}

void initialize_hdf5() {
    try {
        file = new H5File("mpc_data.h5", H5F_ACC_TRUNC);
        
        // Create dataspace with initial size (0,12) and max size (unlimited,12)
        hsize_t init_dims[2] = {0, 12};
        hsize_t max_dims[2] = {H5S_UNLIMITED, 12};
        DataSpace x_space(2, init_dims, max_dims);

        hsize_t u_init[2] = {0, 6};
        hsize_t u_max[2] = {H5S_UNLIMITED, 6};
        DataSpace u_space(2, u_init, u_max);

        // Rest of the code remains the same...
        DSetCreatPropList x_props;
        x_props.setChunk(2, CHUNK_DIMS);
        x_props.setDeflate(6);
        
        DSetCreatPropList u_props;
        u_props.setChunk(2, U_CHUNK);
        u_props.setDeflate(6);

        // Create datasets with corrected dataspaces
        ds_xcurr = new DataSet(file->createDataSet(
            "x_current", PredType::NATIVE_DOUBLE, x_space, x_props));
        ds_xdes = new DataSet(file->createDataSet(
            "x_desired", PredType::NATIVE_DOUBLE, x_space, x_props));
        ds_uopt = new DataSet(file->createDataSet(
            "u_opt", PredType::NATIVE_DOUBLE, u_space, u_props));
        ds_xnext = new DataSet(file->createDataSet(
            "x_opt", PredType::NATIVE_DOUBLE, x_space, x_props));
        
    } catch (const Exception& e) {
        std::cerr << "HDF5 Error: " << e.getCDetailMsg() << "\n";
        exit(1);
    }
}

int main() {
    std::signal(SIGINT, sigint_handler);
    try {
        initialize_hdf5();
        
        // Initialize vehicle model and MPC
        VehicleModel model("config.json");
        NonlinearMPC mpc(model);

        // Main data collection loop
        for (int test = 0; test < 100000 && !shutdown_requested; ++test) {
            DM x_desired = generate_X_desired();
            DM x_ref = DM::repmat(x_desired, 1, 11);
            DM x0 = generate_X_current();

            for (int step = 0; step < 10 && !shutdown_requested; ++step) {
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                DM x_next = x_opt(Slice(), 1);

                // Append to buffers
                auto x0_vec = dm_to_vector(x0);
                auto xd_vec = dm_to_vector(x_desired);
                auto u_vec = dm_to_vector(u_opt);
                auto xn_vec = dm_to_vector(x_next);

                x_current_buf.insert(x_current_buf.end(), x0_vec.begin(), x0_vec.end());
                x_desired_buf.insert(x_desired_buf.end(), xd_vec.begin(), xd_vec.end());
                u_opt_buf.insert(u_opt_buf.end(), u_vec.begin(), u_vec.end());
                x_opt_buf.insert(x_opt_buf.end(), xn_vec.begin(), xn_vec.end());

                if (test % 100 == 0) {
                    std::cout << "Processed " << test << " tests ("
                              << current_size[0] << " samples)\n";
                    H5Fflush(file->getId(), H5F_SCOPE_GLOBAL);
                }

                x0 = x_next;
            }
            std::cout << "Remaining Steps: " << 100000 - test << "\n";
        }

        // Write remaining data
        write_chunk();

        // Cleanup
        delete ds_xnext;
        delete ds_uopt;
        delete ds_xdes;
        delete ds_xcurr;
        delete file;
        cleanup_hdf5();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        cleanup_hdf5();
        return 1;
    }
    cleanup_hdf5();
    return 0;
}
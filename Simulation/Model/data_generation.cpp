// cd /home/eren/GitHub/Simulation/Model
// rm -f *.o *.so jit_* libdynamics_func* *.bin
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c environment_helper.cpp -o environment_helper.o
// nvcc -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" data_generation.cpp -o data_generation.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o data_generation environment_helper.o environment_map.o environment_astar.o environment_global.o data_generation.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lglfw -lGL -lcasadi -lipopt -lzmq -lcudart -lhdf5 -lhdf5_cpp -lz -ldl -lm -L/usr/local/cuda/lib64
// ./data_generation
// python /home/eren/GitHub/Simulation/Model/visualize.py
// rm -f *.o *.so jit_* libdynamics_func*


// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" data_generation.cpp -o data_generation.o
// g++ -o data_generation environment_helper.o environment_map.o environment_astar.o environment_global.o data_generation.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lglfw -lGL -lcasadi -lipopt -lzmq -lcudart -lhdf5 -lhdf5_cpp -lz -ldl -lm -L/usr/local/cuda/lib64
// ./data_generation
// python /home/eren/GitHub/Simulation/Model/visualize.py

#include "environment.h"
#include <H5Cpp.h>
#include <random>
#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include "nlmpc.h"
#include "vehicle_model.h"
#include <csignal>
#include <atomic>
#include <sys/stat.h>

using namespace casadi;
using namespace H5;

// Constants
const int WIDTH = 129;
const int HEIGHT = 129;
const int N = 40;  // MPC horizon
const int MAX_OBSTACLES = 20;
const int MIN_OBSTACLES = 5;
const float OBSTACLE_THRESHOLD = 250.0f;
const float COLLISION_THRESHOLD = 0.5f;
const int MAX_STEPS_PER_SCENARIO = 200;
const int MAX_SCENARIOS = 1;
const int CHUNK_SIZE = 100;
const std::string HDF_PATH = "astar_mpc_data.h5";

// Global state
std::atomic<bool> shutdown_requested(false);
std::random_device rd;
std::mt19937 gen(rd());

// HDF5 handles
H5File* file = nullptr;
DataSet* ds_xcurr = nullptr;
DataSet* ds_xref = nullptr;
DataSet* ds_uopt = nullptr;
DataSet* ds_xnext = nullptr;
hsize_t current_size[2] = {0, 12};
hsize_t ref_dims[2] = {0, 12*(N+1)};

const hsize_t STATE_CHUNK[2] = {100, 12};
const hsize_t REF_CHUNK[2] = {100, 12*(N+1)};
const hsize_t U_CHUNK[2] = {100, 8};

// Data buffers
std::vector<double> x_current_buf, x_ref_buf, u_opt_buf, x_next_buf;

// Signal handler
void sigint_handler(int) {
    std::cout << "\nShutdown requested, finishing current work...\n";
    shutdown_requested = true;
}

// Helper functions
double rand_uniform(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

double rand_uniform_step(double min, double max, bool negate = true) {
    int n_steps = static_cast<int>((max - min) / 0.1);
    int idx = std::uniform_int_distribution<int>(0, n_steps)(gen);
    double val = min + idx * 0.1;
    return negate && rand() % 2 == 0 ? -val : val;
}

double rand_near(double abs_max) {
    return rand_uniform(-abs_max, abs_max);
}

DM generate_X_current() {
    DM eta = DM::vertcat({
        0, 0, 0,
        rand_near(M_PI/10),
        rand_near(M_PI/10),
        rand_near(M_PI)
    });

    DM nu = DM::vertcat({
        rand_near(2.0),
        rand_near(2.0),
        rand_near(1.0),
        rand_near(0.001),
        rand_near(0.001),
        rand_near(0.1)
    });

    return DM::vertcat({eta, nu});
}

std::vector<double> dm_to_vector(const DM& m) {
    std::vector<double> vec;
    for (casadi_int i = 0; i < m.size1(); ++i) {
        for (casadi_int j = 0; j < m.size2(); ++j) {
            vec.push_back(static_cast<double>(m(i, j)));
        }
    }
    return vec;
}

// HDF5 Management
void write_chunk() {
    if (!ds_xcurr || !ds_xref || !ds_uopt || !ds_xnext) {
        std::cerr << "Dataset pointers are invalid!" << std::endl;
        return;
    }
    if (x_current_buf.empty()) return;

    const hsize_t n_new = x_current_buf.size() / 12;
    
    // Extend datasets
    current_size[0] += n_new;
    ref_dims[0] += n_new;
    
    ds_xcurr->extend(current_size);
    ds_xref->extend(ref_dims);
    
    hsize_t u_new_size[2] = {current_size[0], 8};
    ds_uopt->extend(u_new_size);
    
    ds_xnext->extend(current_size);

    // Write x_current
    hsize_t offset[2] = {current_size[0] - n_new, 0};
    hsize_t count[2] = {n_new, 12};
    DataSpace mem_space(2, count);
    
    // Get updated file space after extension
    DataSpace file_space_xcurr = ds_xcurr->getSpace();
    file_space_xcurr.selectHyperslab(H5S_SELECT_SET, count, offset);
    ds_xcurr->write(x_current_buf.data(), PredType::NATIVE_DOUBLE, mem_space, file_space_xcurr);

    // Write x_ref
    hsize_t ref_count[2] = {n_new, 12*(N+1)};
    DataSpace ref_mem_space(2, ref_count);
    DataSpace file_space_xref = ds_xref->getSpace();
    file_space_xref.selectHyperslab(H5S_SELECT_SET, ref_count, offset);
    ds_xref->write(x_ref_buf.data(), PredType::NATIVE_DOUBLE, ref_mem_space, file_space_xref);

    // Write u_opt
    hsize_t u_count[2] = {n_new, 8};
    DataSpace u_mem_space(2, u_count);
    DataSpace file_space_uopt = ds_uopt->getSpace();
    file_space_uopt.selectHyperslab(H5S_SELECT_SET, u_count, offset);
    ds_uopt->write(u_opt_buf.data(), PredType::NATIVE_DOUBLE, u_mem_space, file_space_uopt);

    // Write x_next
    DataSpace file_space_xnext = ds_xnext->getSpace();
    file_space_xnext.selectHyperslab(H5S_SELECT_SET, count, offset);
    ds_xnext->write(x_next_buf.data(), PredType::NATIVE_DOUBLE, mem_space, file_space_xnext);

    // Clear buffers
    x_current_buf.clear();
    x_ref_buf.clear();
    u_opt_buf.clear();
    x_next_buf.clear();
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
        if (ds_xref)  { delete ds_xref;  ds_xref  = nullptr; }
        if (ds_xcurr) { delete ds_xcurr; ds_xcurr = nullptr; }
        if (file)     { delete file;     file     = nullptr; }
        
    } catch (...) {
        std::cerr << "Error during final cleanup\n";
    }
}

void initialize_hdf5() {
    try {
        struct stat buffer;
        bool file_exists = (stat(HDF_PATH.c_str(), &buffer) == 0);

        if(file_exists) {
            file = new H5File(HDF_PATH, H5F_ACC_RDWR);
            
            // Open existing datasets
            ds_xcurr = new DataSet(file->openDataSet("x_current"));
            ds_xref = new DataSet(file->openDataSet("x_ref"));
            ds_uopt = new DataSet(file->openDataSet("u_opt"));
            ds_xnext = new DataSet(file->openDataSet("x_next"));
            
        // Get current dimensions for x_ref
        {
            DataSpace space = ds_xref->getSpace();
            hsize_t dims[2];
            space.getSimpleExtentDims(dims);
            ref_dims[0] = dims[0];
            ref_dims[1] = dims[1];  // should be 12*(N+1)
            std::cout << "x_ref size: " << ref_dims[0] << " x " << ref_dims[1] << "\n";
        }
        // For u_opt, track its first dimension separately, e.g., u_dims
        {
            DataSpace space = ds_uopt->getSpace();
            hsize_t dims[2];
            space.getSimpleExtentDims(dims);
            // You may want a separate array, e.g. u_dims
            // But if you use current_size[0] for number of rows and second dim is fixed 8,
            // ensure they match or handle accordingly.
            std::cout << "u_opt size: " << dims[0] << " x " << dims[1] << "\n";
        }
        // x_next has same shape as x_current
        {
            DataSpace space = ds_xnext->getSpace();
            hsize_t dims[2];
            space.getSimpleExtentDims(dims);
            std::cout << "x_next size: " << dims[0] << " x " << dims[1] << "\n";
        }
        } else {
            file = new H5File(HDF_PATH, H5F_ACC_TRUNC);
            
            // Create dataspace for extendible datasets
            hsize_t init_dims[2] = {0, 12};
            hsize_t max_dims[2] = {H5S_UNLIMITED, 12};
            DataSpace x_space(2, init_dims, max_dims);
            
            hsize_t ref_init[2] = {0, 12*(N+1)};
            hsize_t ref_max[2] = {H5S_UNLIMITED, 12*(N+1)};
            DataSpace ref_space(2, ref_init, ref_max);
            
            hsize_t u_init[2] = {0, 8};
            hsize_t u_max[2] = {H5S_UNLIMITED, 8};
            DataSpace u_space(2, u_init, u_max);

            // Add chunking properties
            DSetCreatPropList props;
            props.setChunk(2, STATE_CHUNK);
            
            DSetCreatPropList ref_props;
            ref_props.setChunk(2, REF_CHUNK);
            
            DSetCreatPropList u_props;
            u_props.setChunk(2, U_CHUNK);

            // Create datasets with chunking
            ds_xcurr = new DataSet(file->createDataSet("x_current", 
                PredType::NATIVE_DOUBLE, x_space, props));
            ds_xref = new DataSet(file->createDataSet("x_ref", 
                PredType::NATIVE_DOUBLE, ref_space, ref_props));
            ds_uopt = new DataSet(file->createDataSet("u_opt", 
                PredType::NATIVE_DOUBLE, u_space, u_props));
            ds_xnext = new DataSet(file->createDataSet("x_next", 
                PredType::NATIVE_DOUBLE, x_space, props));
        }
    } catch (const Exception& e) {
        std::cerr << "HDF5 Error: " << e.getCDetailMsg() << "\n";
        exit(1);
    }
}

int main() {
    std::signal(SIGINT, sigint_handler);
    try {
        initialize_hdf5();
        VehicleModel model("config.json");
        NonlinearMPC mpc("config.json", N);
        mpc.initialization();

        for (int scenario = 0; scenario < MAX_SCENARIOS && !shutdown_requested; ++scenario) {
            // Create environment with random obstacles
            EnvironmentMap map(WIDTH, HEIGHT);
            int num_obstacles = MIN_OBSTACLES + rand() % (MAX_OBSTACLES - MIN_OBSTACLES);

            float ref_x = rand_uniform(-14.0f, 14.0f);
            float ref_y = rand_uniform(-14.0f, 14.0f);

            for (int i = 0; i < num_obstacles; ) {
                float x = rand_uniform(-14.0f, 14.0f);
                float y = rand_uniform(-14.0f, 14.0f);
                
                // Skip if near start/goal
                if (hypot(x, y) < 7.2f || 
                    hypot(x - ref_x, y - ref_y) < 7.2f) {
                    continue;
                }
                map.updateSinglePoint(x, y, 255.0f);
                i++;
            }

            DM x0 = generate_X_current();
            DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);
            map.set_ref(ref_x, ref_y);
            map.updateSinglePoint(ref_x, ref_y, 200.0f);
            double nu1 = static_cast<double>(x0(6));
            double nu2 = static_cast<double>(x0(7));
            map.set_velocity(nu1, nu2);

            std::cout << "Starting Velocity: " << nu1 << ", " << nu2 << "\n"; 
            
            for (int step = 0; step < MAX_STEPS_PER_SCENARIO && !shutdown_requested; ++step) {
                double eta1 = static_cast<double>(x0(0));
                double eta2 = static_cast<double>(x0(1));
                double eta6 = static_cast<double>(x0(5));

                // Update map with current position
                map.updateSinglePoint(eta1, eta2, 130.0f);
                map.slide(eta1, eta2);

                // Generate path with A*
                Path path = map.findPath();
                if (path.length == 0) {
                    std::cerr << "No path found!\n";
                    break;
                }

                // Create reference trajectory
                int m = std::min(path.length, N+1);
                float spacing = static_cast<float>(std::min((m-1) / N, 4));
                std::vector<float2> path_points;
                
                for (int k = 0; k < m; k++) {
                    path_points.push_back(createPath(m, k, spacing, map, path));
                }
                
                for (int k = 0; k <= N; k++) {
                    // Get current position in reference trajectory
                    float2 world_coor = (k < m) ? path_points[k] : make_float2(ref_x, ref_y);
                    
                    // Calculate yaw using improved method
                    float angle;
                    if (k < m - 1) {
                        // Use next point in path
                        float2 next_coor = path_points[k+1];
                        angle = atan2f(next_coor.y - world_coor.y, next_coor.x - world_coor.x);
                    }
                    else if (k == m - 1 && m >= 2) {
                        // Use previous point for last path point
                        float2 prev_coor = path_points[k-1];
                        angle = atan2f(world_coor.y - prev_coor.y, world_coor.x - prev_coor.x);
                    }
                    else {
                        // For beyond path or single-point paths
                        angle = atan2f(ref_y - eta2, ref_x - eta1);
                    }

                    // Handle degenerate cases
                    if (std::isnan(angle)) {
                        angle = (k > 0) ? static_cast<double>(x_ref(5, k-1)) : eta6;
                        std::cerr << "Warning: NaN angle at step " << step << ", using previous angle: " << angle << "\n";
                    }

                    // Apply low-pass filter for smoother transitions
                    if (k > 0) {
                        float prev_angle = static_cast<double>(x_ref(5, k-1));
                        float diff = angle - prev_angle;
                        
                        // Normalize angle difference to [-π, π]
                        if (diff > M_PI) diff -= 2*M_PI;
                        if (diff < -M_PI) diff += 2*M_PI;
                        
                        // Apply smoothing (adjust 0.2-0.3 for different responsiveness)
                        angle = prev_angle + 0.3 * diff;
                    }

                    // Set references
                    x_ref(0, k) = world_coor.x;
                    x_ref(1, k) = world_coor.y;
                    x_ref(5, k) = angle;
                    if (step == 1) map.updateSinglePoint(world_coor.x, world_coor.y, 49.0f);
                }

                // Solve MPC
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                DM x_next = x_opt(Slice(), 1);

                // Check solution validity
                bool valid_solution = true;
                for (int i = 0; i < 8; i++) {
                    if (!std::isfinite(static_cast<double>(u_opt(i, 0)))) {
                        valid_solution = false;
                        break;
                    }
                }
                
                if (!valid_solution) {
                    std::cerr << "Invalid MPC solution\n";
                    break;
                }

                std::cout << "\nStep " << step << ":\n  World Positions: " << map.world_position_.x << ", " << map.world_position_.y << "\n"
                            << "  Reference: " << map.ref_.x << ", " << map.ref_.y << "\n"
                            << "  State: " << x0 << "\n";
                
                if (step >= MAX_STEPS_PER_SCENARIO-1 || step%55 == 0 || step == 2) map.save(std::to_string(step));

                // Store data
                auto x0_vec = dm_to_vector(x0);
                auto x_ref_vec = dm_to_vector(x_ref);
                auto u_opt_vec = dm_to_vector(u_opt(Slice(), 0));
                auto x_next_vec = dm_to_vector(x_next);
                
                x_current_buf.insert(x_current_buf.end(), x0_vec.begin(), x0_vec.end());
                x_ref_buf.insert(x_ref_buf.end(), x_ref_vec.begin(), x_ref_vec.end());
                u_opt_buf.insert(u_opt_buf.end(), u_opt_vec.begin(), u_opt_vec.end());
                x_next_buf.insert(x_next_buf.end(), x_next_vec.begin(), x_next_vec.end());

                if (x_current_buf.size() >= CHUNK_SIZE * 12) {
                    write_chunk();
                }

                x0 = x_next;

                double dist_to_goal = sqrt(pow(static_cast<double>(x0(0)) - ref_x, 2) + 
                                     pow(static_cast<double>(x0(1)) - ref_y, 2));
                if (dist_to_goal < COLLISION_THRESHOLD) {
                    std::cout << "\nGoal reached!\n";
                }
            }
            
            std::cout << "\nCompleted scenario " << scenario << "/" << MAX_SCENARIOS << "\n";
        }
        
        cleanup_hdf5();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        cleanup_hdf5();
        return 1;
    }
    return 0;
}
// cd /home/eren/GitHub/Simulation/Model
// rm -f *.o jit_* data_generation_v
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c environment_helper.cpp -o environment_helper.o
// nvcc -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" data_generation_v.cpp -o data_generation_v.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o data_generation_v environment_helper.o environment_map.o environment_astar.o environment_global.o data_generation_v.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lglfw -lGL -lcasadi -lipopt -lzmq -lcudart -lhdf5 -lhdf5_cpp -lz -ldl -lm -L/usr/local/cuda/lib64
// ./data_generation_v


// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" data_generation_v.cpp -o data_generation_v.o
// g++ -o data_generation_v environment_helper.o environment_map.o environment_astar.o environment_global.o data_generation_v.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lglfw -lGL -lcasadi -lipopt -lzmq -lcudart -lhdf5 -lhdf5_cpp -lz -ldl -lm -L/usr/local/cuda/lib64
// ./data_generation_v

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
#include <GLFW/glfw3.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <algorithm>

using namespace casadi;
using namespace H5;

// Constants
const int WIDTH = 129;
const int HEIGHT = 129;
const int N = 20;  // MPC horizon
const int MAX_OBSTACLES = 50;
const int MIN_OBSTACLES = 10;
const float OBSTACLE_THRESHOLD = 250.0f;
const float COLLISION_THRESHOLD = 0.5f;
const int MAX_STEPS_PER_SCENARIO = 40;
const int MAX_SCENARIOS = 10000;
const int CHUNK_SIZE = 100;
const std::string HDF_PATH = "astar_mpc_data.h5";

// Visualization constants
const int VIS_WIDTH = 800;
const int VIS_HEIGHT = 800;

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

// Visualization data
struct VisualizationData {
    std::vector<float2> obstacles;
    std::vector<float2> path_points;
    float2 vehicle_pos = {0.0f, 0.0f};
    float vehicle_yaw = 0.0f;
    float2 goal_pos = {0.0f, 0.0f};
};

VisualizationData vis_data;
std::mutex vis_mutex;
GLFWwindow* window = nullptr;

// Signal handler
void sigint_handler(int) {
    std::cout << "\nShutdown requested, finishing current work...\n";
    shutdown_requested = true;
    if (window) glfwSetWindowShouldClose(window, GLFW_TRUE);
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
        rand_uniform(-5.0, 5.0),
        rand_uniform(-5.0, 5.0),
        rand_uniform_step(2.0, 25.0, false),
        rand_near(M_PI/4),
        rand_near(M_PI/4),
        rand_uniform(-M_PI, M_PI)
    });

    DM nu = DM::vertcat({
        rand_uniform(0.0, 5.0),
        rand_uniform(0.0, 5.0),
        rand_uniform(0.0, 5.0),
        rand_near(0.05),
        rand_near(0.05),
        rand_uniform(-0.1, 0.1)
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
    
    DataSpace file_space = ds_xcurr->getSpace();
    file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
    ds_xcurr->write(x_current_buf.data(), PredType::NATIVE_DOUBLE, mem_space, file_space);

    // Write x_ref
    hsize_t ref_count[2] = {n_new, 12*(N+1)};
    DataSpace ref_mem_space(2, ref_count);
    file_space = ds_xref->getSpace();
    file_space.selectHyperslab(H5S_SELECT_SET, ref_count, offset);
    ds_xref->write(x_ref_buf.data(), PredType::NATIVE_DOUBLE, ref_mem_space, file_space);

    // Write u_opt
    hsize_t u_count[2] = {n_new, 8};
    DataSpace u_mem_space(2, u_count);
    file_space = ds_uopt->getSpace();
    file_space.selectHyperslab(H5S_SELECT_SET, u_count, offset);
    ds_uopt->write(u_opt_buf.data(), PredType::NATIVE_DOUBLE, u_mem_space, file_space);

    // Write x_next
    file_space = ds_xnext->getSpace();
    file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
    ds_xnext->write(x_next_buf.data(), PredType::NATIVE_DOUBLE, mem_space, file_space);

    // Clear buffers
    x_current_buf.clear();
    x_ref_buf.clear();
    u_opt_buf.clear();
    x_next_buf.clear();
}

void cleanup_hdf5() {
    if (!x_current_buf.empty()) {
        write_chunk();
    }
    
    if (file) {
        H5Fflush(file->getId(), H5F_SCOPE_GLOBAL);
        delete ds_xnext;
        delete ds_uopt;
        delete ds_xref;
        delete ds_xcurr;
        delete file;
    }
}

void initialize_hdf5() {
    try {
        struct stat buffer;
        bool file_exists = (stat(HDF_PATH.c_str(), &buffer) == 0);

        if (file_exists) {
            file = new H5File(HDF_PATH, H5F_ACC_RDWR);
            ds_xcurr = new DataSet(file->openDataSet("x_current"));
            ds_xref = new DataSet(file->openDataSet("x_ref"));
            ds_uopt = new DataSet(file->openDataSet("u_opt"));
            ds_xnext = new DataSet(file->openDataSet("x_next"));
            
            DataSpace space = ds_xcurr->getSpace();
            space.getSimpleExtentDims(current_size);
        } else {
            file = new H5File(HDF_PATH, H5F_ACC_TRUNC);
            
            // Create extendable datasets
            hsize_t init_dims[2] = {0, 12};
            hsize_t max_dims[2] = {H5S_UNLIMITED, 12};
            DataSpace x_space(2, init_dims, max_dims);
            
            hsize_t ref_init[2] = {0, 12*(N+1)};
            hsize_t ref_max[2] = {H5S_UNLIMITED, 12*(N+1)};
            DataSpace ref_space(2, ref_init, ref_max);
            
            hsize_t u_init[2] = {0, 8};
            hsize_t u_max[2] = {H5S_UNLIMITED, 8};
            DataSpace u_space(2, u_init, u_max);

            // Set chunk properties
            DSetCreatPropList props;
            props.setChunk(2, STATE_CHUNK);
            props.setDeflate(6);
            
            DSetCreatPropList ref_props;
            ref_props.setChunk(2, REF_CHUNK);
            ref_props.setDeflate(6);
            
            DSetCreatPropList u_props;
            u_props.setChunk(2, U_CHUNK);
            u_props.setDeflate(6);

            // Create datasets
            ds_xcurr = new DataSet(file->createDataSet("x_current", PredType::NATIVE_DOUBLE, x_space, props));
            ds_xref = new DataSet(file->createDataSet("x_ref", PredType::NATIVE_DOUBLE, ref_space, ref_props));
            ds_uopt = new DataSet(file->createDataSet("u_opt", PredType::NATIVE_DOUBLE, u_space, u_props));
            ds_xnext = new DataSet(file->createDataSet("x_next", PredType::NATIVE_DOUBLE, x_space, props));
        }
    } catch (const Exception& e) {
        std::cerr << "HDF5 Error: " << e.getCDetailMsg() << "\n";
        exit(1);
    }
}

// Visualization functions
void draw_vehicle(float x, float y, float yaw) {
    glPushMatrix();
    glTranslatef(x, y, 0.0f);
    glRotatef(yaw * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
    
    // Draw vehicle body
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glBegin(GL_TRIANGLES);
    glVertex2f(0.5f, 0.0f);
    glVertex2f(-0.3f, 0.3f);
    glVertex2f(-0.3f, -0.3f);
    glEnd();
    
    // Draw heading indicator
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow
    glBegin(GL_LINES);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(0.5f, 0.0f);
    glEnd();
    
    glPopMatrix();
}

void draw_path(const std::vector<float2>& points) {
    if (points.empty()) return;
    
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pt : points) {
        glVertex2f(pt.x, pt.y);
    }
    glEnd();
}

void draw_goal(float x, float y) {
    glColor3f(1.0f, 0.0f, 1.0f); // Magenta
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glVertex2f(x, y);
    glEnd();
}

void draw_obstacles(const std::vector<float2>& obstacles) {
    if (obstacles.empty()) return;
    
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (const auto& obs : obstacles) {
        glVertex2f(obs.x, obs.y);
    }
    glEnd();
}

void render_visualization() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Lock mutex to safely access visualization data
    std::lock_guard<std::mutex> lock(vis_mutex);
    
    // Set up coordinate system
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float view_size = 25.0f;
    glOrtho(-view_size, view_size, -view_size, view_size, -1.0f, 1.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Draw coordinate grid
    glColor3f(0.3f, 0.3f, 0.3f); // Dark gray
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (float i = -25.0f; i <= 25.0f; i += 5.0f) {
        // Vertical lines
        glVertex2f(i, -25.0f);
        glVertex2f(i, 25.0f);
        
        // Horizontal lines
        glVertex2f(-25.0f, i);
        glVertex2f(25.0f, i);
    }
    glEnd();
    
    // Draw elements
    draw_obstacles(vis_data.obstacles);
    draw_path(vis_data.path_points);
    draw_goal(vis_data.goal_pos.x, vis_data.goal_pos.y);
    draw_vehicle(vis_data.vehicle_pos.x, vis_data.vehicle_pos.y, vis_data.vehicle_yaw);
    
    glfwSwapBuffers(window);
}

void visualization_thread_func() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return;
    }
    
    window = glfwCreateWindow(VIS_WIDTH, VIS_HEIGHT, "MPC-A* Path Planning", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window\n";
        return;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    
    // Set up OpenGL
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // Dark background
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    
    // Main visualization loop
    while (!shutdown_requested && !glfwWindowShouldClose(window)) {
        render_visualization();
        glfwPollEvents();
    }
    
    glfwDestroyWindow(window);
    glfwTerminate();
    window = nullptr;
}

// Helper to convert int2 to float2 with sliding map correction
float2 int2_to_float2(int2 pt, const EnvironmentMap& map) {
    float2 world;
    // Calculate world position considering map sliding
    world.x = map.world_position_.x + (pt.x - map.shift_total_.x) * map.r_m_;
    world.y = map.world_position_.y + (pt.y - map.shift_total_.y) * map.r_m_;
    return world;
}

// Modified visualization update to handle sliding
void update_visualization_data(const std::vector<float2>& new_obstacles,
                              const EnvironmentMap& map,
                              const DM& x0, 
                              const Path& path,
                              const float2& goal_pos) {
    std::lock_guard<std::mutex> lock(vis_mutex);
    
    // Update obstacles (only add new ones)
    for (const auto& obs : new_obstacles) {
        // Check if obstacle already exists
        bool exists = false;
        for (const auto& existing : vis_data.obstacles) {
            if (fabs(existing.x - obs.x) < 0.1f && fabs(existing.y - obs.y) < 0.1f) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            vis_data.obstacles.push_back(obs);
        }
    }
    
    // Update vehicle position
    vis_data.vehicle_pos.x = static_cast<double>(x0(0));
    vis_data.vehicle_pos.y = static_cast<double>(x0(1));
    vis_data.vehicle_yaw = static_cast<double>(x0(5));
    
    // Update goal position
    vis_data.goal_pos = goal_pos;
    
    // Convert path to world coordinates with sliding correction
    vis_data.path_points.clear();
    for (int k = 0; k < path.length; k++) {
        vis_data.path_points.push_back(int2_to_float2(path.points[k], map));
    }
}

int main() {
    std::signal(SIGINT, sigint_handler);
    try {
        // Start visualization thread
        std::thread visualization_thread(visualization_thread_func);
        
        // Wait for the visualization window to initialize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        initialize_hdf5();
        VehicleModel model("config.json");
        NonlinearMPC mpc("config.json", N);
        mpc.initialization();

        for (int scenario = 0; scenario < MAX_SCENARIOS && !shutdown_requested; ++scenario) {
            // Create environment with random obstacles
            EnvironmentMap map(WIDTH, HEIGHT);
            int num_obstacles = MIN_OBSTACLES + rand() % (MAX_OBSTACLES - MIN_OBSTACLES);
            
            // Track obstacles for visualization
            std::vector<float2> scenario_obstacles;
            
            // Add obstacles to the map
            for (int i = 0; i < num_obstacles; ++i) {
                float x = rand_uniform(-20.0f, 20.0f);
                float y = rand_uniform(-20.0f, 20.0f);
                map.updateSinglePoint(x, y, 255.0f);
                scenario_obstacles.push_back({x, y});
            }

            // Set random reference point
            float ref_x = rand_uniform(-15.0f, 15.0f);
            float ref_y = rand_uniform(-15.0f, 15.0f);
            map.updateSinglePoint(ref_x, ref_y, 240.0f);
            map.set_ref(ref_x, ref_y);
            float2 goal_pos = {ref_x, ref_y};

            // Generate random initial state
            DM x0 = generate_X_current();
            double start_x = static_cast<double>(x0(0));
            double start_y = static_cast<double>(x0(1));
            
            // Update visualization with initial state
            update_visualization_data(scenario_obstacles, map, x0, Path{nullptr, 0}, goal_pos);

            Path path;

            for (int step = 0; step < MAX_STEPS_PER_SCENARIO && !shutdown_requested; ++step) {
                double eta1 = static_cast<double>(x0(0));
                double eta2 = static_cast<double>(x0(1));
                
                // Update map with current position
                map.updateSinglePoint(eta1, eta2, 200.0f);
                map.slide(eta1, eta2);
                map.set_ref(ref_x, ref_y);

                // Generate path with A*
                path = map.findPath();
                if (path.length == 0) {
                    std::cerr << "No path found!\n";
                    break;
                }

                // Update visualization with current map state
                update_visualization_data({}, map, x0, path, goal_pos);
                
                // Convert path to world coordinates for MPC reference
                std::vector<float2> path_points;
                for (int k = 0; k < path.length; k++) {
                    path_points.push_back(int2_to_float2(path.points[k], map));
                }

                // Create reference trajectory for MPC
                int m = std::min(path.length, N+1);
                DM x_ref = DM::zeros(12, N+1);
                double current_depth = static_cast<double>(x0(2));
                
                for (int k = 0; k <= N; k++) {
                    float2 world_coor = (k < m) ? path_points[k] : make_float2(ref_x, ref_y);
                    
                    // Calculate yaw angle
                    double angle;
                    if (k < m - 1) {
                        float2 next_coor = path_points[k+1];
                        angle = atan2(next_coor.y - world_coor.y, next_coor.x - world_coor.x);
                    } else if (k == m - 1 && m >= 2) {
                        float2 prev_coor = path_points[k-1];
                        angle = atan2(world_coor.y - prev_coor.y, world_coor.x - prev_coor.x);
                    } else {
                        angle = atan2(ref_y - eta2, ref_x - eta1);
                    }
                    
                    // Set reference state
                    x_ref(0, k) = world_coor.x;
                    x_ref(1, k) = world_coor.y;
                    x_ref(2, k) = current_depth;
                    x_ref(5, k) = angle;
                    if (step < 2) map.updateSinglePoint(world_coor.x, world_coor.y, 49.0f);
                }

                // Solve MPC
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                DM x_next = x_opt(Slice(), 1);

                if (step >= MAX_STEPS_PER_SCENARIO-1 || step%55 == 0 || step < 1) map.save(std::to_string(step));

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

                // Store data
                auto x0_vec = dm_to_vector(x0);
                auto x_ref_vec = dm_to_vector(x_ref);
                auto u_opt_vec = dm_to_vector(u_opt(Slice(), 0));
                auto x_next_vec = dm_to_vector(x_next);
                
                x_current_buf.insert(x_current_buf.end(), x0_vec.begin(), x0_vec.end());
                x_ref_buf.insert(x_ref_buf.end(), x_ref_vec.begin(), x_ref_vec.end());
                u_opt_buf.insert(u_opt_buf.end(), u_opt_vec.begin(), u_opt_vec.end());
                x_next_buf.insert(x_next_buf.end(), x_next_vec.begin(), x_next_vec.end());

                // Write data in chunks
                if (x_current_buf.size() >= CHUNK_SIZE * 12) {
                    write_chunk();
                }

                // Update state for next iteration
                x0 = x_next;

                // Check if goal is reached
                double dist_to_goal = sqrt(pow(static_cast<double>(x0(0)) - ref_x, 2) + 
                                     pow(static_cast<double>(x0(1)) - ref_y, 2));
                if (dist_to_goal < COLLISION_THRESHOLD) {
                    std::cout << "Goal reached in " << step << " steps!\n";
                    // break;
                }
                
                // Slow down for visualization
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // Free path resources
            if (path.points) {
                free(path.points);
            }
            
            // Clear obstacles for next scenario
            {
                std::lock_guard<std::mutex> lock(vis_mutex);
                vis_data.obstacles.clear();
            }
            
            std::cout << "Completed scenario " << scenario << "/" << MAX_SCENARIOS << "\n";
        }
        
        cleanup_hdf5();
        shutdown_requested = true;
        if (visualization_thread.joinable()) {
            visualization_thread.join();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        cleanup_hdf5();
        shutdown_requested = true;
        return 1;
    }
    return 0;
}
// conda activate mppi
// cd /home/eren/GitHub/Simulation/Model
// rm -f *.o *.so jit_* libdynamics_func* *.bin *.csv vd_test *.h5
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c environment_helper.cpp -o environment_helper.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" vd_test.cpp -o vd_test.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o vd_test environment_helper.o environment_map.o environment_astar.o environment_global.o vd_test.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -lhdf5_cpp -lhdf5 -lglfw -lGL -lGLU -L/usr/local/cuda/lib64
// ./vd_test

// conda activate mp_test
// cd /home/eren/GitHub/ControlSystem/Test/Model
// rm -rf build
// mkdir build
// cd build
// cmake -DBOOST_ROOT="$CONDA_PREFIX" \
//     -DCMAKE_CXX_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++" \
//     -DCMAKE_CUDA_HOST_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc" \
//     -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
//     -DCMAKE_BUILD_TYPE=Debug ..
// make -j4
// ./test

#include "environment.h"
#include "nlmpc.h"
#include "vehicle_model.h"

#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <H5Cpp.h>
#include <random>
#include <atomic>
#include <sys/stat.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <algorithm>
#include <getopt.h>
#include <unistd.h>


#include "data.hpp"
#include "helpers.hpp"
#include "visualization.hpp"

const int MAX_OBSTACLES = 20;
const int MIN_OBSTACLES = 5;
bool do_reset = false;
int MAX_STEPS = 100;
int MAX_SCENARIOS = 1000;
const int WIDTH = 129;
const int HEIGHT = 129;
float ref_x = 20.0f;
float ref_y = 10.0f;
const float OBSTACLE_THRESHOLD = 250.0f;
const int CHUNK_SIZE = 100;

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  -r, --reset               Delete .h5 file before starting\n"
              << "  -s, --step <N>            Set max steps (default 100)\n"
              << "  -p, --plan <M>            Set max scenarios (default 1000)\n"
              << "  -h, --help                Show this help\n";
}

int main(int argc, char** argv) {
    const char* short_opts = "rs:p:h";
    const struct option long_opts[] = {
        {"reset", no_argument,        nullptr, 'r'},
        {"step",  required_argument,  nullptr, 's'},
        {"plan",  required_argument,  nullptr, 'p'},
        {"help",  no_argument,        nullptr, 'h'},
        {nullptr, 0,                  nullptr,  0 }
    };

    int opt;
    while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1) {
        switch (opt) {
            case 'r':
                do_reset = true;
                break;
            case 's':
                MAX_STEPS = std::atoi(optarg);
                if (MAX_STEPS < 0) {
                    std::cerr << "Invalid --step value\n";
                    return EXIT_FAILURE;
                }
                break;
            case 'p':
                MAX_SCENARIOS = std::atoi(optarg);
                if (MAX_SCENARIOS < 0) {
                    std::cerr << "Invalid --plan value\n";
                    return EXIT_FAILURE;
                }
                break;
            case 'h':
            case '?':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? EXIT_SUCCESS : EXIT_FAILURE;
        }
    }

    std::cout << "reset = " << do_reset
              << ", max_steps = " << MAX_STEPS
              << ", max_scenarios = " << MAX_SCENARIOS << "\n";

    // If --reset, remove the file
    if (do_reset) {
        const char* filename = "data.h5";
        if (unlink(filename) == 0)
            std::cout << "Removed " << filename << "\n";
        else if (errno != ENOENT)
            perror("Error deleting .h5 file");
    }

    std::signal(SIGINT, sigint_handler);
    try {
        const int WIDTH = 129;
        const int HEIGHT = 129;
        const int N = 40;
        float ref_x = 20.0f;
        float ref_y = 10.0f;
        EnvironmentMap map(WIDTH, HEIGHT, N);
        VehicleModel model("../config.json"); 
        NonlinearMPC mpc("../config.json", N);
        mpc.initialization();
        initialize_hdf5();
        std::thread visualization_thread(visualization_thread_func);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        SAFETY_DIST = map.obstacle_radius_ * map.r_m_;

        DM x0 = DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
        DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);

        for (int scenario = 0; scenario < MAX_SCENARIOS && !shutdown_requested; ++scenario) {
            map.resetAll();
            mpc.reset_previous_solution();

            ref_x = rand_uniform(-30.0f, 30.0f);
            ref_y = rand_uniform(-30.0f, 30.0f);
            float2 goal_pos = {ref_x, ref_y};

            int num_obstacles = MIN_OBSTACLES + rand() % (MAX_OBSTACLES - MIN_OBSTACLES);
            std::vector<float2> scenario_obstacles;

            for (int i = 0; i < num_obstacles; ) {
                float x = rand_uniform(-15.0f, 15.0f);
                float y = rand_uniform(-15.0f, 15.0f);
                
                // Skip if near start/goal
                if (hypot(x, y) < SAFETY_DIST || 
                    hypot(x - ref_x, y - ref_y) < SAFETY_DIST) {
                    continue;
                }
                map.updateSinglePoint(x, y, 255.0f);
                scenario_obstacles.push_back({x, y});
                i++;
            }

            x0 = generate_X_current();
            update_visualization_data(scenario_obstacles, map, x0, Path{}, N, goal_pos);

            for (int step = 0; step < MAX_STEPS; ++step) {
                double eta1 = static_cast<float>(static_cast<double>(x0(0)));
                double eta2 = static_cast<float>(static_cast<double>(x0(1)));
                double eta6 = static_cast<float>(static_cast<double>(x0(5)));
                
                map.slide(eta1, eta2);
                Path path = map.findPath(ref_x, ref_y);

                if (!map.is_safe()) {
                    if (path.length > 0) {
                        float2 ref = path.trajectory[0];
                        x_ref = DM::repmat(DM::vertcat({ref.x, ref.y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);
                        if (step%20 == 0) map.updateSinglePoint(ref.x, ref.y, 50.0f);                        
                    }
                } else {
                    if (path.length == 0) {
                        std::cerr << "Path not found! Using fallback.\n";
                        x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);
                    } else {
                        // Use the results directly
                        for (int k = 0; k <= N; k++) {
                            float2 position = path.trajectory[k];
                            float yaw = path.angle[k];
                            
                            // Use in controller
                            x_ref(0, k) = position.x;
                            x_ref(1, k) = position.y;
                            x_ref(5, k) = yaw;
                            if (step%20 == 0) map.updateSinglePoint(position.x, position.y, 50.0f);
                        }
                    }
                }
                    
                update_visualization_data(scenario_obstacles, map, x0, path, N, goal_pos);

                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);

                DM x_next = x_opt(Slice(), 1);
                auto state_error = x_ref(Slice(), 0) - x_next;

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

                std::cout << "Step " << step << "\n"
                            << "  Controls: " << u_opt << "\n"
                            << "  State: " << x0 << "\n"
                            << "  Reference: " << x_ref(Slice(), 0) << "\n"
                            << "  Path Length: " << path.length << "\n"
                            << "  Next State: " << x_next << "\n"
                            << "  State Error: " << state_error << "\n";

                x0 = x_next;

                if (path.points) {
                    free(path.points);
                }
                if (path.trajectory) {
                    free(path.trajectory);
                }
                if (path.angle) {
                    free(path.angle);
                }
                if (step%20 == 0) {
                    map.save(std::to_string(step));
                }
                if (step%20 == 0) {
                    map.save(std::to_string(step));
                }
            }
            {
                std::lock_guard<std::mutex> lock(vis_mutex);
                vis_data.obstacles.clear();
            }
            scenario_obstacles.clear();

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::cout << "\nCompleted scenario " << scenario << "/" << MAX_SCENARIOS << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        cleanup_hdf5();
        return 1;
    }
    write_chunk();
    cleanup_hdf5();
    return 0;
}


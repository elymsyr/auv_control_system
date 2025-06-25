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


#include "data.hpp"
#include "helpers.hpp"
#include "visualization.hpp"
#include "metrics.hpp"

int main() {
    std::signal(SIGINT, sigint_handler);
    try {
        const int WIDTH = 129;
        const int HEIGHT = 129;
        const int N = 40;
        float ref_x = 20.0f;
        float ref_y = 10.0f;
        EnvironmentMap map(WIDTH, HEIGHT, N);
        VehicleModel model("config.json"); 
        NonlinearMPC mpc("config.json", N);
        mpc.initialization();
        initialize_hdf5();
        std::thread visualization_thread(visualization_thread_func);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        SAFETY_DIST = map.obstacle_radius_ * map.r_m_;

        PerformanceMetrics metrics;
        std::vector<PerformanceMetrics> step_metrics;

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
            update_visualization_data(scenario_obstacles, map, x0, Path{}, goal_pos);

            for (int step = 0; step < MAX_STEPS; ++step) {
                metrics.reset();
                auto step_start = std::chrono::high_resolution_clock::now();
                
                double eta1 = static_cast<float>(static_cast<double>(x0(0)));
                double eta2 = static_cast<float>(static_cast<double>(x0(1)));
                double eta6 = static_cast<float>(static_cast<double>(x0(5)));
                
                // Track map update time
                auto map_start = std::chrono::high_resolution_clock::now();
                auto map_end = std::chrono::high_resolution_clock::now();
                metrics.map_update_time = std::chrono::duration<double, std::milli>(map_end - map_start).count();
                
                // Track slide time
                auto slide_start = std::chrono::high_resolution_clock::now();
                map.slide(eta1, eta2);
                auto slide_end = std::chrono::high_resolution_clock::now();
                metrics.slide_time = std::chrono::duration<double, std::milli>(slide_end - slide_start).count();
                
                // Track pathfinding time
                auto path_start = std::chrono::high_resolution_clock::now();
                Path path = map.findPath(ref_x, ref_y);
                auto path_end = std::chrono::high_resolution_clock::now();
                metrics.pathfinding_time = std::chrono::duration<double, std::milli>(path_end - path_start).count();

                // Get GPU memory usage
                metrics.gpu_memory_usage = get_gpu_memory_usage();

                // Use the results directly
                for (int k = 0; k <= N; k++) {
                    float2 position = path.trajectory[k];
                    float yaw = path.angle[k];
                    
                    // Use in controller
                    x_ref(0, k) = position.x;
                    x_ref(1, k) = position.y;
                    x_ref(5, k) = yaw;
                }

                update_visualization_data(scenario_obstacles, map, x0, path, goal_pos);

                // Track MPC time
                auto mpc_start = std::chrono::high_resolution_clock::now();
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                auto mpc_end = std::chrono::high_resolution_clock::now();
                metrics.mpc_time = std::chrono::duration<double, std::milli>(mpc_end - mpc_start).count();

                DM x_next = x_opt(Slice(), 1);
                auto state_error = x_ref(Slice(), 0) - x_next;

                // Calculate total step time
                auto step_end = std::chrono::high_resolution_clock::now();
                metrics.total_time = std::chrono::duration<double, std::milli>(step_end - step_start).count();
                
                // Store metrics for this step
                step_metrics.push_back(metrics);

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
                
                // Print performance metrics for this step
                std::cout << "  Performance: "
                        << "Map=" << metrics.map_update_time << "ms, "
                        << "Slide=" << metrics.slide_time << "ms, "
                        << "Path=" << metrics.pathfinding_time << "ms, "
                        << "MPC=" << metrics.mpc_time << "ms, "
                        << "Total=" << metrics.total_time << "ms, "
                        << "GPU=" << metrics.gpu_memory_usage / (1024 * 1024) << "MB\n";

                x0 = x_next;

                if (path.points) {
                    free(path.points);
                }
                // Clear obstacles for next scenario
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            {
                std::lock_guard<std::mutex> lock(vis_mutex);
                vis_data.obstacles.clear();
            }            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::cout << "\nCompleted scenario " << scenario << "/" << MAX_SCENARIOS << "\n";
        }
        
        // Calculate and print average metrics
        PerformanceMetrics avg;
        for (const auto& m : step_metrics) {
            avg.map_update_time += m.map_update_time;
            avg.slide_time += m.slide_time;
            avg.pathfinding_time += m.pathfinding_time;
            avg.mpc_time += m.mpc_time;
            avg.total_time += m.total_time;
            avg.gpu_memory_usage += m.gpu_memory_usage;
        }
        cleanup_hdf5();
        
        double count = step_metrics.size();
        avg.map_update_time /= count;
        avg.slide_time /= count;
        avg.pathfinding_time /= count;
        avg.mpc_time /= count;
        avg.total_time /= count;
        avg.gpu_memory_usage /= count;
        
        std::cout << "\n=== AVERAGE PERFORMANCE METRICS ===\n";
        std::cout << "Map Update:    " << avg.map_update_time << " ms\n";
        std::cout << "Slide:         " << avg.slide_time << " ms\n";
        std::cout << "Pathfinding:   " << avg.pathfinding_time << " ms\n";
        std::cout << "MPC Solve:     " << avg.mpc_time << " ms\n";
        std::cout << "Total Step:    " << avg.total_time << " ms\n";
        std::cout << "GPU Memory:    " << avg.gpu_memory_usage / (1024 * 1024) << " MB\n";
        std::cout << "===================================\n";
        
        // Save metrics to file
        std::ofstream metrics_file("performance_metrics.csv");
        metrics_file << "Step,MapUpdate,Slide,Pathfinding,MPC,Total,GPUMemory\n";
        for (int i = 0; i < step_metrics.size(); i++) {
            const auto& m = step_metrics[i];
            metrics_file << i << ","
                        << m.map_update_time << ","
                        << m.slide_time << ","
                        << m.pathfinding_time << ","
                        << m.mpc_time << ","
                        << m.total_time << ","
                        << m.gpu_memory_usage << "\n";
        }
        metrics_file.close();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        cleanup_hdf5();
        return 1;
    }
    return 0;
}


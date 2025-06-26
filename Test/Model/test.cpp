// conda activate mppi
// cd /home/eren/GitHub/ControlSystem/Test/Model
// rm -f *.o *.so jit_* libdynamics_func* *.bin *.csv test
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c environment_helper.cpp -o environment_helper.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" test.cpp -o test.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o test environment_helper.o environment_map.o environment_astar.o environment_global.o test.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -L/usr/local/cuda/lib64
// ./test
// python /home/eren/GitHub/Simulation/Model/visualize.py


// cd /home/eren/GitHub/Simulation/Model
// rm -f *.bin *.csv jit_* test
// g++ -o test environment_helper.o environment_map.o environment_astar.o environment_global.o test.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -L/usr/local/cuda/lib64
// ./test
// python /home/eren/GitHub/Simulation/Model/visualize.py

#include "environment.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include "nlmpc.h"
#include "vehicle_model.h"

#define CUDA_CALL(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                  << ": " << cudaGetErrorString(err) << "\n"; \
        exit(EXIT_FAILURE); \
    } \
}

// Performance metrics structure
struct PerformanceMetrics {
    double map_update_time = 0;
    double slide_time = 0;
    double pathfinding_time = 0;
    double mpc_time = 0;
    double total_time = 0;
    size_t gpu_memory_usage = 0;
    
    void reset() {
        map_update_time = 0;
        slide_time = 0;
        pathfinding_time = 0;
        mpc_time = 0;
        total_time = 0;
        gpu_memory_usage = 0;
    }
    
    void print() const {
        std::cout << "\n=== PERFORMANCE METRICS ===\n";
        std::cout << "Map Update:    " << map_update_time << " ms\n";
        std::cout << "Slide:         " << slide_time << " ms\n";
        std::cout << "Pathfinding:   " << pathfinding_time << " ms\n";
        std::cout << "MPC Solve:     " << mpc_time << " ms\n";
        std::cout << "Total:         " << total_time << " ms\n";
        std::cout << "GPU Memory:    " << gpu_memory_usage / (1024 * 1024) << " MB\n";
        std::cout << "===========================\n";
    }
};

// Function to get current GPU memory usage
size_t get_gpu_memory_usage() {
    size_t free, total;
    CUDA_CALL(cudaMemGetInfo(&free, &total));
    return total - free;
}

DM obstacles_to_dm(const std::vector<std::pair<float, float>>& obstacles);

int main() {
    const int WIDTH = 129;
    const int HEIGHT = 129;
    const int N = 40;
    float ref_x = 20.0f;
    float ref_y = 10.0f;
    EnvironmentMap map(WIDTH, HEIGHT, N);
    VehicleModel model("config.json"); 
    NonlinearMPC mpc("config.json", N);
    mpc.initialization();

    PerformanceMetrics metrics;
    std::vector<PerformanceMetrics> step_metrics;

    {
        std::cout << "\n--- Adding Single Points ---\n";
        auto start = std::chrono::high_resolution_clock::now();
        
        map.updateSinglePoint(-10.112f, 2.436f, 255.0f);
        map.updateSinglePoint(5.025f, 12.8908f, 255.0f);
        map.updateSinglePoint(10.0506f, 4.34f, 255.0f);
        map.updateSinglePoint(-14.0506f, -1.34f, 255.0f);
        map.updateSinglePoint(14.0506f, 9.34f, 255.0f);
        
        auto end = std::chrono::high_resolution_clock::now();
        metrics.map_update_time = std::chrono::duration<double, std::milli>(end - start).count();
    }

    DM x0 = DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);

    int max_step = 100;

    for (int step = 0; step < max_step; ++step) {
        metrics.reset();
        auto step_start = std::chrono::high_resolution_clock::now();
        
        double eta1 = static_cast<float>(static_cast<double>(x0(0)));
        double eta2 = static_cast<float>(static_cast<double>(x0(1)));
        double eta6 = static_cast<float>(static_cast<double>(x0(5)));
        
        // Track map update time
        auto map_start = std::chrono::high_resolution_clock::now();
        map.updateSinglePoint(eta1, eta2, 190.0f);
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
            if (step%20 == 0) map.updateSinglePoint(position.x, position.y, 50.0f);
        }

        // Track MPC time
        auto mpc_start = std::chrono::high_resolution_clock::now();
        auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
        auto mpc_end = std::chrono::high_resolution_clock::now();
        metrics.mpc_time = std::chrono::duration<double, std::milli>(mpc_end - mpc_start).count();

        auto state_error = x_ref(Slice(), 0) - x_opt(Slice(), 0);

        // Calculate total step time
        auto step_end = std::chrono::high_resolution_clock::now();
        metrics.total_time = std::chrono::duration<double, std::milli>(step_end - step_start).count();
        
        // Store metrics for this step
        step_metrics.push_back(metrics);

        std::cout << "Step " << step << "\n"
                    << "  Controls: " << u_opt << "\n"
                    << "  State: " << x0 << "\n"
                    << "  Reference: " << x_ref(Slice(), 0) << "\n"
                    << "  Path Length: " << path.length << "\n"
                    << "  Next State: " << x_opt(Slice(), 1) << "\n"
                    << "  State Error: " << state_error << "\n";
        
        // Print performance metrics for this step
        std::cout << "  Performance: "
                  << "Map=" << metrics.map_update_time << "ms, "
                  << "Slide=" << metrics.slide_time << "ms, "
                  << "Path=" << metrics.pathfinding_time << "ms, "
                  << "MPC=" << metrics.mpc_time << "ms, "
                  << "Total=" << metrics.total_time << "ms, "
                  << "GPU=" << metrics.gpu_memory_usage / (1024 * 1024) << "MB\n";

        x0 = x_opt(Slice(), 1);

        // Save current state and reference
        if (step == max_step-1 || step%20 == 0) {
            map.save(std::to_string(step));
        }
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
    
    return 0;
}

DM obstacles_to_dm(const std::vector<std::pair<float, float>>& obstacles) {
    casadi::DM obs_dm = casadi::DM::zeros(2, obstacles.size());
    for (int i = 0; i < obstacles.size(); ++i) {
        obs_dm(0, i) = obstacles[i].first;
        obs_dm(1, i) = obstacles[i].second;
    }
    return obs_dm;
}
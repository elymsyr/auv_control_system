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

DM obstacles_to_dm(const std::vector<std::pair<float, float>>& obstacles);

int main() {
    std::ofstream state_file("state_history.txt");
    std::ofstream ref_file("reference_history.txt");

    // Create environment map
    const int WIDTH = 129;
    const int HEIGHT = 129;
    const int N = 20;
    float ref_x = 39.0f;
    float ref_y = 10.0f;
    EnvironmentMap map(WIDTH, HEIGHT);
    VehicleModel model("config.json"); 
    NonlinearMPC mpc(model, N);

    {
        std::cout << "\n--- Adding Single Points ---\n";

        map.updateSinglePoint(2.223f, 1.213f, 255.0f);
        map.updateSinglePoint(-6.125f, -2.436f, 255.0f);
        map.updateSinglePoint(-3.112f, -2.436f, 255.0f);
        map.updateSinglePoint(2.8659f, 1.326f, 255.0f);
        map.updateSinglePoint(2.12f, 2.437f, 255.0f);
        map.updateSinglePoint(3.12f, 1.12f, 255.0f);
        map.updateSinglePoint(4.12f, 1.42f, 255.0f);
        map.updateSinglePoint(-3.2156f, 2.13459f, 255.0f);
        map.updateSinglePoint(5.025f, 1.8908f, 255.0f);
        map.updateSinglePoint(10.0506f, 4.34f, 255.0f);
        map.updateSinglePoint(13.0506f, 12.34f, 255.0f);
        map.updateSinglePoint(14.0506f, 1.34f, 255.0f);
        map.updateSinglePoint(14.0506f, 9.34f, 255.0f);
        map.updateSinglePoint(ref_x, ref_y, 249.0f);
    }

    DM x0 = DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);
    map.set_ref(ref_x, ref_y);
    map.set_velocity(0.0f, 0.0f);

    int max_step = 200;

    for (int step = 0; step < max_step; ++step) {
        double eta1 = static_cast<float>(static_cast<double>(x0(0)));
        double eta2 = static_cast<float>(static_cast<double>(x0(1)));
        double eta6 = static_cast<float>(static_cast<double>(x0(5)));
        map.updateSinglePoint(eta1, eta2, 200.0f);
        map.slide(eta1, eta2);
        
        map.set_ref(ref_x, ref_y);
        Path path = map.findPath();
        int m = std::min(path.length, N+1);
        float spacing = static_cast<float>(std::min((m-1) / (N), 10));

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
            if (step < 2) map.updateSinglePoint(world_coor.x, world_coor.y, 49.0f);
        }

        state_file << static_cast<double>(x0(0)) << " "    // ref_x
                   << static_cast<double>(x0(1)) << " "    // ref_y
                   << static_cast<double>(x0(2)) << " "    // ref_z
                   << static_cast<double>(x0(3)) << " "    // ref_roll
                   << static_cast<double>(x0(4)) << " "    // ref_pitch
                   << static_cast<double>(x0(5)) << "\n";  // ref_yaw
        
        ref_file << static_cast<double>(x_ref(0, 0)) << " "    // ref_x
                 << static_cast<double>(x_ref(1, 0)) << " "    // ref_y
                 << static_cast<double>(x_ref(2, 0)) << " "    // ref_z
                 << static_cast<double>(x_ref(3, 0)) << " "    // ref_roll
                 << static_cast<double>(x_ref(4, 0)) << " "    // ref_pitch
                 << static_cast<double>(x_ref(5, 0)) << "\n";  // ref_yaw

        auto [u_opt, x_opt] = mpc.solve(x0, x_ref);

        x0 = x_opt(Slice(), 1);
        auto state_error = x_ref(Slice(), 0) - x0;

        std::cout << "Step " << step << "\n"
                    << "  Controls: " << u_opt << "\n"
                    << "  State: " << x0 << "\n"
                    << "  Reference: " << x_ref(Slice(), 0) << "\n"
                    << "  Path Lenth: " << path.length << "\n"
                    << "  State Error: " << state_error << "\n";

        // Save current state and reference
        if (step >= max_step-1 || step%55 == 0 || step < 1) map.save(std::to_string(step));
    }
    state_file.close();
    ref_file.close();
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

// cd /home/eren/GitHub/ControlSystem/environment/map-entegrated-mpc
// rm -f *.o *.so jit_* libdynamics_func* *.bin *.txt
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c environment_helper.cpp -o environment_helper.o
// nvcc -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" main.cpp -o main.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o main environment_helper.o environment_map.o environment_astar.o environment_global.o main.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -L/usr/local/cuda/lib64
// ./main
// python /home/eren/GitHub/ControlSystem/environment/map-entegrated-mpc/visualize.py
// python render.py
// rm -f *.o *.so jit_* libdynamics_func* *.bin
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

// DM create_reference_from_path(const DM& current_state, const std::vector<std::pair<float, float>>& path, int horizon);
// DM pathToReference(const DM& current_state, Path path, const EnvironmentMap& map, int horizon);
DM obstacles_to_dm(const std::vector<std::pair<float, float>>& obstacles);

int main() {
    // Create environment map
    const int WIDTH = 129;
    const int HEIGHT = 129;
    const int N = 20;
    float ref_x = -10.0f;
    float ref_y = -10.0f;
    EnvironmentMap map(WIDTH, HEIGHT);
    VehicleModel model("config.json"); 
    NonlinearMPC mpc(model, N);

    // std::cout << "After construction:\n";
    // // map.save("initial_empty.bin");

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
    }

    DM x0 = DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);
    map.set_ref(ref_x, ref_y);
    map.set_velocity(0.0f, 0.0f);

    int max_step = 5;

    for (int step = 0; step < max_step; ++step) {
        double eta1 = static_cast<double>(x0(0));
        double eta2 = static_cast<double>(x0(1));
        map.updateSinglePoint(eta1, eta2, 155.0f);
        map.slide(static_cast<float>(eta1), static_cast<float>(eta2));

        std::vector<std::pair<float, float>> obstacles = map.obstacle_selection(mpc.num_obstacles_);

        DM obs_dm = obstacles_to_dm(obstacles);

        auto [u_opt, x_opt] = mpc.solve(x0, x_ref, obs_dm);

        x0 = x_opt(Slice(), 1);
        auto state_error = x_ref(Slice(), 0) - x0;

        std::cout << "Step " << step << "\n"
                    << "  controls: " << u_opt << "\n"
                    << "  state: " << x0 << "\n"
                    << "  state error: " << state_error << "\n";
        if (step >= max_step-3) map.save(std::to_string(step));
    }

    // {
    //     std::cout << "\n--- Map Sliding Test ---\n";
        
    //     map.slide(322.0f, 0.0f);
    //     std::cout << "Map slid by (122.0, -47.0)\n";
    //     map.save("initial_shifted.bin");
        
    //     auto obstacles = map.obstacle_selection(5);
    //     std::cout << "Obstacles after sliding:\n";
    //     for (size_t i = 0; i < obstacles.size(); i++) {
    //         std::cout << "  Obstacle " << i+1 << ": (" 
    //                   << obstacles[i].first << ", " << obstacles[i].second << ")\n";
    //     }
    // }

    // {
    //     std::cout << "\n--- Point Batch Test ---\n";
    //     PointBatch* batch = EnvironmentMap::createPointBatch(10000);
    //     EnvironmentMap::fillPointBatchWithRandom(batch, WIDTH, HEIGHT);
    //     map.updateWithBatch(batch);
    //     EnvironmentMap::destroyPointBatch(batch);
    //     map.save("initial_filled.bin");
    // }

    // {
    //     map.set_velocity(-50.0f, -50.0f);
    //     map.set_x_ref(250.0f, 350.0f);

    //     auto obstacles = map.obstacle_selection();
    //     std::cout << "Obstacles after sliding:\n";
    //     for (size_t i = 0; i < obstacles.size(); i++) {
    //         map.updateSinglePoint(obstacles[i].first, obstacles[i].second, 200);
    //     }
    //     map.updateSinglePoint(250.0f, 350.0f, 255);
    //     map.updateSinglePoint(0.0f, 0.0f, 250);
    //     map.updateSinglePoint(-100.0f, -100.0f, 255);
    //     map.save("initial_signed.bin");
    // }
    return 0;
}

// DM create_reference_from_path(const DM& current_state, 
//                             const std::vector<std::pair<float, float>>& path,
//                             int horizon) {
//     DM x_ref = DM::zeros(12, horizon+1);
//     if (path.empty()) return x_ref;
    
//     double current_x = static_cast<double>(current_state(0));
//     double current_y = static_cast<double>(current_state(1));
    
//     // Find closest point on path
//     int closest_idx = 0;
//     double min_dist = std::numeric_limits<double>::max();
//     for (int i = 0; i < path.size(); ++i) {
//         double dx = current_x - path[i].first;
//         double dy = current_y - path[i].second;
//         double dist = dx*dx + dy*dy;
//         if (dist < min_dist) {
//             min_dist = dist;
//             closest_idx = i;
//         }
//     }
    
//     // Create reference trajectory
//     int points_to_use = std::min(static_cast<int>(path.size()) - closest_idx, horizon+1);
//     for (int i = 0; i < points_to_use; ++i) {
//         int path_idx = closest_idx + i;
//         x_ref(0, i) = path[path_idx].first;
//         x_ref(1, i) = path[path_idx].second;
//     }
    
//     // Extend with last point if needed
//     if (points_to_use < horizon+1) {
//         for (int i = points_to_use; i <= horizon; ++i) {
//             x_ref(0, i) = path.back().first;
//             x_ref(1, i) = path.back().second;
//         }
//     }
    
//     return x_ref;
// }

// DM pathToReference(const DM& current_state, Path path, 
//                    const EnvironmentMap& map, int horizon) {
//     if (path.length == 0) {
//         // Return default reference if no path
//         std::cout << "No path found, returning default reference.\n";
//         return DM::repmat(DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, horizon+1);
//     }

//     // Copy path to host
//     std::vector<int2> h_path(path.length);
//     CUDA_CALL(cudaMemcpy(h_path.data(), path.points, 
//                          path.length * sizeof(int2), cudaMemcpyDeviceToHost));

//     // Convert to global coordinates
//     std::vector<std::pair<float, float>> global_path;
//     for (int i = 0; i < path.length; i++) {
//         float x_global, y_global;
//         map.gridToGlobal(h_path[i].x, h_path[i].y, x_global, y_global);
//         global_path.push_back({x_global, y_global});
//     }

//     // Create reference trajectory
//     return create_reference_from_path(current_state, global_path, horizon);
// }

// Convert obstacles to DM matrix
DM obstacles_to_dm(const std::vector<std::pair<float, float>>& obstacles) {
    DM obs_dm = DM::zeros(2, obstacles.size());
    for (int i = 0; i < obstacles.size(); ++i) {
        obs_dm(0, i) = obstacles[i].first;
        obs_dm(1, i) = obstacles[i].second;
    }
    return obs_dm;
}

// cd /home/eren/GitHub/ControlSystem/environment/map-entegrated-mpc
// rm -f *.o *.so jit_* libdynamics_func* *.bin
// nvcc -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" main.cpp -o main.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o main environment_map.o environment_astar.o environment_global.o main.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -L/usr/local/cuda/lib64
// ./main
// rm -f *.o *.so jit_* libdynamics_func*
// python /home/eren/GitHub/ControlSystem/environment/map-entegrated-mpc/visualize.py
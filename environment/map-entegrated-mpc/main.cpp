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
    std::ofstream path_file("path_history.txt");

    // Create environment map
    const int WIDTH = 129;
    const int HEIGHT = 129;
    const int N = 20;
    float ref_x = -14.0f;
    float ref_y = -12.0f;
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
        map.updateSinglePoint(ref_x, ref_y, 249.0f);
    }

    DM x0 = DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);
    map.set_ref(ref_x, ref_y);
    map.set_velocity(0.0f, 0.0f);

    int max_step = 100;

    for (int step = 0; step < max_step; ++step) {
        double eta1 = static_cast<float>(static_cast<double>(x0(0)));
        double eta2 = static_cast<float>(static_cast<double>(x0(1)));
        double eta6 = static_cast<float>(static_cast<double>(x0(5)));
        map.updateSinglePoint(eta1, eta2, 200.0f);
        map.slide(eta1, eta2);

        std::vector<std::pair<float, float>> obstacles = map.obstacle_selection(mpc.num_obstacles_);
        Path path = map.findPath();
        int m = std::min(path.length, N+1);
        float spacing = static_cast<float>(std::min((m-1) / (N), 4));

        state_file << static_cast<double>(x0(0)) << " "    // ref_x
                   << static_cast<double>(x0(1)) << " "    // ref_y
                   << static_cast<double>(x0(2)) << " "    // ref_z
                   << static_cast<double>(x0(3)) << " "    // ref_roll
                   << static_cast<double>(x0(4)) << " "    // ref_pitch
                   << static_cast<double>(x0(5)) << "\n";  // ref_yaw
        
        ref_file << static_cast<double>(x_ref(0, spacing)) << " "    // ref_x
                 << static_cast<double>(x_ref(1, spacing)) << " "    // ref_y
                 << static_cast<double>(x_ref(2, spacing)) << " "    // ref_z
                 << static_cast<double>(x_ref(2, spacing)) << " "    // ref_roll
                 << static_cast<double>(x_ref(2, spacing)) << " "    // ref_pitch
                 << static_cast<double>(x_ref(2, spacing)) << "\n";  // ref_yaw
        
        // Save path points
        if (path.length > 0) {
            for (int i = 0; i < path.length; i++) {
                float grid_x = path.points[i].x;
                float grid_y = path.points[i].y;
                float world_x = (grid_x - map.width_/2.0f) * map.r_m_ + map.world_position_.x;
                float world_y = (grid_y - map.height_/2.0f) * map.r_m_ + map.world_position_.y;
                path_file << world_x << " " << world_y << " ";
            }
        }
        path_file << "\n";

        float angle = angleBetweenPoints(eta1, eta2, ref_x, ref_y);
        if (path.length > 0) {
            for (int k = 0; k <= N; k++) {
                float2 world_coor = createPath(m, k, spacing, map, path);
                // Set reference
                x_ref(0, k) = world_coor.x;
                x_ref(1, k) = world_coor.y;
                if (k+spacing <= path.length) {
                    float2 next_coor = createPath(m, k+spacing, spacing, map, path);
                    angle = angleBetweenPoints(world_coor.x, world_coor.y, next_coor.x, next_coor.y);
                }
                x_ref(2, k) = angle;
                if (step >= max_step-1 || step < 1) map.updateSinglePoint(world_coor.x, world_coor.y, 50.0f);
            }
        } else {
            // Fallback: use original fixed reference
            for (int k = 0; k <= N; k++) {
                x_ref(0, k) = ref_x;
                x_ref(1, k) = ref_y;
                x_ref(2, k) = angle;
            }
            std::cerr << "Warning: No path found, using fixed reference\n";
        }

        map.set_ref(ref_x, ref_y);
        DM obs_dm = obstacles_to_dm(obstacles);

        auto [u_opt, x_opt] = mpc.solve(x0, x_ref, obs_dm);

        x0 = x_opt(Slice(), 1);
        auto state_error = x_ref(Slice(), 0) - x0;

        std::cout << "Step " << step << "\n"
                    << "  controls: " << u_opt << "\n"
                    << "  state: " << x0 << "\n"
                    << "  state error: " << state_error << "\n";
        
        // Save current state and reference
        if (step >= max_step-1 || step < 1) map.save(std::to_string(step));
    }
    state_file.close();
    ref_file.close();
    path_file.close();
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
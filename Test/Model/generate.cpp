// conda activate mppi
// cd /home/eren/GitHub/Simulation/Model
// rm -f *.o *.so jit_* libdynamics_func* *.bin *.csv generate
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c environment_helper.cpp -o environment_helper.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_map.cu -o environment_map.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_astar.cu -o environment_astar.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -c environment_global.cu -o environment_global.o
// nvcc -allow-unsupported-compiler -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" generate.cpp -o generate.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c nlmpc.cpp -o nlmpc.o
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c vehicle_model.cpp -o vehicle_model.o
// g++ -o generate environment_helper.o environment_map.o environment_astar.o environment_global.o generate.o vehicle_model.o nlmpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -lhdf5_cpp -lhdf5 -lglfw -lGL -lGLU -L/usr/local/cuda/lib64
// ./generate
// python /home/eren/GitHub/Simulation/Model/visualize.py

#include "environment.h"
#include "nlmpc.h"
#include "vehicle_model.h"

#include <iostream>
#include <vector>
#include <cstdlib>
#include <H5Cpp.h>
#include <random>
#include <atomic>
#include <sys/stat.h>
#include <algorithm>

#include "data.hpp"
#include "helpers.hpp"

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

    const float SAFETY_DIST = map.obstacle_radius_ * map.r_m_;

    DM x0 = DM::vertcat({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    DM x_ref = DM::repmat(DM::vertcat({ref_x, ref_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, N+1);

    for (int scenario = 0; scenario < MAX_SCENARIOS && !shutdown_requested; ++scenario) {
        map.resetAll();
        mpc.reset_previous_solution();

        ref_x = rand_uniform(-30.0f, 30.0f);
        ref_y = rand_uniform(-30.0f, 30.0f);

        int num_obstacles = MIN_OBSTACLES + rand() % (MAX_OBSTACLES - MIN_OBSTACLES);

        for (int i = 0; i < num_obstacles; ) {
            float x = rand_uniform(-15.0f, 15.0f);
            float y = rand_uniform(-15.0f, 15.0f);
            
            // Skip if near start/goal
            if (hypot(x, y) < SAFETY_DIST || 
                hypot(x - ref_x, y - ref_y) < SAFETY_DIST) {
                continue;
            }
            map.updateSinglePoint(x, y, 255.0f);
            i++;
        }

        x0 = generate_X_current();

        for (int step = 0; step < MAX_STEPS; ++step) {
            double eta1 = static_cast<float>(static_cast<double>(x0(0)));
            double eta2 = static_cast<float>(static_cast<double>(x0(1)));
            double eta6 = static_cast<float>(static_cast<double>(x0(5)));
            
            map.updateSinglePoint(eta1, eta2, 190.0f);
            map.slide(eta1, eta2);
            
            Path path = map.findPath(ref_x, ref_y);

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

            auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
            auto state_error = x_ref(Slice(), 0) - x_opt(Slice(), 0);

            std::cout << "Step " << step << "\n"
                        << "  Controls: " << u_opt << "\n"
                        << "  State: " << x0 << "\n"
                        << "  Reference: " << x_ref(Slice(), 0) << "\n"
                        << "  Path Length: " << path.length << "\n"
                        << "  Next State: " << x_opt(Slice(), 1) << "\n"
                        << "  State Error: " << state_error << "\n";

            x0 = x_opt(Slice(), 1);

            // Save current state and reference
            if (step == MAX_STEPS-1 || step%20 == 0) {
                map.save(std::to_string(step));
            }
            if (path.points) {
                free(path.points);
            }
        }
        std::cout << "\nCompleted scenario " << scenario << "/" << MAX_SCENARIOS << "\n";
    }
    return 0;
}


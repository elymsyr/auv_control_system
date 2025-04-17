#ifndef ENVIRONMENT_STATE_H
#define ENVIRONMENT_STATE_H

#include <vector>
#include <mutex>
#include <string>
#include "sensor_data.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <cstdint>

class EnvironmentState {
public:

    struct Mission {
        int mission = 0;
        int state = 0;
        bool active = false;
    };

    #pragma pack(push, 1)
    struct VehicleState {
        double eta[6] = {0, 0, 0, 0, 0, 0};
        double nu[6] = {0, 0, 0, 0, 0, 0};
        double nu_dot[6] = {0, 0, 0, 0, 0, 0};
        double temperature = 0;
        double pressure = 0;
    };
    struct VehicleStateDes {
        double eta[6] = {10, 10, 10, 0, 0, -1.1};
        double nu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    };
    #pragma pack(pop)

    struct Mapped {
        std::vector<std::vector<std::vector<int>>> grid; // [z][y][x]
        int centerX, centerY; // Track grid center offsets
        const int width, height, depth;
    
        Mapped(int w = 40, int h = 40, int d = 2) 
            : width(w), height(h), depth(d),
              grid(d, std::vector<std::vector<int>>(h, std::vector<int>(w, 0))),
              centerX(w/2), centerY(h/2) {}
    
        void shiftGrid(int dx, int dy);
        int updateCell(int x, int y, int z, int value);
    };

    void updateState(const SensorData& data);
    VehicleState getCurrentState() const;
    VehicleStateDes getDesiredState() const;
    std::vector<uint8_t> serialize();
    void setDesiredState(const VehicleStateDes& state);

private:
    mutable std::mutex mtx;
    VehicleState currentState;
    VehicleStateDes desiredState;
    Mission mission;
    Mapped mapped;
};

#endif // ENVIRONMENT_STATE_H
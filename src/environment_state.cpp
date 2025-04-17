#include "environment_state.h"
#include "sensor_system.h"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <zlib.h>

void EnvironmentState::updateState(const SensorData& data, int interval) {
    VehicleState state;
    // Convert SensorData to VehicleState (example conversion logic)
    // Calculate new eta based on some logic (example: integrate nu over time)
    for (size_t i = 0; i < state.eta.size(); ++i) {
        state.eta[i] += data.nu[i] * interval; // Example integration
    }

    state.nu = data.nu;
    state.nu_dot = data.nu_dot;
    state.temperature = data.temperature;
    state.pressure = data.pressure;

    currentState.push(state);
}

EnvironmentState::VehicleState EnvironmentState::getCurrentState() {
    EnvironmentState::VehicleState state;
    if (!currentState.empty()) {
        const auto front = currentState.front(); // Retrieve the front element
        state = front; // Copy the value directly
    }
    return state;
}

EnvironmentState::VehicleStateDes EnvironmentState::getDesiredState() {
    VehicleStateDes state;
    if (!desiredState.empty()) {
        desiredState.pop(state); // Retrieve and remove the front element
    }
    return state;
}

void EnvironmentState::setDesiredState(const VehicleStateDes& state) {
    desiredState.push(state);
}

std::vector<uint8_t> EnvironmentState::serialize() {
    std::vector<uint8_t> buffer;
    
    // Header byte
    buffer.push_back('E');
    
    // Helper function to handle endianness
    auto append_double = [&buffer](double value) {
        // Convert to network byte order (big endian)
        uint64_t net_value;
        memcpy(&net_value, &value, sizeof(value));
        net_value = htobe64(net_value);  // From <endian.h> on Linux
        
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&net_value);
        buffer.insert(buffer.end(), bytes, bytes + sizeof(net_value));
    };
    
    // Lambda for binary packing
    auto pack_state = [&append_double](const auto& s, std::vector<uint8_t>& buf) {
        if constexpr (std::is_same_v<std::decay_t<decltype(s)>, VehicleState>) {
            for(double v : s.eta) append_double(v);
            for(double v : s.nu) append_double(v);
            for(double v : s.nu_dot) append_double(v);
            append_double(s.temperature);
            append_double(s.pressure);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(s)>, VehicleStateDes>) {
            for(double v : s.eta) append_double(v);  // Fixed: was s.eta
            for(double v : s.nu) append_double(v);  // Fixed: was s.nu
        }
    };
    
    pack_state(currentState, buffer);
    pack_state(desiredState, buffer);
    
    return buffer;
}

void EnvironmentState::Mapped::shiftGrid(int dx, int dy) {
    // Shift grid data when vehicle moves
    // Example:
    centerX += dx;
    centerY += dy;
    // Update grid based on new center position
    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                grid[z][y][x] = grid[z][y][(x + dx + width) % width];
            }
        }
    }
}

int EnvironmentState::Mapped::updateCell(int x, int y, int z, int value) {
    // Update voxel state
    if (x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth) {
        grid[z][y][x] = value;
        return 1;
    }
    return 0;
}

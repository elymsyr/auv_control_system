#include "mission_system.h"
#include <iostream>

void MissionSystem::set_mission(MissionMode mode) {
    mission_mode = mode;
    mission_state = 0;
}

MissionSystem::MissionMode MissionSystem::check_mission() const {
    return mission_mode;
}

void MissionSystem::mission_step() {
    switch (mission_mode) {
        case MissionMode::WAIT:
            break;
        case MissionMode::TEST_IMPLEMENTATION:    
            test_implementation();
            break;
    }
};

void MissionSystem::mission_report() {
    // Report mission status
    std::cout << "Mission report: " << mission_state << std::endl;
    // Increment mission state for demonstration
    mission_state++;
};

void MissionSystem::mission_test() {
    // Test mission implementation
    std::cout << "Testing mission implementation..." << std::endl;
    // Increment mission state for demonstration
    mission_state++;
};

void MissionSystem::mission_control() {
    // Control mission state
    std::cout << "Controlling mission state: " << mission_state << std::endl;
    // Increment mission state for demonstration
    mission_state++;
};

void MissionSystem::test_implementation() {
    
};

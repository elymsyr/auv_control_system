#include "main_system.h"
#include "sensor_system.h"
#include "environment_system.h"
#include "motion_system.h"
#include "control_system.h"
#include "communication_system.h"
#include "shared_data.h"
#include "shared_data.h"
#include "environment_state.h"
#include "shared_data.h"
#include "system_data.h"

int main() {
    SystemData systemData;
    SharedGroundCommand sharedGroundCommand;
    SharedSignalData sharedSignalData;
    EnvironmentState environmentData;
    
    MainSystem mainSystem(systemData, environmentData, sharedGroundCommand);
    
    CommunicationSystem commSystem("CommunicationSystem", 500, systemData, 1, environmentData, sharedGroundCommand);
    EnvironmentSystem envSystem("EnvironmentSystem", 200, systemData, 3, environmentData);
    MotionSystem motionSystem("MotionSystem", 200, systemData, 4, environmentData, sharedSignalData);
    ControlSystem controlSystem("ControlSystem", 200, systemData, 5, sharedSignalData);

    mainSystem.makeSystem({&commSystem, &envSystem, &motionSystem, &controlSystem});
    mainSystem.init(1);

    return 0;
}
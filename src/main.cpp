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
    SharedSensorData sharedSensorData;
    SharedGroundCommand sharedGroundCommand;
    SharedSignalData sharedSignalData;
    EnvironmentState environmentData;
    
    MainSystem mainSystem(systemData, environmentData, sharedGroundCommand);
    
    CommunicationSystem commSystem("CommunicationSystem", 500, systemData, 1, environmentData, sharedGroundCommand);
    SensorSystem sensorSystem("SensorSystem", 90, systemData, 2, sharedSensorData);
    EnvironmentSystem envSystem("EnvironmentSystem", 100, systemData, 3, sharedSensorData, environmentData);
    MotionSystem motionSystem("MotionSystem", 200, systemData, 4, environmentData, sharedSignalData);
    ControlSystem controlSystem("ControlSystem", 200, systemData, 5, sharedSignalData);

    mainSystem.makeSystem({&commSystem, &sensorSystem, &envSystem, &motionSystem, &controlSystem});
    mainSystem.init(1);

    return 0;
}
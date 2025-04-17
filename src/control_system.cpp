#include "control_system.h"
#include <thread>
#include <chrono>

ControlSystem::ControlSystem(std::string name, int runtime, SystemData& system, int order, SharedSignalData& signalData)
    : Subsystem(name, runtime, system, order), signalData(signalData) {}

bool ControlSystem::midInit() {
        return true;
}

void ControlSystem::midHalt() {
    commandQueue.reset();
    system.addPacket(0, 0, 1, 1, order);
}

void ControlSystem::pushCommand(const SignalData& cmd) {
    commandQueue.push(cmd);
}

void ControlSystem::liveLoop() {
    while (initialized.load()) {
        while (live.load()) {
            reset_timer();
            SignalData cmd;
            if (commandQueue.pop(cmd)) {
                executeControlCommands(cmd);
            }
            updateHeartbeat();
            sleep_til();
        }
        updateHeartbeat();
    }
}

void ControlSystem::executeControlCommands(const SignalData& cmd) {
    // Implementation for hardware control
}
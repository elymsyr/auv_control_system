#ifndef MAIN_SYSTEM_H
#define MAIN_SYSTEM_H

#include <vector>
#include <map>
#include "subsystem.h"
#include "environment_state.h"
#include "shared_data.h"

class MainSystem {
private:
    std::vector<std::unique_ptr<Subsystem>> subsystems;
    EnvironmentState& envState;
    SharedGroundCommand& groundCommand;
    std::atomic<bool> live{false};
    std::thread healthCheckThread;
    std::atomic<bool> initialized{false};

public:
    MainSystem(SystemData& system, EnvironmentState& envState, SharedGroundCommand& groundCommand);
    void addSubsystem(Subsystem* subsystem);
    void init(int order);
    void suspend(int order);
    void resume(int order);
    void restart(int order);
    void shutdown(int order);
    void makeSystem(std::vector<Subsystem*> newSubsystems) {
        subsystems.clear();
        for (auto* subsystem : newSubsystems) {
            subsystems.emplace_back(subsystem);
        }
    }
    void test();
    SystemData& system;

private:
    void checkSubsystemHealth();
    void handleSubsystemFailure(Subsystem* subsystem);
    void handleCommand(int system, int option, int control, int mode, int order);
};

#endif // MAIN_SYSTEM_H
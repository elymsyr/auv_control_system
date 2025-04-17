#include "main_system.h"
#include <thread>
#include <chrono>
#include <ctime>
#include <iostream>

MainSystem::MainSystem(SystemData& system, EnvironmentState& envState, SharedGroundCommand& groundCommand)
    : groundCommand(groundCommand), envState(envState), system(system)  // Initialize reference in member initializer list
{}

void MainSystem::addSubsystem(Subsystem* subsystem) {
    subsystems.push_back(subsystem);
}

void MainSystem::test() {
    std::cout << "Main system test function called." << std::endl;
}

void MainSystem::init(int order) {
    initialized.store(true);
    if (order == 0) {
        for(auto* subsystem : subsystems) {
            subsystem->init();
        }
    } else {
        if (order > 0 && order <= subsystems.size()) {
            subsystems[order-1]->init();
        } else {
            std::cerr << "Invalid subsystem order: " << order << std::endl;
        }
    }

    std::thread healthCheckThread([this]() {
        while (initialized) {
            checkSubsystemHealth();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    healthCheckThread.detach();

    std::mutex loopMutex;
    bool lockLoop = false;

    while (true) {
        std::unique_lock<std::mutex> lock(loopMutex);
        if (lockLoop) {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        checkSubsystemHealth();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto [system, option, control, mode, order] = groundCommand.read();

        lockLoop = true;
        
        handleCommand(system, option, control, mode, order);
        
        lockLoop = false;
    }
}

void MainSystem::handleCommand(int system, int option, int control, int mode, int order) {
    if (system != 9) std::cout << system << option << control << mode << order << std::endl;

    switch (system) {
        case 1:
            std::cout << "System command received: system=1" << std::endl;
            switch (option) {
                case 0:
                    std::cout << "Option command received: option=0" << std::endl;
                    switch (mode) {
                        case 0:
                            std::cout << "Mode command received: mode=0 (init)" << std::endl;
                            init(order);
                            break;
                        case 1:
                            std::cout << "Mode command received: mode=1 (suspend)" << std::endl;
                            suspend(order);
                            break;
                        case 2:
                            std::cout << "Mode command received: mode=2 (shutdown)" << std::endl;
                            shutdown(order);
                            break;
                        case 3:
                            std::cout << "Mode command received: mode=3 (resume)" << std::endl;
                            resume(order);
                            break;
                        case 4:
                            std::cout << "Mode command received: mode=4 (restart)" << std::endl;
                            restart(order);
                            break;
                        case 5:
                            std::cout << "Mode command received: mode=5 (checkSubsystemHealth)" << std::endl;
                            checkSubsystemHealth();
                            break;
                        case 6:
                            std::cout << "Mode command received: mode=6 (test)" << std::endl;
                            test();
                            break;
                        default:
                            std::cout << "Unknown mode command received: mode=" << mode << std::endl;
                            break;
                    }
                    break;
                default:
                    std::cout << "Unknown option command received: option=" << option << std::endl;
                    break;
            }
            break;
        case 2:
            std::cout << "System command received: system=2" << std::endl;
            switch (option) {
                case 0:
                    std::cout << "Option command received: option=0" << std::endl;
                    switch (mode) {
                        case 0:
                            std::cout << "Mode command received: mode=0" << std::endl;
                            std::cout << "Sending mission information..." << std::endl;
                            break;
                            // sendMissionInfo();
                        case 1:
                            std::cout << "Setting mission mode to " << mode << std::endl;
                            break;
                            // setMissionMode(mode);
                        case 2:
                            std::cout << "Starting mission..." << std::endl;
                            break;
                            // startMission();
                        case 3:
                            std::cout << "Generating mission report..." << std::endl;
                            break;
                            // generateMissionReport(mode == 1);
                        case 4:
                            std::cout << "Testing mission systems..." << std::endl;
                            break;
                            // testMissionSystems();
                        case 5:
                            std::cout << "Unknown mission control code: " << control << std::endl;
                            break;
                    }
            }
        default:
            break;
    }
}

void MainSystem::suspend(int order) {
    if (order == 0) {
        for (int i = subsystems.size() - 1; i >= 0; --i) {
            subsystems[i]->suspend();
            order -= 1;
        }
    } else {
        subsystems[order-1]->suspend();
    }
}

void MainSystem::resume(int order) {
    if (order == 0) {
        for(auto* subsystem : subsystems) {
            subsystem->resume();
        }
    } else {
        subsystems[order-1]->resume();
    }
}

void MainSystem::restart(int order) {
    if (order == 0) {
        for (int i = subsystems.size() - 1; i >= 0; --i) {
            subsystems[i]->restart();
            order -= 1;
        }
    } else {
        subsystems[order-1]->suspend();
    }
}

void MainSystem::shutdown(int order) {
    if (order == 0) {
        initialized.store(false);
        system.addPacket(2, 2, 2, 2, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        for (int i = subsystems.size() - 1; i >= 0; --i) {
            subsystems[i]->halt();
            order -= 1;
        }
        subsystems.clear();
        exit(0);
    }
    else {
        subsystems[order-1]->halt();
    }
}

void MainSystem::checkSubsystemHealth() {
    time_t now = time(nullptr);
    int order = 1;
    for(auto* subsystem : subsystems) {
        system.addPacket(2, 0, subsystem->isInitialized() ? 1 : 0, subsystem->isLive() ? 1 : 0, order);
        if(subsystem->isRunning() && now - subsystem->getLastHeartbeat() > 2) {
            system.addPacket(1, 9, 0, 0, order);
            handleSubsystemFailure(subsystem);
        }
        order += 1;
    }
}

void MainSystem::handleSubsystemFailure(Subsystem* subsystem) {
    subsystem->restart();
}

#include "system/main_system.h"

MainSystem::MainSystem(std::string host, bool debug, std::unordered_map<SystemID, int> system_configs, std::string name, int runtime, unsigned int system_code)
    : command_sub_(command_received, command_mtx, is_new_), Subsystem(name, runtime, system_code), system_configs_(std::move(system_configs))
{
    command_received.set();
    command_host_ = host;
    debug_mode_ = debug;
    init();
    for (const auto& [id, runtime] : system_configs_) {
        switch (id) {
            case SystemID::ENVIRONMENT:
                systems_[id] = std::make_unique<EnvironmentSystem>("Environment", runtime, 1);
                break;
            case SystemID::MISSION:
                systems_[id] = std::make_unique<MissionSystem>("Mission", runtime, 2);
                mission_system = dynamic_cast<MissionSystem*>(systems_[id].get());
                break;
            case SystemID::MOTION:
                systems_[id] = std::make_unique<MotionSystem>("Motion", runtime, 3);
                break;
            case SystemID::CONTROL:
                systems_[id] = std::make_unique<ControlSystem>("Control", runtime, 4);
                break;
            default:
                throw std::runtime_error("Unknown system ID");
        }
    }
    start();
}

MainSystem::~MainSystem() {
    try {
        std::lock_guard lk(mtx);
        shutdown_requested = true;
        cv_run.notify_one();
    } catch (const std::exception& e) {
        std::cerr << "Shutdown failed (0) for " << name << ": " << e.what() << "\n";
    }
    try {
        if (worker.joinable())
            worker.join();
        for (auto& [id, system] : systems_) {
            system->stop();
            system->halt();
        }
    } catch (const std::exception& e) {
        std::cerr << "Shutdown failed (1) for " << name << ": " << e.what() << "\n";
    }
    try {
        keep_alive_cv_.notify_one();
    } catch (const std::exception& e) {
        std::cerr << "Shutdown failed (2) for " << name << ": " << e.what() << "\n";
    }

}

void MainSystem::waitForDestruction() {
    std::unique_lock<std::mutex> lk(keep_alive_mtx_);
    keep_alive_cv_.wait(lk);
}

void MainSystem::init_() {
    int attempts = 0;
    int retries = 100;
    while (!command_sub_.is_running() && attempts < retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        command_sub_.connect("tcp://" + command_host_ + ":8889");
        attempts++;
    }
    initialized = true;
}

void MainSystem::function() {
    // std::unique_lock lock(command_mtx);
    if (is_new_) {
        CommandTopic cmd;
        {
            std::shared_lock slock(topic_read_mutex);
            cmd = command_received;
        }
        parse_command(cmd.system, cmd.command);
        is_new_ = false;
    }
}

void MainSystem::parse_command(int system, int operation) {
    try {
        std::cout << "Parsing command: system " << system << ", operation " << operation << "\n";
        std::unique_lock lock(system_mtx);
        if (system == 0) {
            switch (static_cast<::Operation>(operation)) {
                case ::Operation::INIT:
                    for (auto& [id, system] : systems_) {
                        system->init();
                    }
                    break;
                case ::Operation::START:
                    for (auto& [id, system] : systems_) {
                        system->start();
                    }
                    break;
                case ::Operation::STOP:
                    for (auto& [id, system] : systems_) {
                        system->stop();
                    }
                    break;
                case ::Operation::HALT:
                    std::cout << "HALT operation received. Shutting down program.\n";
                    std::exit(0);
                default:
                    std::cerr << "Unknown operation\n";
            }
        }
        else if (system == 6) {
            switch (static_cast<MissionIMP>(operation)) {
                case MissionIMP::SonarMisTest:
                    mission_system->setMission(MissionIMP::SonarMisTest);
                    std::cout << mission_system->getMission() << " mission set.\n";
                    break;
                case MissionIMP::FollowMisTest:
                    std::cout << "FollowMisTest is not implemented yet.\n";
                    break;
                case MissionIMP::Test:
                    std::cout << "Test is not implemented yet.\n";
                    break;
                case MissionIMP::SonarMis:
                    std::cout << "SonarMis is not implemented yet.\n";
                    break;
                case MissionIMP::FollowMis:
                    std::cout << "FollowMis is not implemented yet.\n";
                    break;
                case MissionIMP::START:
                    mission_system->startMission();
                    break;
                case MissionIMP::STOP:
                    mission_system->stopMission();
                    break;
                case MissionIMP::REPORT:
                    mission_system->reportMission();
                    break;
                default:
                    std::cerr << "Unknown operation\n";
            }
        } else {
            auto it = systems_.find(static_cast<SystemID>(system));
            if (it != systems_.end()) {
                switch (static_cast<::Operation>(operation)) {
                    case ::Operation::INIT:
                        it->second->init();
                        break;
                    case ::Operation::START:
                        it->second->start();
                        break;
                    case ::Operation::STOP:
                        it->second->stop();
                        break;
                    default:
                        std::cerr << "Unknown operation\n";
                }
            } else {
                std::cerr << "Unknown system ID\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Parsing failed for " << name << ": " << e.what() << "\n";
    }
}

void MainSystem::publish() {}

void MainSystem::start_test() {
    for (auto& [id, system] : systems_) {
        system->init();
        system->start();
    }
}

void MainSystem::halt() {
    stop();
    command_sub_.close();
    initialized = false;
}

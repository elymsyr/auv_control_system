#include "main_system.h"

MainSystem::MainSystem(std::string name, int runtime, unsigned int system_code, std::unordered_map<SystemID, int> system_configs)
    : command_sub_(command_received, command_mtx, is_new_), Subsystem(name, runtime, system_code), system_configs_(std::move(system_configs))
{
    command_received.set();
    proxy_thread = std::thread([this] { start_proxy(); });
    int retries = 0;
    while (!proxy_running_ && retries++ < 10) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (const auto& [id, runtime] : system_configs_) {
        switch (id) {
            case SystemID::ENVIRONMENT:
                systems_[id] = std::make_unique<EnvironmentSystem>("Environment", runtime, 1);
                break;
            case SystemID::MISSION:
                systems_[id] = std::make_unique<MissionSystem>("Mission", runtime, 2);
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
}

MainSystem::~MainSystem() {
    {
        std::lock_guard lk(mtx);
        shutdown_requested = true;
        cv_run.notify_one();
    }
    if (worker.joinable())
        worker.join();
    for (auto& [id, system] : systems_) {
        system->stop();
        system->halt();
    }
    proxy_ctx->close();
    if (proxy_thread.joinable())
        proxy_thread.join();
    notify(3, 0);
    state_pub_.close();
}

void MainSystem::init_() {
    command_sub_.connect("tcp://localhost:8889");
    start();
}

void MainSystem::function() {
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
    std::cout << "Parsing command: system " << system << ", operation " << operation << "\n";
    std::unique_lock lock(system_mtx);
    if (system == 0) {
        switch (static_cast<Operation>(operation)) {
            case Operation::INIT:
                for (auto& [id, system] : systems_) {
                    system->init();
                }
                break;
            case Operation::START:
                for (auto& [id, system] : systems_) {
                    system->start();
                }
                break;
            case Operation::STOP:
                for (auto& [id, system] : systems_) {
                    system->stop();
                }
                break;
            case Operation::HALT:
                for (auto& [id, system] : systems_) {
                    system->halt();
                }
                break;
            default:
                std::cerr << "Unknown operation\n";
        }
    }
    else if (system == 6) {
        std::cout << "Received a mission command\n";
        return;
    }
    else {
        auto it = systems_.find(static_cast<SystemID>(system));
        if (it != systems_.end()) {
            switch (static_cast<Operation>(operation)) {
                case Operation::INIT:
                    it->second->init();
                    break;
                case Operation::START:
                    it->second->start();
                    break;
                case Operation::STOP:
                    it->second->stop();
                    break;
                case Operation::HALT:
                    it->second->halt();
                    break;
                default:
                    std::cerr << "Unknown operation\n";
            }
        } else {
            std::cerr << "Unknown system ID\n";
        }
    }
}

void MainSystem::start_proxy() {
    zmq::context_t proxy_ctx(1);
    try {
        zmq::socket_t frontend(proxy_ctx, ZMQ_XSUB);
        zmq::socket_t backend(proxy_ctx, ZMQ_XPUB);
        
        frontend.bind("tcp://*:5555");
        backend.bind("tcp://*:8888");
        
        proxy_running_ = true;
        this->proxy_ctx = std::make_unique<zmq::context_t>(std::move(proxy_ctx));
        zmq::proxy(frontend, backend);  // Use updated proxy function
    }
    catch (const zmq::error_t& e) {
        if (e.num() != ETERM) {
            std::cerr << "Proxy error: " << e.what() << "\n";
        }
    }
    proxy_running_ = false;
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
}
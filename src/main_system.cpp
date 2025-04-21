#include "main_system.h"

MainSystem::MainSystem(std::string name, int runtime, unsigned int system_code, std::unordered_map<SystemID, int> system_configs)
    : command_sub_(command_received, command_mtx), system_sub_(system_state, system_mtx), Subsystem(name, runtime, system_code), system_configs_(std::move(system_configs))
{
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
    proxy_ctx.close();
    if (proxy_thread.joinable())
        proxy_thread.join();
}

void MainSystem::init_() {
    system_sub_.connect("tcp://localhost:5555");
    command_sub_.connect("tcp://localhost:8889");
    std::cout << name << " initialized\n";
}

void MainSystem::function() {}

void MainSystem::start_proxy() {
    zmq::context_t proxy_ctx(1);
    try {
        zmq::socket_t frontend(proxy_ctx, ZMQ_XSUB);
        zmq::socket_t backend(proxy_ctx, ZMQ_XPUB);
        
        frontend.bind("tcp://*:5555");
        backend.bind("tcp://*:8888");
        
        proxy_running_ = true;
        zmq::proxy_steerable(frontend, backend, nullptr, nullptr);
    }
    catch (const zmq::error_t& e) {
        if (e.num() != ETERM) {  // Ignore normal termination
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
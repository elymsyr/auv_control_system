#include "main_system.h"

MainSystem::MainSystem(std::string name, int runtime, unsigned int system_code, std::unordered_map<SystemID, int> system_configs)
    : Subsystem(name, runtime, system_code), system_configs_(std::move(system_configs))
{
    proxy_thread = std::thread([this] { start_proxy(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

void MainSystem::init() {
    system_sub_.connect("tcp://localhost:5555");
    command_sub_.connect("tcp://localhost:8889");
    std::cout << name << " initialized\n";
}

void MainSystem::function() {}

void MainSystem::refresh_received() {
    {
        std::lock_guard lk(mtx);
        try{
            command_received.set(command_sub_.receive());
            // TODO:
            // add a function -> void command_opeate(unsigned int system, unsigned int command)
            // operate init, start, stop, halt reset operations of the subsystems
            // control mission system to set start stop report operations for the mission
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to receive command\n";
        }
    }
}

void MainSystem::start_proxy() {
    zmq::socket_t frontend(proxy_ctx, ZMQ_XSUB);
    zmq::socket_t backend(proxy_ctx, ZMQ_XPUB);
    
    try {
        frontend.bind("tcp://*:5555");  // Subsystems connect here
        backend.bind("tcp://*:8888");    // GUI connects here
        zmq::proxy(frontend, backend);
    }
    catch (const zmq::error_t& e) {
        std::cerr << "Proxy error: " << e.what() << "\n";
    }
}

void MainSystem::publish() {}

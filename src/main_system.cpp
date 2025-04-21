#include "main_system.h"

System::System(Subsystem& sys, Subscriber<StateTopic>& sub, unsigned int ord)
    : system(sys), subscriber(sub), order(ord) {}


MainSystem::MainSystem(std::string name = "Main", int runtime = 200, unsigned int system_code = 0, std::unordered_map<SystemID, int> system_configs = { {SystemID::MISSION, 100}, {SystemID::CONTROL, 100}, {SystemID::MOTION, 100}, {SystemID::ENVIRONMENT, 50} })
    : Subsystem(name, runtime, system_code), system_configs_(std::move(system_configs))
{
    for (const auto& [id, runtime] : system_configs_) {
        switch (id) {
            case SystemID::ENVIRONMENT:
                systems_[id] = std::make_unique<System>( std::make_unique<EnvironmentSystem>("Environment", runtime, 1), Subscriber<StateTopic>(), 1 );
                break;
            case SystemID::MISSION:
                systems_[id] = std::make_unique<System>( std::make_unique<MissionSystem>("Mission", runtime, 2), Subscriber<StateTopic>(), 2 );
                break;
            case SystemID::MOTION:
                systems_[id] = std::make_unique<System>( std::make_unique<MotionSystem>("Motion", runtime, 3), Subscriber<StateTopic>(), 3 );
                break;
            case SystemID::CONTROL:
                systems_[id] = std::make_unique<System>( std::make_unique<ControlSystem>("Control", runtime, 4), Subscriber<StateTopic>(), 4 );
                break;
            default:
                throw std::runtime_error("Unknown system ID");
        }
    }
}

void MainSystem::init() {
    mission_sub_.connect("tcp://localhost:5561");
    env_sub_.connect("tcp://localhost:5560");
    command_sub_.connect("tcp://localhost:8889");
    sub_system_connect();
    std::cout << name << " initialized\n";
}

void MainSystem::function() {
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
        try{
            mission_state.set(mission_sub_.receive());
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to receive mission state\n";
        }
        try{
            env_state.set(env_sub_.receive());
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to receive environment state\n";
        }
    }
}

void MainSystem::publish() {}


void MainSystem::manage_systems(const std::vector<SystemID>& systems = {}, Operation operation) {
    auto performOperation = [&](Subsystem* system, Operation op) {
        if (op == Operation::INIT) {
            system->init();
        } else if (op == Operation::START) {
            system->start();
        } else if (op == Operation::STOP) {
            system->stop();
        } else if (op == Operation::HALT) {
            system->halt();
        } else {
            throw std::invalid_argument("Unknown operation");
        }
    };

    if (systems.empty()) {
        for (const auto& [id, system] : systems_) {
            performOperation(&system->system, operation);
        }
    } else {
        for (const auto& id : systems) {
            if (systems_.find(id) != systems_.end()) {
                performOperation(&systems_[id]->system, operation);
            } else {
                std::cerr << "System ID not found: " << static_cast<int>(id) << "\n";
            }
        }
    }
}

void MainSystem::sub_system_receive() {
    std::shared_lock lock(topic_read_mutex);
    for (const auto& [id, system] : systems_) {
        system->subscriber.receive();
    }
}

void MainSystem::sub_system_connect() {
    for (const auto& [id, system] : systems_) {
        system->subscriber.connect("tcp://localhost:" + std::to_string(5550 + system->order));
    }
}

void MainSystem::bind_system_pub() {
    state_pub_.bind("tcp://*:8888");
}

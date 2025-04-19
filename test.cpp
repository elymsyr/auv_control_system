
#include "include/environment_system.h"
#include "include/communication_system.h"
#include "include/topics.hpp"
#include "include/subsystem.h"
#include <iostream>
#include <zmq.hpp>
#include <chrono>
#include <thread>

int main() {
    // Create and initialize environment system
    EnvironmentSystem env_system;
    env_system.init();
    
    // Start the subsystem
    std::cout << "Starting environment system..." << std::endl;
    env_system.start();
    
    // Create subscriber to monitor environment data
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555");
    subscriber.set(zmq::sockopt::subscribe, StateTopic::TOPIC);
    
    // Monitor for 5 seconds
    auto start_time = std::chrono::steady_clock::now();
    while((std::chrono::steady_clock::now() - start_time) < std::chrono::seconds(5)) {
        zmq::message_t topic, data;
        
        // Proper error checking for both receives
        if (subscriber.recv(topic, zmq::recv_flags::dontwait)) {
            if (subscriber.recv(data)) {  // Check return value
                StateTopic state_data = Deserialize<StateTopic>(data);
                std::cout << "Received state update: " 
                          << (int)state_data.message_code << std::endl;
            } else {
                std::cerr << "Failed to receive data payload" << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // Stop the subsystem
    std::cout << "Stopping environment system..." << std::endl;
    env_system.stop();
    
    return 0;
}
#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <shared_mutex>
#include <iostream>
#include "topics.hpp"
#include "communication_system.h"


class Subsystem {
public:
    Publisher<StateTopic> state_pub_;
    StateTopic system_state;
    std::chrono::milliseconds runtime;
    std::string name;
    unsigned int system_code : 3;

    Subsystem(const std::string& name, int runtime, unsigned int system_code)
        :name(name),
        runtime(std::chrono::milliseconds(runtime)),
        system_code(system_code) {worker = std::thread([this]{ this->workerLoop(); });}
    ~Subsystem() {
        try {
            std::lock_guard lk(mtx);
            shutdown_requested = true;
            cv_run.notify_one();
        } catch (const std::exception& e) {
            std::cerr << "Shutdown failed (0) for " << name << ": " << e.what() << "\n";
        }
        try {
            for (int i = 0; i < 10 && !worker.joinable(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (worker.joinable())
                worker.join();
        } catch (const std::exception& e) {
            std::cerr << "Shutdown failed (1) for " << name << ": " << e.what() << "\n";
        }
        try {
            notify(3, 0);
            state_pub_.close();
        } catch (const std::exception& e) {
            std::cerr << "Shutdown failed (2) for " << name << ": " << e.what() << "\n";
        }
    }

    void init() {
        if (initialized) {
            std::cerr << "Subsystem " << name << " is already initialized.\n";
            return;
        }
        try {
            _init();
            init_();
            notify(0, 0);
            initialized = true;
            std::cout << "init: " << name << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Init failed for " << name << ": " << e.what() << "\n";
            throw;
        }
    }

    virtual void init_() {};

    virtual void _init() {
        int attempts = 0;
        while (attempts++ < 30) {
            try {
                state_pub_.connect("tcp://localhost:5555");
                state_pub_.publish(StateTopic{});
                break;
            } 
            catch (const zmq::error_t& e) {
                std::cout << "state_pub init failed (" << attempts << ") for " << name << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    virtual void start() {
        try {
            std::cout << "start: " << name << "\n";
            {
            std::lock_guard lk(mtx);
            run_requested = true;
            }
            cv_run.notify_one();
            notify(1, 0);
        } catch (const std::exception& e) {
            std::cerr << "Start failed for " << name << ": " << e.what() << "\n";
            throw;
        }
    }
    virtual void stop() {
        try {
            {
            std::lock_guard lk(mtx);
            run_requested = false;
            }
            cv_run.notify_one();
            notify(2, 0);
        } catch (const std::exception& e) {
            std::cerr << "Stop failed for " << name << ": " << e.what() << "\n";
            throw;
        }
    }
    virtual void halt() = 0;

    void notify(unsigned int process, uint8_t message) {
        try {
            {
                std::lock_guard<std::mutex> lock(system_state_mtx);
                system_state.set(system_code, process, message);
            }
        } catch (const std::exception& e) {
            std::cerr << "system_state.set failed for " << name << ": " << e.what() << "\n";
        }
        try {
            {
                std::shared_lock<std::shared_mutex> lock(topic_read_mutex);
                state_pub_.publish(system_state);
            }
        } catch (const std::exception& e) {
            std::cerr << "state_pub_.publish(system_state) failed for " << name << ": " << e.what() << "\n";
        }
    }

private:
    virtual void function() = 0;
    virtual void publish() = 0;

    void workerLoop() {
        try {
            std::unique_lock lk(mtx);
            while (!shutdown_requested) {
                cv_run.wait(lk, [&]{ return run_requested || shutdown_requested; });
                
                if (shutdown_requested) break;
        
                while (run_requested) {
                    lk.unlock();
                    
                    // Main work cycle
                    auto start = std::chrono::high_resolution_clock::now();
                    function();
                    publish();
                    
                    // Sleep with periodic wakeup checks
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    
                    lk.lock();
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Worker loop failed for " << name << ": " << e.what() << "\n";
        }
    }

protected:
    mutable std::shared_mutex topic_read_mutex; // std::shared_lock lock(state_mutex); (Multiple readers allowed, prevents writes.)
    std::thread worker;
    std::mutex  mtx, system_state_mtx;
    std::condition_variable cv_run, cv_halt;
    bool initialized = false, run_requested = false, shutdown_requested = false;
};

#endif // SUBSYSTEM_H
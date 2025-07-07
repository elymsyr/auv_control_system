#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <shared_mutex>
#include <iostream>
#include "communication/topics.hpp"
#include "communication/communication_methods.h"


class Subsystem {
public:
    // Publisher<StateTopic> state_pub_;
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
            for (int i = 0; i < 20 && !worker.joinable(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (worker.joinable())
                worker.join();
        } catch (const std::exception& e) {
            std::cerr << "Shutdown failed (1) for " << name << ": " << e.what() << "\n";
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
            // notify(0, 0);
            initialized = true;
            std::cout << "init: " << name << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Init failed for " << name << ": " << e.what() << "\n";
            throw;
        }
    }

    virtual void init_() {}

    virtual void _init() {}

    virtual void start() {
        try {
            {
            std::lock_guard lk(mtx);
            run_requested = true;
            }
            cv_run.notify_one();
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
        } catch (const std::exception& e) {
            std::cerr << "Stop failed for " << name << ": " << e.what() << "\n";
            throw;
        }
    }
    virtual void halt() = 0;

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(runtime));
                    
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
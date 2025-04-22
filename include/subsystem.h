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
        {
            std::lock_guard lk(mtx);
            shutdown_requested = true;
            cv_run.notify_one();
        }
        if (worker.joinable())
            worker.join();
        notify(3, 0);
        state_pub_.close();
    }

    virtual void init() {
        _init();
        init_();
        notify(0, 0);
        std::cout << "init: " << name << "\n";
    }

    virtual void init_() {};

    virtual void _init() {
        try {
            state_pub_.connect("tcp://localhost:5555");
        }
        catch (const std::exception& e) {
            std::cerr << "System "<< system_code << ": Failed to bind state publisher: " << e.what() << "\n";
            throw;
        }
    }

    virtual void start() {
        std::cout << "start: " << name << "\n";
        {
          std::lock_guard lk(mtx);
          run_requested = true;
        }
        cv_run.notify_one();
        notify(1, 0);
    }
    virtual void stop() {
        {
          std::lock_guard lk(mtx);
          run_requested = false;
        }
        cv_run.notify_one();
        notify(2, 0);
    }
    virtual void halt() = 0;

    void notify(unsigned int process, uint8_t message) {
        {
            std::lock_guard<std::mutex> lock(system_state_mtx);
            system_state.set(system_code, process, message);
        }
        {
            std::shared_lock<std::shared_mutex> lock(topic_read_mutex);
            state_pub_.publish(system_state);
        }
    }

private:
    virtual void function() = 0;
    virtual void publish() = 0;

    void workerLoop() {
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
    }

protected:
    mutable std::shared_mutex topic_read_mutex; // std::shared_lock lock(state_mutex); (Multiple readers allowed, prevents writes.)
    std::thread worker;
    std::mutex  mtx, system_state_mtx;
    std::condition_variable cv_run, cv_halt;
    bool run_requested = false, shutdown_requested = false;
};

#endif // SUBSYSTEM_H
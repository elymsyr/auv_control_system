#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <atomic>
#include <ctime>
#include <chrono>
#include <string>
#include <iostream>
#include <thread>
#include "system_data.h"

class Subsystem {
protected:
    std::atomic<bool> live{false};
    std::atomic<bool> initialized{false};
    std::atomic<time_t> lastHeartbeat{0};
    std::thread loopThread;

public:
    std::atomic<bool> restarting{false};
    Subsystem(const std::string& name, int runtime, SystemData& system, int order) : name(name), runtime(std::chrono::milliseconds(runtime)), system(system), order(order) {}
    virtual ~Subsystem() { if (loopThread.joinable()) loopThread.join(); }
    std::chrono::milliseconds runtime;
    SystemData& system;
    virtual bool isLive() const { return live.load(); }
    bool isRunning() const { return initialized.load() && live.load(); }
    bool isInitialized() const { return initialized.load(); }
    std::string getName() const { return name; }
    std::string name;
    int order;
    virtual void init() { bool post = preInit(); if (post) { bool loop = midInit(); if (loop) postInit(); } }
    virtual bool midInit() { return true; }
    virtual void halt() { bool post = preHalt(); if (post)  {midHalt(); } }
    virtual void midHalt() { system.addPacket(0, 0, 1, 1, order); return; }
    virtual void suspend() { bool post = preSuspend();  if (post) {midSuspend(); } }
    virtual void midSuspend() { system.addPacket(0, 0, 1, 2, order); return; }
    virtual void resume() { bool post = preResume();  if (post) {midResume(); } }
    virtual void midResume() { system.addPacket(0, 0, 1, 3, order); return; }
    virtual void restart() { if (!restarting.load()) {restarting.store(true); system.addPacket(1, 9, 0, 2, order); halt(); init(); restarting.store(false); }}
    virtual void liveLoop() = 0;
    virtual void updateHeartbeat() { lastHeartbeat = time(nullptr); }
    time_t getLastHeartbeat() const { return lastHeartbeat; }


    std::chrono::steady_clock::time_point timer = std::chrono::steady_clock::now();
    
    void reset_timer() {
        timer = std::chrono::steady_clock::now();
    }

    void sleep_til(int extra = 0){
        std::this_thread::sleep_until(timer + runtime + std::chrono::milliseconds(extra));
    }

private:
    void printNameProcess(std::string process) const {
        std::cout << process << ": " << name << std::endl;
    }
    virtual bool preInit() {
        if (initialized.load()) {
            system.addPacket(0, 0, 2, 0, order);
            return false;
        } else {
            initialized.store(true);
            live.store(true);
            updateHeartbeat();
            return true;
        }
    }
    virtual bool preHalt() {
        if (!initialized.load()){
            system.addPacket(0, 0, 2, 1, order);
            return false;
        }
        
        live.store(false);
        initialized.store(false);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        if (loopThread.joinable()) {
            try {
                    loopThread.join();
            } catch (...) {
                std::cerr << name << " detached." << std::endl;
                system.addPacket(1, 9, 0, 3, order);
                loopThread.detach();
            }
        }
        return true;
    }
    virtual bool preSuspend() {
        if (!isRunning()) {
            system.addPacket(0, 0, 0, 2, order);
            return false;
        } else {
            live.store(false);
            return true;
        }
    }
    virtual bool preResume() {
        if (!initialized.load()) {
            std::cerr << name << " is not initialized." << std::endl;
            system.addPacket(0, 0, 0, 0, order);
            system.addPacket(0, 0, 0, 3, order);
            return false;
        } else if (live.load()) {
            std::cerr << name << " is already running." << std::endl;
            system.addPacket(0, 0, 2, 3, order);
            return false;
        } else {
            live.store(true);
            updateHeartbeat();
            return true;
        }
    }
    virtual void postInit() {
        try {
            if (!loopThread.joinable()) {
                loopThread = std::thread([this]() {
                    pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
                    system.addPacket(0, 0, 1, 0, order);
                    liveLoop();
                });
            }
            else {system.addPacket(0, 0, 2, 0, order);}
        } catch (const std::exception& e) {
            std::cerr << "Exception in postInit: " << e.what() << std::endl;
            system.addPacket(0, 0, 0, 0, order);
        }
    }
};

#endif // SUBSYSTEM_H
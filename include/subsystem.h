#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

# include <thread>
# include <mutex>
# include <condition_variable>
# include <chrono>
# include <shared_mutex>
# include "topics.hpp"

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
        system_code(system_code)
    {
        bind_system_pub();
        worker = std::thread([this]{ this->workerLoop(); });
    }
    ~Subsystem() {
        {
        std::lock_guard lk(mtx);
        shutdown_requested = true;
        cv_run.notify_one();
        }
        if (worker.joinable())
            worker.join();
    }

    virtual void bind_system_pub() {
        state_pub_.bind("tcp://*:" + std::to_string(5550 + system_code));
    }
    virtual void init() = 0;
    virtual void start() {
        {
          std::lock_guard lk(mtx);
          run_requested = true;
        }
        cv_run.notify_one();
    }
    virtual void stop() {
        {
          std::lock_guard lk(mtx);
          run_requested = false;
        }
        cv_run.notify_one();
    }
    virtual void halt() {
        {
          std::lock_guard lk(mtx);
          run_requested = false;
          shutdown_requested = true;
        }
        cv_halt.notify_one();
    }

private:
    virtual void function() = 0;
    virtual void postState() = 0;

    void workerLoop() {
        std::unique_lock lk(mtx);
        while (!shutdown_requested) {
            cv_run.wait(lk, [&]{ return run_requested || shutdown_requested; });
            if (shutdown_requested) break;

            while (run_requested) {
                lk.unlock();
                auto start = std::chrono::high_resolution_clock::now();
                function();
                postState();
                std::this_thread::sleep_until(start + runtime);
                lk.lock();
            }
            cv_halt.wait(lk, [&]{ return !run_requested || shutdown_requested; });
        }
    }

protected:
    mutable std::shared_mutex topic_read_mutex; // std::shared_lock lock(state_mutex); (Multiple readers allowed, prevents writes.)
    std::thread worker;
    std::mutex  mtx;
    std::condition_variable cv_run, cv_halt;
    bool run_requested = false, shutdown_requested = false;
};

#endif // SUBSYSTEM_H
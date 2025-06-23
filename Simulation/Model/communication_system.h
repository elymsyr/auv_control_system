#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#pragma once
#include <zmq.hpp>
#include "topics.hpp"
#include <atomic>
#include <mutex>
#include <thread>

class ZeroMQContext {
public:
    static zmq::context_t& get() {
        static zmq::context_t instance(1);
        return instance;
    }
};

// Base Publisher
template <typename T>
class Publisher {
protected:
    zmq::socket_t socket_;
    static constexpr const char* TOPIC = T::TOPIC;
    bool bound_ = false;
    std::mutex socket_mutex_;
    
public:
    Publisher() : socket_(ZeroMQContext::get(), ZMQ_PUB) {
        socket_.set(zmq::sockopt::linger, 0);
        // socket_.set(zmq::sockopt::sndhwm, 1000);
    }

    ~Publisher() {
        close();
    }

    // For direct publishers
    void bind(const std::string& endpoint) {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if(bound_) return;
        try {
            socket_.bind(endpoint);
            bound_ = true;
        } catch (const zmq::error_t& e) {
            throw std::runtime_error("Bind failed: " + std::string(e.what()));
        }
    }

    // For proxy-connected publishers
    void connect(const std::string& endpoint) {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if(bound_) return;
        try {
            socket_.connect(endpoint);
            bound_ = true;
        } catch (const zmq::error_t& e) {
            throw std::runtime_error("Connect failed: " + std::string(e.what()));
        }
    }

    void publish(const T& data) {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (!bound_) {
            throw std::runtime_error("Publish failed: Socket not connected");
        }
        
        try {
            zmq::message_t topic_msg(TOPIC, strlen(TOPIC));
            zmq::message_t content_msg;
            Serialize(content_msg, data);
            
            // Use non-blocking send to prevent hangs
            if (!socket_.send(topic_msg, zmq::send_flags::sndmore | zmq::send_flags::dontwait)) {
                throw std::runtime_error("Send topic failed: Would block");
            }
            if (!socket_.send(content_msg, zmq::send_flags::dontwait)) {
                throw std::runtime_error("Send content failed: Would block");
            }
        } catch (const zmq::error_t& e) {
            if (e.num() == ETERM) {
                // Context was terminated, normal during shutdown
                return;
            }
            throw std::runtime_error("Publish failed: " + std::string(e.what()));
        }
    }

    void close() {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (bound_) {
            try {
                // Set linger to 0 for immediate close
                socket_.set(zmq::sockopt::linger, 0);
                socket_.close();
            } catch (...) {
                // Ignore errors during close
            }
            bound_ = false;
        }
    }

    bool is_bound() const { return bound_; }
};

// Base Subscriber
template <typename T>
class Subscriber {
protected:
    zmq::socket_t socket_;
    static constexpr const char* TOPIC = T::TOPIC;

private:
    T& data_;
    std::mutex& data_mutex_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};

    void receiver_loop() {
        while (running_) {
            zmq::message_t topic, content;
            
            try {
                if (socket_.recv(topic, zmq::recv_flags::dontwait)) {
                    if (std::string_view(static_cast<char*>(topic.data()), topic.size()) == TOPIC) {
                        if (socket_.recv(content, zmq::recv_flags::dontwait)) {
                            T new_data = Deserialize<T>(content);
                            {
                                std::lock_guard<std::mutex> lock(data_mutex_);
                                data_.set(new_data);  // Use the object's set method
                            }
                        }
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } catch (const zmq::error_t& e) {
                if (e.num() != ETERM) {
                    std::cerr << "Receiver error: " << e.what() << "\n";
                }
            }
        }
    }

public:
    // Constructor taking reference to external data and its mutex
    Subscriber(T& data, std::mutex& mutex) 
        : data_(data), data_mutex_(mutex), socket_(ZeroMQContext::get(), ZMQ_SUB) 
    {
        socket_.set(zmq::sockopt::linger, 0);
        socket_.set(zmq::sockopt::rcvtimeo, 100);
    }

    void connect(const std::string& endpoint) {
        try {
            socket_.connect(endpoint);
            socket_.set(zmq::sockopt::subscribe, TOPIC);
            running_ = true;
            worker_thread_ = std::thread(&Subscriber::receiver_loop, this);
        } catch (const zmq::error_t& e) {
            throw std::runtime_error("Connect failed: " + std::string(e.what()));
        }
    }

    // Get current data with thread-safe access
    T get_data() const {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            return data_;
        } catch (const std::exception& e) {
            throw std::runtime_error("Get data failed: " + std::string(e.what()));
        }
    }

    ~Subscriber() {
        close();
    }

    void close() {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            running_ = false;
            if (worker_thread_.joinable()) {
                worker_thread_.join();
            }
            socket_.close();
        } catch (const std::exception& e) {
            std::cerr << "Mutex lock failed: " << e.what() << "\n";
        }
    }

    bool is_running() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return running_;
    }
};

class SubscriberMain {
protected:
    zmq::socket_t socket_;
    static constexpr const char* TOPIC = CommandTopic::TOPIC;

private:
    CommandTopic& data_;
    bool& is_new_;
    std::mutex& data_mutex_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};

    void receiver_loop() {
        while (running_) {
            zmq::message_t topic, content;
            try {
                if (socket_.recv(topic) && 
                    std::string_view(static_cast<char*>(topic.data()), topic.size()) == TOPIC) {
                    if (socket_.recv(content)) {
                        CommandTopic new_data = Deserialize<CommandTopic>(content);
                        {
                            std::lock_guard<std::mutex> lock(data_mutex_);
                            data_.set(new_data);
                            is_new_ = true;
                        }
                    }
                }
            } catch (const zmq::error_t& e) {
                if (e.num() != ETERM) {
                    std::cerr << "Receiver error: " << e.what() << "\n";
                }
            }
        }
    }

public:
    // Constructor taking reference to external data and its mutex
    SubscriberMain(CommandTopic& data, std::mutex& mutex, bool& is_new_) 
        : data_(data), data_mutex_(mutex), is_new_(is_new_), socket_(ZeroMQContext::get(), ZMQ_SUB) 
    {
        socket_.set(zmq::sockopt::linger, 0);
        socket_.set(zmq::sockopt::rcvtimeo, 100);
    }

    void connect(const std::string& endpoint) {
        socket_.connect(endpoint);
        socket_.set(zmq::sockopt::subscribe, TOPIC);
        running_ = true;
        worker_thread_ = std::thread(&SubscriberMain::receiver_loop, this);
    }

    // Get current data with thread-safe access
    CommandTopic get_data() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_;
    }

    ~SubscriberMain() {
        close();
    }

    void close() {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        socket_.close();
    }

    bool is_running() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return running_;
    }
};

#endif // COMMUNICATION_H
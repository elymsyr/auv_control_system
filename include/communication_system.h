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
    bool is_bound = false;
    
public:
    Publisher() : socket_(ZeroMQContext::get(), ZMQ_PUB) {
        socket_.set(zmq::sockopt::linger, 0);
    }

    ~Publisher() {
        close();
    }

    // For direct publishers
    void bind(const std::string& endpoint) {
        if(is_bound) throw std::runtime_error("Socket already bound");
        socket_.bind(endpoint);
        is_bound = true;
    }

    // For proxy-connected publishers
    void connect(const std::string& endpoint) {
        if(is_bound) throw std::runtime_error("Socket already bound");
        socket_.connect(endpoint);
    }

    void publish(const T& data) {
        zmq::message_t topic_msg(TOPIC, strlen(TOPIC)); // Use C-string functions
        zmq::message_t content_msg;
        Serialize(content_msg, data);
        
        try {
            socket_.send(topic_msg, zmq::send_flags::sndmore);
            socket_.send(content_msg, zmq::send_flags::dontwait);
        }
        catch(const zmq::error_t& e) {
            if(e.num() != EAGAIN) {
                throw std::runtime_error("Publish failed: " + std::string(e.what()));
            }
        }
    }

    void close() {
        if(socket_.handle() != nullptr) {
            socket_.close();
        }
        is_bound = false;
    }
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
        socket_.connect(endpoint);
        socket_.set(zmq::sockopt::subscribe, TOPIC);
        running_ = true;
        worker_thread_ = std::thread(&Subscriber::receiver_loop, this);
    }

    // Get current data with thread-safe access
    T get_data() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_;
    }

    ~Subscriber() {
        close();
    }

    void close() {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        socket_.close();
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
};

#endif // COMMUNICATION_H
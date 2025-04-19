#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#pragma once
#include <zmq.hpp>
#include "topics.hpp"

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
    
public:
    Publisher() : socket_(ZeroMQContext::get(), ZMQ_PUB) {}
    
    void bind(const std::string& endpoint) {
        socket_.bind(endpoint);
    }

    void publish(const T& data) {
        zmq::message_t topic_msg(data.TOPIC, strlen(data.TOPIC));
        zmq::message_t content_msg;
        Serialize<T>(content_msg, data);  // Pass data to serialize
        
        socket_.send(topic_msg, zmq::send_flags::sndmore);
        socket_.send(content_msg, zmq::send_flags::none);
    }
};

// Base Subscriber
template <typename T>
class Subscriber {
protected:
    zmq::socket_t socket_;
    
public:
    Subscriber() : socket_(ZeroMQContext::get(), ZMQ_SUB) {}
    
    void connect(const std::string& endpoint) {
        socket_.connect(endpoint);
        socket_.set(zmq::sockopt::subscribe, T::TOPIC);
    }

    T receive() {
        zmq::message_t topic, content;
        
        // Check both receive operations
        if (!socket_.recv(topic)) {
            throw std::runtime_error("Failed to receive topic");
        }
        if (!socket_.recv(content)) {
            throw std::runtime_error("Failed to receive content");
        }
        
        return T::Deserialize(content);
    }
};

#endif // COMMUNICATION_H
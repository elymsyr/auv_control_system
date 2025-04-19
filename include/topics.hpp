#ifndef TOPICS_H
#define TOPICS_H

#pragma once
#include <cstring>
#include <zmq.hpp>

template<typename Topic>
void Serialize(zmq::message_t& msg, const Topic& data) {  // Add data parameter
    msg.rebuild(sizeof(Topic));
    memcpy(msg.data(), &data, sizeof(Topic));  // Use data reference
}

template<typename Topic>
Topic Deserialize(const zmq::message_t& msg) {
    Topic data;
    memcpy(&data, msg.data(), sizeof(Topic));
    return data;
}

// Environment System Topic
struct EnvironmentTopic {
    double eta[6];
    double nu[6];
    double nu_dot[6];

    static constexpr const char* TOPIC = "Environment";

    inline void set(const EnvironmentTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(const std::array<double, 6>& eta = {0, 0, 0, 0, 0, 0}, 
        const std::array<double, 6>& nu = {0, 0, 0, 0, 0, 0}, 
        const std::array<double, 6>& nu_dot = {0, 0, 0, 0, 0, 0}, 
        uint8_t msg_code = 0) {
        memcpy(this->eta, eta.data(), sizeof(this->eta));
        memcpy(this->nu, nu.data(), sizeof(this->nu));
        memcpy(this->nu_dot, nu_dot.data(), sizeof(this->nu_dot));
    }
};

// Mission System Topic
struct MissionTopic {
    double eta_des[6];
    double nu_des[6];
    double nu_dot_des[6];

    static constexpr const char* TOPIC = "Mission";

    inline void set(const MissionTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(const std::array<double, 6>& eta_des = {0, 0, 0, 0, 0, 0}, 
             const std::array<double, 6>& nu_des = {0, 0, 0, 0, 0, 0}, 
             const std::array<double, 6>& nu_dot_des = {0, 0, 0, 0, 0, 0}, 
             uint8_t msg_code = 0) {
        memcpy(this->eta_des, eta_des.data(), sizeof(this->eta_des));
        memcpy(this->nu_des, nu_des.data(), sizeof(this->nu_des));
        memcpy(this->nu_dot_des, nu_dot_des.data(), sizeof(this->nu_dot_des));
    }
};

// State System Topic
struct StateTopic {
    unsigned int system_code : 3;
    uint8_t message_code;
    std::time_t timestamp = std::time(nullptr);

    static constexpr const char* TOPIC = "State";

    inline void set(const StateTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(unsigned int code = 0, uint8_t msg_code = 0) {
        system_code = code & 0b111;
        message_code = msg_code;
        timestamp = std::time(nullptr);
    }
};

// Motion System Topic
struct MotionTopic {
    double propeller[6];

    static constexpr const char* TOPIC = "Motion";

    inline void set(const MotionTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(const std::array<double, 6>& propeller = {0, 0, 0, 0, 0, 0}) {
        memcpy(this->propeller, propeller.data(), sizeof(this->propeller));
    }
};

// Signal System Topic
struct SignalTopic {
    bool vision;
    bool sonar;
    bool gps;

    static constexpr const char* TOPIC = "Signal";

    inline void set(const SignalTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(bool vision = false, bool sonar = false, bool gps = false) {
        this->vision = vision;
        this->sonar = sonar;
        this->gps = gps;
    }
};

#endif // TOPICS_H
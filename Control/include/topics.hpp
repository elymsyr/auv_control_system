#ifndef TOPICS_H
#define TOPICS_H

#pragma once
#include <cstring>
#include <zmq.hpp>
#include <casadi/casadi.hpp>

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

    void get_dm(casadi::DM& merged) {
        if (merged.is_empty() || merged.size1() != 12 || merged.size2() != 1) {
            merged = casadi::DM::zeros(12, 1);
        }

        // Get writable pointer
        double* data = merged.ptr();
        if (data) {
            std::memcpy(data,     eta, 6 * sizeof(double));
            std::memcpy(data + 6, nu,   6 * sizeof(double));
        }
    }

    std::array<double, 12> get_array() {
        std::array<double, 12> arr;
        std::memcpy(arr.data(), eta, 6 * sizeof(double));
        std::memcpy(arr.data() + 6, nu, 6 * sizeof(double));
        return arr;
    }
};

// Mission System Topic
#define HORIZON 41

struct MissionTopic {
    double eta_des[HORIZON][6] = {{0}};
    double nu_des[HORIZON][6] = {{0}};

    static constexpr const char* TOPIC = "Mission";

    inline void set(const MissionTopic& o) {
        std::memcpy(eta_des, o.eta_des, sizeof(eta_des));
        std::memcpy(nu_des, o.nu_des, sizeof(nu_des));
    }

    void set(const std::array<double, 6>& eta = {0, 0, 0, 0, 0, 0},
                                const std::array<double, 6>& nu = {0, 0, 0, 0, 0, 0}) {
        for (int i = 0; i < HORIZON; i++) {
            std::memcpy(eta_des[i], eta.data(), 6 * sizeof(double));
            std::memcpy(nu_des[i], nu.data(), 6 * sizeof(double));
        }
    }

    void set(const std::vector<std::array<double, 6>>& eta_points,
                        const std::vector<std::array<double, 6>>& nu_points) {
        if (eta_points.size() != nu_points.size()) {
            throw std::invalid_argument("eta and nu vectors must be same size");
        }
        if (eta_points.size() > HORIZON) {
            throw std::out_of_range("Too many trajectory points");
        }
        for (int i = 0; i < HORIZON; i++) {
            std::memcpy(eta_des[i], eta_points[i].data(), 6 * sizeof(double));
            std::memcpy(nu_des[i], nu_points[i].data(), 6 * sizeof(double));
        }
    }

    void get_dm(casadi::DM& merged) const { // , const std::array<double, 12>& x_current
        if (merged.is_empty() || merged.size1() != 12 || merged.size2() != HORIZON) {
            merged = casadi::DM::zeros(12, HORIZON);
        }
        double* data = merged.ptr();

        for (int i = 0; i < HORIZON; i++) {
            // Process eta components (position/orientation)
            for (int j = 0; j < 6; j++) {
                data[i * 12 + j] = eta_des[i][j];
            }
            
            // Process nu components (velocity)
            for (int j = 0; j < 6; j++) {
                data[i * 12 + 6 + j] = nu_des[i][j];
            }
        }
    }

    // void get_dm(casadi::DM& merged) {
    //     if (merged.is_empty() || merged.size1() != 12 || merged.size2() != 1) {
    //         merged = casadi::DM::zeros(12, 1);
    //     }

    //     // Get writable pointer
    //     double* data = merged.ptr();
    //     if (data) {
    //         std::memcpy(data,     eta_des, 6 * sizeof(double));
    //         std::memcpy(data + 6, nu_des,   6 * sizeof(double));
    //     }
    // }

    void set(const casadi::DM& x_ref) {
        // Verify matrix dimensions
        if (x_ref.size1() != 12) {
            throw std::invalid_argument("x_ref must have exactly 12 rows");
        }
        
        const int horizon_cols = x_ref.size2();
        const double* data = x_ref.ptr();
        
        for (int k = 0; k < HORIZON; k++) {
            // Use last column if horizon exceeds trajectory length
            const int col_idx = (k < horizon_cols) ? k : horizon_cols - 1;
            
            // Extract eta (position/orientation) - first 6 elements
            for (int i = 0; i < 6; i++) {
                eta_des[k][i] = data[col_idx * 12 + i];
            }
            
            // Extract nu (velocities) - next 6 elements
            for (int i = 0; i < 6; i++) {
                nu_des[k][i] = data[col_idx * 12 + 6 + i];
            }
        }
    }

    std::array<double, 12> get_array() {
        std::array<double, 12> arr;
        std::memcpy(arr.data(), eta_des, 6 * sizeof(double));
        std::memcpy(arr.data() + 6, nu_des, 6 * sizeof(double));
        return arr;
    }
};

// State System Topic
struct StateTopic {
    unsigned int system : 3;
    unsigned int process : 3;
    uint8_t message;
    std::time_t timestamp = std::time(nullptr);

    static constexpr const char* TOPIC = "State";

    inline void set(const StateTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(unsigned int system_code = 0, unsigned int process_code = 0, uint8_t message_code = 0) {
        system = system_code & 0b111;
        process = process_code & 0b111;
        message = message_code;
        timestamp = std::time(nullptr);
    }
};

// State System Topic
struct CommandTopic {
    unsigned int system : 3;
    uint8_t command;
    std::time_t timestamp = std::time(nullptr);

    static constexpr const char* TOPIC = "Command";

    inline void set(const CommandTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(unsigned int system_code = 7, uint8_t command_code = 0) {
        system = system_code & 0b111;
        command = command_code;
        timestamp = std::time(nullptr);
    }
};

// Motion System Topic
struct MotionTopic {
    double propeller[8];
    double x_next[12];

    static constexpr const char* TOPIC = "Motion";

    void set(const casadi::DM& propeller_dm, const casadi::DM& x_next_dm) {
        if (propeller_dm.numel() != 8 || x_next_dm.numel() != 12) {
            throw std::runtime_error("Invalid propeller DM size");
        }

        const double* data = propeller_dm.ptr();
        std::copy(data, data + 8, propeller);
        data = x_next_dm.ptr();
        std::copy(data, data + 12, x_next);
    }

    void set(const std::array<double, 8>& propeller = {0, 0, 0, 0, 0, 0, 0, 0}, const std::array<double, 12>& x_next = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}) {
        std::memcpy(this->propeller, propeller.data(), sizeof(this->propeller));
        std::memcpy(this->x_next, x_next.data(), sizeof(this->x_next));
    }

    void get(std::vector<double>& propeller_copy, std::vector<double>& x_next_copy) {
        if (propeller_copy.size() < 8) propeller_copy.resize(8);
        std::memcpy(propeller_copy.data(), this->propeller, sizeof(this->propeller));

        if (x_next_copy.size() < 12) x_next_copy.resize(12);
        std::memcpy(x_next_copy.data(), this->x_next, sizeof(this->x_next));
    }

    inline void set(const MotionTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
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

struct TestSonarTopic {
    double detection[10];
    double degree[10];
    double timestamp = 0.0;

    static constexpr const char* TOPIC = "TestSonar";

    inline void set(const TestSonarTopic& o) {
        std::memcpy(this, &o, sizeof(*this));
    }

    void set(const std::array<double, 10>& detection = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
              const std::array<double, 10>& degree = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
              double timestamp = 0.0) {
        memcpy(this->detection, detection.data(), sizeof(this->detection));
        memcpy(this->degree, degree.data(), sizeof(this->degree));
        this->timestamp = timestamp;
    }
};

#endif // TOPICS_H
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>  // For rand()

template <typename T>
T clamp(const T& value, const T& min, const T& max) {
    return (value < min) ? min : (value > max) ? max : value;
}

struct Vector3 {
    double x, y, z;
    Vector3 operator+(const Vector3& other) const { return {x+other.x, y+other.y, z+other.z}; }
    Vector3 operator-(const Vector3& other) const { return {x-other.x, y-other.y, z-other.z}; }
    Vector3 operator*(double scalar) const { return {x*scalar, y*scalar, z*scalar}; }
    Vector3 operator/(double scalar) const { return {x/scalar, y/scalar, z/scalar}; }
    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }
};

template<typename State, typename DerivFunc>
State RK4Integrate(const State &state, double dt, DerivFunc f) {
    State k1 = f(state);
    State k2 = f(state + k1 * (dt / 2.0));
    State k3 = f(state + k2 * (dt / 2.0));
    State k4 = f(state + k3 * dt);
    return state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

struct Quaternion {
    double w, x, y, z;
    void normalize() {
        double norm = std::sqrt(w*w + x*x + y*y + z*z);
        if (norm < 1e-9) return;
        w /= norm; x /= norm; y /= norm; z /= norm;
    }
    Quaternion operator+(const Quaternion& other) const {
        return {w + other.w, x + other.x, y + other.y, z + other.z};
    }
    Quaternion operator*(double scalar) const {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }
};

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double min, double max)
        : Kp(Kp), Ki(Ki), Kd(Kd), min_output(min), max_output(max) {}

    double compute(double error, double dt) {
        double derivative = (error - prev_error) / dt;
        integral += error * dt;
        integral = clamp(integral, min_output/Ki, max_output/Ki);
        double output = Kp * error + Ki * integral + Kd * derivative;
        output = clamp(output, min_output, max_output);
        prev_error = error;
        return output;
    }
    
    void reset() { integral = 0; prev_error = 0; }

private:
    double Kp, Ki, Kd;
    double min_output, max_output;
    double integral = 0;
    double prev_error = 0;
};

class Quadcopter {
public:
    Quadcopter() :
        pos_pid_x(1.2, 0.1, 0.5, -0.3, 0.3),
        pos_pid_y(1.2, 0.1, 0.5, -0.3, 0.3),
        pos_pid_z(2.0, 0.1, 1.0, -10.0, 10.0),
        att_pid_roll(4.0, 0.1, 2.0, -6.0, 6.0),
        att_pid_pitch(4.0, 0.1, 2.0, -6.0, 6.0),
        att_pid_yaw(0.3, 0.01, 0.05, -0.2, 0.2)
    {
        motor_kf = 8.33e-8;
        motor_km = 1.5e-10;
        motor_electrical_tau = 0.01;
        motor_mech_tau = 0.05;
        min_rpm = 100.0;
        max_rpm = 10000.0;
        max_rpm_squared = max_rpm * max_rpm;
        min_rpm_squared = min_rpm * min_rpm;
        drag_coefficient = 0.02;
        wind = {0.0, 0.0, 0.0};

        mass = 1.2;
        arm_length = 0.25;
        max_thrust = 4 * motor_kf * max_rpm * max_rpm;
        min_thrust = 4 * motor_kf * min_rpm * min_rpm;

        inertia = {0.03, 0.03, 0.06};
        Ixx = 0.03;
        Iyy = 0.03;
        Izz = 0.06;

        reset_state();
    }

    struct State {
        struct Translational {
            Vector3 position;
            Vector3 velocity;
            Translational operator+(const Translational& other) const {
                return {position + other.position, velocity + other.velocity};
            }
            Translational operator*(double scalar) const {
                return {position * scalar, velocity * scalar};
            }
        };
        struct Rotational {
            Quaternion orientation;
            Vector3 angular_velocity;
            Rotational operator+(const Rotational& other) const {
                return {orientation + other.orientation, angular_velocity + other.angular_velocity};
            }
            Rotational operator*(double scalar) const {
                return {orientation * scalar, angular_velocity * scalar};
            }
        };
        Translational trans_state;
        Rotational rot_state;
        double motor_rpm[4];
    };

    void reset_state() {
        state.trans_state.position = {0, 0, 2.0};
        state.trans_state.velocity = {0, 0, 0};
        state.rot_state.orientation = {1, 0, 0, 0};
        state.rot_state.angular_velocity = {0, 0, 0};
        double hover_rpm = sqrt((mass * 9.81) / (4 * motor_kf));
        for (int i = 0; i < 4; i++) state.motor_rpm[i] = hover_rpm;
    }

    double update_motor(double target_rpm, double current_rpm, double dt) {
        double alpha = (1.0 / (motor_electrical_tau + motor_mech_tau));
        double rpm_rate = alpha * (target_rpm - current_rpm);
        current_rpm += rpm_rate * dt;
        return clamp(current_rpm, min_rpm, max_rpm);
    }

    void update(double dt, const Vector3& target_pos) {
        Vector3 noisy_position = state.trans_state.position + Vector3{gaussian_noise(), gaussian_noise(), gaussian_noise()};
        Vector3 pos_error = target_pos - noisy_position;
        double base_thrust = mass * 9.81;  // Total thrust needed to hover (Newtons)
        double pid_output_z = pos_pid_z.compute(pos_error.z, dt);
        double T = clamp(base_thrust + pid_output_z, 0.5 * base_thrust, 1.5 * base_thrust);
        std::cout << "T: " << T << std::endl;
        double desired_pitch = clamp(pos_pid_x.compute(pos_error.x, dt), -0.4, 0.4);  // Increased from ±0.15
        double desired_roll  = clamp(pos_pid_y.compute(pos_error.y, dt), -0.4, 0.4);

        std::cout << "desired_pitch: " << desired_pitch << std::endl;
        std::cout << "desired_roll: " << desired_roll << std::endl;

        Vector3 euler = quaternion_to_euler(state.rot_state.orientation);
        double torque_roll  = att_pid_roll.compute(desired_roll - euler.x, dt);
        double torque_pitch = att_pid_pitch.compute(desired_pitch - euler.y, dt);
        double target_yaw = std::atan2(target_pos.y - state.trans_state.position.y, target_pos.x - state.trans_state.position.x);
        double yaw_error = target_yaw - euler.z;
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
        double torque_yaw = att_pid_yaw.compute(yaw_error, dt);
        Vector3 tau = {torque_roll, torque_pitch, torque_yaw};
        std::cout << "tau: " << tau.x << ", " << tau.y << ", " << tau.z << std::endl;
        
        const double L = arm_length / sqrt(2.0);
        double c = motor_km / motor_kf;
        
        // Corrected mixing for X-configuration:
        double f0 = (T/4) - (tau.x/(4*L)) - (tau.y/(4*L)) + (tau.z/(4*c));
        double f1 = (T/4) + (tau.x/(4*L)) - (tau.y/(4*L)) - (tau.z/(4*c));
        double f2 = (T/4) + (tau.x/(4*L)) + (tau.y/(4*L)) + (tau.z/(4*c));
        double f3 = (T/4) - (tau.x/(4*L)) + (tau.y/(4*L)) - (tau.z/(4*c));
        
        double target_w_sq[4] = {
            clamp(f0/motor_kf, min_rpm_squared, max_rpm_squared),
            clamp(f1/motor_kf, min_rpm_squared, max_rpm_squared),
            clamp(f2/motor_kf, min_rpm_squared, max_rpm_squared),
            clamp(f3/motor_kf, min_rpm_squared, max_rpm_squared)
        };
        std::cout << "target_w_sq: " << target_w_sq[0] << ", " << target_w_sq[1] << ", " << target_w_sq[2] << ", " << target_w_sq[3] << std::endl;
    
        for (int i = 0; i < 4; i++) {
            double target_rpm = sqrt(target_w_sq[i]);
            state.motor_rpm[i] = update_motor(target_rpm, state.motor_rpm[i], dt);
        }
        
        Vector3 force_body{0, 0, 0};
        Vector3 torque_body{0, 0, 0};
        for (int i = 0; i < 4; i++) {
            double f = motor_kf * pow(state.motor_rpm[i], 2);
            force_body.z += f;
            
            // X-configuration torque contributions (motors 0:FL, 1:FR, 2:BR, 3:BL)
            switch(i) {
                case 0: // Front-left: +roll +pitch
                    torque_body.x += f * L;
                    torque_body.y += f * L;
                    break;
                case 1: // Front-right: -roll +pitch
                    torque_body.x -= f * L;
                    torque_body.y += f * L;
                    break;
                case 2: // Back-right: -roll -pitch
                    torque_body.x -= f * L;
                    torque_body.y -= f * L;
                    break;
                case 3: // Back-left: +roll -pitch
                    torque_body.x += f * L;
                    torque_body.y -= f * L;
                    break;
            }
            torque_body.z += ((i % 2 == 0) ? 1.0 : -1.0) * motor_km * pow(state.motor_rpm[i], 2);
        }
        
        Vector3 relative_velocity = state.trans_state.velocity - wind;
        Vector3 aero_drag = relative_velocity * (-drag_coefficient * relative_velocity.magnitude());
        Vector3 gravity = {0, 0, -mass * 9.81};
        Vector3 thrust_world = rotate_vector(state.rot_state.orientation, force_body);
        Vector3 acceleration = (thrust_world + gravity + aero_drag) / mass;
        
        auto translational_deriv = [acceleration](const State::Translational &s) -> State::Translational {
            State::Translational ds;
            ds.position = s.velocity;
            ds.velocity = acceleration;
            return ds;
        };
        
        auto rotational_deriv = [this, torque_body](const State::Rotational &s) -> State::Rotational {
            State::Rotational ds;
            ds.angular_velocity = calculate_angular_acceleration(torque_body, s.angular_velocity);
            ds.orientation = quaternion_derivative(s.orientation, s.angular_velocity);
            return ds;
        };
        
        state.trans_state = RK4Integrate(state.trans_state, dt, translational_deriv);
        state.rot_state = RK4Integrate(state.rot_state, dt, rotational_deriv);
        state.rot_state.orientation.normalize();
        
        // if (std::abs(state.trans_state.position.z) > 100.0) {
        //     reset_state();
        // }
    }

    Vector3 quaternion_to_euler(const Quaternion& q) const {
        Vector3 euler;
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x*q.x + q.y*q.y);
        euler.x = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (q.w * q.y - q.z * q.x);
        euler.y = (std::abs(sinp) >= 1) ? std::copysign(M_PI/2, sinp) : std::asin(sinp);

        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z);
        euler.z = std::atan2(siny_cosp, cosy_cosp);
        return euler;
    }

    State get_state() const { return state; }
    double get_motor_kf() const { return motor_kf; }

private:
    State state;
    double drag_coefficient;
    PIDController pos_pid_x, pos_pid_y, pos_pid_z;
    PIDController att_pid_roll, att_pid_pitch, att_pid_yaw;
    double mass, arm_length, motor_kf, motor_km, max_rpm, min_rpm, max_thrust, min_thrust, motor_electrical_tau, motor_mech_tau, max_rpm_squared, min_rpm_squared;
    Vector3 inertia, wind;
    double Ixx, Iyy, Izz;

    Vector3 calculate_angular_acceleration(const Vector3& torque, const Vector3& omega) const {
        Vector3 angular_accel;
        angular_accel.x = (torque.x - (Izz - Iyy) * omega.y * omega.z) / Ixx;
        angular_accel.y = (torque.y - (Ixx - Izz) * omega.x * omega.z) / Iyy;
        angular_accel.z = (torque.z - (Iyy - Ixx) * omega.x * omega.y) / Izz;
        return angular_accel;
    }

    Quaternion quaternion_derivative(const Quaternion& q, const Vector3& omega) const {
        return {
            (-q.x * omega.x - q.y * omega.y - q.z * omega.z) * 0.5,
            ( q.w * omega.x + q.z * omega.y - q.y * omega.z) * 0.5,
            ( q.w * omega.y - q.z * omega.x + q.x * omega.z) * 0.5,
            ( q.w * omega.z + q.y * omega.x - q.x * omega.y) * 0.5
        };
    }

    Vector3 rotate_vector(const Quaternion& q, const Vector3& v) const {
        return {
            (1 - 2*q.y*q.y - 2*q.z*q.z)*v.x + (2*q.x*q.y - 2*q.w*q.z)*v.y + (2*q.x*q.z + 2*q.w*q.y)*v.z,
            (2*q.x*q.y + 2*q.w*q.z)*v.x + (1 - 2*q.x*q.x - 2*q.z*q.z)*v.y + (2*q.y*q.z - 2*q.w*q.x)*v.z,
            (2*q.x*q.z - 2*q.w*q.y)*v.x + (2*q.y*q.z + 2*q.w*q.x)*v.y + (1 - 2*q.x*q.x - 2*q.y*q.y)*v.z
        };
    }

    double gaussian_noise() const {
        return (std::rand() % 1000 - 500) / 5000.0;
    }
};

int main() {
    Quadcopter quad;
    double dt = 0.002;
    double time = 0;
    const double duration = 10.0;
    const int steps = static_cast<int>(duration/dt);

    std::cout << "Time,X,Y,Z,Roll,Pitch,Yaw,M1,M2,M3,M4\n";
    Vector3 target_pos{3.0, 0.0, 2.0};

    for (int i = 0; i < steps; ++i) {
        time = i * dt;
        quad.update(dt, target_pos);
        auto state = quad.get_state();
        auto euler = quad.quaternion_to_euler(state.rot_state.orientation);

        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               time, state.trans_state.position.x, state.trans_state.position.y, state.trans_state.position.z,
               euler.x * 180/M_PI, euler.y * 180/M_PI, euler.z * 180/M_PI,
               state.motor_rpm[0], state.motor_rpm[1], 
               state.motor_rpm[2], state.motor_rpm[3]);
    }
    return 0;
}
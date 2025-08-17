#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include <casadi/casadi.hpp>
#include "communication/topics.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <optional>
#include <string>

using json = nlohmann::json;

class VehicleModel {
public:
    explicit VehicleModel(const std::string& config_path);

    casadi::MX barrier_function_mx(const casadi::MX& x, const casadi::MX& y) const;
    casadi::MX skew_symmetric(const casadi::MX& a) const;
    casadi::MX transformation_matrix(const casadi::MX& eta) const;
    casadi::MX coriolis_matrix(const casadi::MX& nu) const;
    casadi::MX damping_matrix(const casadi::MX& nu) const;
    casadi::MX restoring_forces(const casadi::MX& eta) const;
    std::pair<casadi::MX, casadi::MX> dynamics(const casadi::MX& eta, const casadi::MX& nu, const casadi::MX& tau_p) const;
    std::array<double, 12> calculate_next_state(
        const EnvironmentTopic& current_state,
        const std::array<double, 8>& propeller_thrust, // Assuming 6 propellers for example
        double dt) const;

    casadi::MX get_A_matrix() const;
    casadi::MX get_M_inv() const;
    double get_p_front_mid_max() const;
    double get_p_rear_max() const;
    bool has_map() const;

private:
    void load_config(const std::string& path);
    void calculate_linear();

    // Member variables
    double Ixx_, Ixy_, Ixz_, Iyx_, Iyy_, Iyz_, Izx_, Izzy_, Izz_;
    double Lxx_, Lxy_, Lxz_, Lyx_, Lyy_, Lyz_, Lzx_, Lzy_, Lzz_;
    double r_x_, r_y_, r_z_;
    double x_B_, y_B_, z_B_;
    double w_, h_, l_, lm_, wf_, lr_, rf_;
    double mass_, a_, volume_;
    double fluid_density_, displaced_volume_, g_;
    double D_u_, D_v_, D_w_, D_p_, D_q_, D_r_;
    double Dn_u_, Dn_v_, Dn_w_, Dn_p_, Dn_q_, Dn_r_;
    double C_X_, C_Y_, C_Z_, C_Y_r_, C_Z_q_, C_K_, C_M_, C_N_;
    double p_rear_max_, p_front_mid_max_;
    double W_, B_, W_minus_B_;
    int N;

    casadi::MX r_g_, r_B_;
    casadi::MX A_, M_, M_inv_, Ma_, Dl_, Dn_;
    casadi::MX skew_m_, skew_I_, skew_A11_, skew_A22_;
};

#endif // VEHICLE_MODEL_H
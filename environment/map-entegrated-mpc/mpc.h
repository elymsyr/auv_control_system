#ifndef MPC_H
#define MPC_H

#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <optional>
#include <string>
#include "EnvironmentMap.h"

using namespace casadi;
using json = nlohmann::json;

class VehicleModel {
public:
    explicit VehicleModel(const std::string& config_path, EnvironmentMap* map);

    MX barrier_function_mx(const MX& x, const MX& y) const;
    MX skew_symmetric(const MX& a) const;
    MX transformation_matrix(const MX& eta) const;
    MX coriolis_matrix(const MX& nu) const;
    MX damping_matrix(const MX& nu) const;
    MX restoring_forces(const MX& eta) const;
    std::pair<MX, MX> dynamics(const MX& eta, const MX& nu, const MX& tau_p) const;

    MX get_A_matrix() const;
    MX get_M_inv() const;
    double get_p_front_mid_max() const;
    double get_p_rear_max() const;
    bool has_map() const { return map_ != nullptr; }

    EnvironmentMap* map_;
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

    MX r_g_, r_B_;
    MX A_, M_, M_inv_, Ma_, Dl_, Dn_;
    MX skew_m_, skew_I_, skew_A11_, skew_A22_;
};

class NonlinearMPC {
public:
    NonlinearMPC(const VehicleModel& model, int N = 20, double dt = 0.1, MX eps = MX(1e-3));
    void reset_previous_solution();
    std::pair<DM, DM> solve(const DM& x0, const DM& x_ref);

private:
    VehicleModel model_;
    int N_, nx_, nu_;
    double dt_;
    Opti opti_;
    MX X_, U_, eps_;
    MX x0_param_, x_ref_param_;
    std::optional<OptiSol> prev_sol_;

    void setup_optimization();
};

#endif // MPC_H
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <optional>
#include <fstream>
#include <string>
#include <zmq.hpp>
#include <chrono>
#include <cstring>

using namespace casadi;
using json = nlohmann::json;

class VehicleModel {
public:
    explicit VehicleModel(const std::string& config_path) {
        load_config(config_path);
        calculate_linear();
    }

    // Now working with MX instead of DM.
    MX skew_symmetric(const MX& a) const {
        return MX::vertcat({
            MX::horzcat({0, -a(2), a(1)}),
            MX::horzcat({a(2), 0, -a(0)}),
            MX::horzcat({-a(1), a(0), 0})
        });
    }

    MX transformation_matrix(const MX& eta) const {
        MX phi   = eta(3);
        MX theta = eta(4);
        MX psi   = eta(5);
        
        MX R = MX::vertcat({
            MX::horzcat({cos(psi)*cos(theta), 
                         -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), 
                         sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)}),
            MX::horzcat({sin(psi)*cos(theta), 
                         cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), 
                         -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi)}),
            MX::horzcat({-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)})
        });
        
        MX T = MX::vertcat({
            MX::horzcat({1, sin(phi)*tan(theta), cos(phi)*tan(theta)}),
            MX::horzcat({0, cos(phi), -sin(phi)}),
            MX::horzcat({0, sin(phi)/cos(theta), cos(phi)/cos(theta)})
        });
        
        return MX::blockcat({{R, MX::zeros(3,3)}, {MX::zeros(3,3), T}});
    }

    MX coriolis_matrix(const MX& nu) const {
        MX nu1 = nu(Slice(0, 3));
        MX nu2 = nu(Slice(3, 6));
        MX skew_nu1 = skew_symmetric(nu1);
        MX skew_nu2 = skew_symmetric(nu2);
        
        MX Crb_top = MX::horzcat({MX::zeros(3,3), -mass_ * skew_nu1 - mtimes(skew_nu2, skew_m_)});
        MX Crb_bottom = MX::horzcat({-mass_ * skew_nu1 + mtimes(skew_nu2, skew_m_), -mtimes(skew_nu2, skew_I_)});
        MX Crb = MX::vertcat({Crb_top, Crb_bottom});
        
        MX Ca_top = MX::horzcat({MX::zeros(3,3), -mtimes(skew_A11_, skew_nu1)});
        MX Ca_bottom = MX::horzcat({-mtimes(skew_A11_, skew_nu1), -mtimes(skew_A22_, skew_nu2)});
        MX Ca = MX::vertcat({Ca_top, Ca_bottom});
        
        return simplify(Crb + Ca);
    }

    MX damping_matrix(const MX& nu) const {
        Sparsity diag6_sp = Sparsity::diag(6);
        MX Dn = MX::zeros(diag6_sp);
        for(int i=0; i<6; ++i) Dn(i,i) = fabs(nu(i)) * Dn_(i,i);
        return Dl_ + Dn;
    }

    MX restoring_forces(const MX& eta) const {
        MX phi   = eta(3);
        MX theta = eta(4);
        
        MX g_eta = MX::vertcat({
            W_minus_B_ * sin(theta),
            -W_minus_B_ * cos(theta) * sin(phi),
            -W_minus_B_ * cos(theta) * cos(phi),
            -(r_y_ * W_ - y_B_ * B_) * cos(theta)*cos(phi) + (r_z_ * W_ - z_B_ * B_) * cos(theta)*sin(phi),
            (r_z_ * W_ - z_B_ * B_) * sin(theta) + (r_x_ * W_ - x_B_ * B_) * cos(theta)*cos(phi),
            -(r_x_ * W_ - x_B_ * B_) * cos(theta)*sin(phi) - (r_y_ * W_ - y_B_ * B_) * sin(theta)
        });
        return g_eta;
    }

    // Now dynamics returns a pair of MX objects.
    std::pair<MX, MX> dynamics(const MX& eta, const MX& nu, const MX& tau_p) const {
        MX J_eta   = transformation_matrix(eta);
        MX eta_dot = mtimes(J_eta, nu);
        
        MX C      = coriolis_matrix(nu);
        MX D      = damping_matrix(nu);
        MX g      = restoring_forces(eta);
        
        MX tau    = mtimes(A_, tau_p);
        MX nu_dot = simplify(mtimes(M_inv_, tau - mtimes(C, nu) - mtimes(D, nu) - g));
        
        return {eta_dot, nu_dot};
    }

    MX get_A_matrix() const { return A_; }
    MX get_M_inv() const { return M_inv_; }
    double get_p_front_mid_max() const { return p_front_mid_max_; }
    double get_p_rear_max() const { return p_rear_max_; }

private:
    void load_config(const std::string& path) {
        std::ifstream f(path);
        json config = json::parse(f)["assembly_mass_properties"];

        // Load inertia parameters
        auto moments = config["moments_of_inertia_about_output_coordinate_system"];
        Ixx_ = moments["Ixx"].get<double>();
        Ixy_ = moments["Ixy"].get<double>();
        Ixz_ = moments["Ixz"].get<double>();
        Iyx_ = moments["Iyx"].get<double>();
        Iyy_ = moments["Iyy"].get<double>();
        Iyz_ = moments["Iyz"].get<double>();
        Izx_ = moments["Izx"].get<double>();
        Izzy_ = moments["Izy"].get<double>();
        Izz_ = moments["Izz"].get<double>();

        // Load COM inertia
        auto moments_com = config["moments_of_inertia_about_center_of_mass"];
        Lxx_ = moments_com["Lxx"].get<double>();
        Lxy_ = moments_com["Lxy"].get<double>();
        Lxz_ = moments_com["Lxz"].get<double>();
        Lyx_ = moments_com["Lyx"].get<double>();
        Lyy_ = moments_com["Lyy"].get<double>();
        Lyz_ = moments_com["Lyz"].get<double>();
        Lzx_ = moments_com["Lzx"].get<double>();
        Lzy_ = moments_com["Lzy"].get<double>();
        Lzz_ = moments_com["Lzz"].get<double>();

        // Load center of mass
        auto com = config["center_of_mass"];
        r_x_ = com["X"].get<double>();
        r_y_ = com["Y"].get<double>();
        r_z_ = com["Z"].get<double>();
        r_g_ = MX::vertcat({r_x_, r_y_, r_z_});

        // Center of buoyancy
        auto buoyancy = config["center_of_buoancy"];
        x_B_ = buoyancy["X"].get<double>();
        y_B_ = buoyancy["Y"].get<double>();
        z_B_ = buoyancy["Z"].get<double>();
        r_B_ = MX::vertcat({x_B_, y_B_, z_B_});

        // Dimensions
        auto dim = config["dimensions"];
        w_ = dim["width"].get<double>();
        h_ = dim["height"].get<double>();
        l_ = dim["length"].get<double>();
        lm_ = dim["lm"].get<double>();
        wf_ = dim["wf"].get<double>();
        lr_ = dim["lr"].get<double>();

        // Mass and parameters
        mass_ = config["mass"]["value"].get<double>();
        double a_deg = config["rear_propeller_angle"]["value"].get<double>();
        a_ = a_deg * M_PI / 180.0;
        volume_ = config["volume"]["value"].get<double>();

        C_X_ = config["added_mass"]["C_X"].get<double>();
        C_Y_ = config["added_mass"]["C_Y"].get<double>();
        C_Z_ = config["added_mass"]["C_Z"].get<double>();
        C_Y_r_ = config["added_mass"]["C_Y_r"].get<double>();
        C_Z_q_ = config["added_mass"]["C_Z_q"].get<double>();
        C_K_ = config["added_mass"]["C_K"].get<double>();
        C_M_ = config["added_mass"]["C_M"].get<double>();
        C_N_ = config["added_mass"]["C_N"].get<double>();

        // Dynamics parameters
        auto dynamics = config["dynamics"];
        fluid_density_ = dynamics["fluid_density"]["value"].get<double>();
        displaced_volume_ = dynamics["displaced_volume"]["value"].get<double>();
        g_ = dynamics["g"]["value"].get<double>();

        // Damping coefficients
        auto damping = dynamics["damping"];
        D_u_ = damping["linear"]["D_u"].get<double>();
        D_v_ = damping["linear"]["D_v"].get<double>();
        D_w_ = damping["linear"]["D_w"].get<double>();
        D_p_ = damping["angular"]["D_p"].get<double>();
        D_q_ = damping["angular"]["D_q"].get<double>();
        D_r_ = damping["angular"]["D_r"].get<double>();
        
        Dn_u_ = damping["linear_n"]["Dn_u"].get<double>();
        Dn_v_ = damping["linear_n"]["Dn_v"].get<double>();
        Dn_w_ = damping["linear_n"]["Dn_w"].get<double>();
        Dn_p_ = damping["angular_n"]["Dn_p"].get<double>();
        Dn_q_ = damping["angular_n"]["Dn_q"].get<double>();
        Dn_r_ = damping["angular_n"]["Dn_r"].get<double>();

        // Propeller parameters
        auto propeller = dynamics["propeller"];
        p_rear_max_ = propeller["force_range_r"]["max"].get<double>();
        p_front_mid_max_ = propeller["force_range_f_m"]["max"].get<double>();

        // Weight and buoyancy
        W_ = mass_ * g_;
        B_ = fluid_density_ * displaced_volume_ * g_;
        W_minus_B_ = W_ - B_;
    }

    void calculate_linear() {
        // Use MX for symbolic matrices. (Note: constants are automatically converted.)
        Sparsity diag_sp = Sparsity::diag(3);
        MX A11 = MX::zeros(diag_sp);
        A11(0,0) = C_X_;
        A11(1,1) = C_Y_;
        A11(2,2) = C_Z_;

        Sparsity sp_A12(3, 3, {0,0,1,2}, {2,1}, true);  // True for column-compressed
        MX A12 = MX::zeros(sp_A12);
        A12(1,2) = C_Z_q_;
        A12(2,1) = C_Y_r_;

        MX A21 = A12.T();

        MX A22 = MX::diag(MX::vertcat({MX(C_K_), MX(C_M_), MX(C_N_)}));
        
        Ma_ = MX::vertcat({MX::horzcat({A11, A12}), MX::horzcat({A21, A22})});
        
        MX I = MX::vertcat({
            MX::horzcat({MX(Lxx_), -MX(Lxy_), -MX(Lxz_)}),
            MX::horzcat({-MX(Lyx_), MX(Lyy_), -MX(Lyz_)}),
            MX::horzcat({-MX(Lzx_), -MX(Lzy_), MX(Lzz_)})
        });
        
        skew_m_   = mass_ * skew_symmetric(r_g_);
        skew_A11_ = mass_ * skew_symmetric(A11);
        skew_A22_ = mass_ * skew_symmetric(A22);
        skew_I_   = skew_symmetric(I);
        
        MX Mrb_top    = MX::horzcat({mass_ * MX::eye(3), -skew_m_});
        MX Mrb_bottom = MX::horzcat({skew_m_, I});
        MX Mrb = MX::vertcat({Mrb_top, Mrb_bottom});
        
        M_ = Mrb + Ma_;

        // Use solve to get the inverse symbolically
        M_inv_ = simplify(MX::inv(M_));
        
        MX Dl_lin = MX::diag(MX::vertcat({MX(D_u_), MX(D_v_), MX(D_w_)}));
        MX Dl_ang = MX::diag(MX::vertcat({MX(D_p_), MX(D_q_), MX(D_r_)}));
        Dl_ = MX::blockcat({{Dl_lin, MX::zeros(3,3)}, {MX::zeros(3,3), Dl_ang}});
        
        MX Dn_ang = MX::diag(MX::vertcat({MX(Dn_p_), MX(Dn_q_), MX(Dn_r_)}));
        MX Dn_lin = MX::diag(MX::vertcat({MX(Dn_u_), MX(Dn_v_), MX(Dn_w_)}));
        Dn_ = MX::blockcat({{Dn_lin, MX::zeros(3,3)}, {MX::zeros(3,3), Dn_ang}});
        
        A_ = MX::vertcat({
            MX::horzcat({1, 1, cos(a_), cos(a_), 0, 0}),
            MX::horzcat({0, 0, -sin(a_), sin(a_), 0, 0}),
            MX::horzcat({0, 0, 0, 0, 1, 1}),
            MX::horzcat({0, 0, 0, 0, lm_, -lm_}),
            MX::horzcat({0, 0, 0, 0, 0, 0}),
            MX::horzcat({-wf_, wf_, lr_*sin(a_), -lr_*sin(a_), 0, 0})
        });
    }

    // Member variables (parameters loaded as doubles; computed matrices stored as MX)
    double Ixx_, Ixy_, Ixz_, Iyx_, Iyy_, Iyz_, Izx_, Izzy_, Izz_;
    double Lxx_, Lxy_, Lxz_, Lyx_, Lyy_, Lyz_, Lzx_, Lzy_, Lzz_;
    double r_x_, r_y_, r_z_;
    double x_B_, y_B_, z_B_;
    double w_, h_, l_, lm_, wf_, lr_;
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

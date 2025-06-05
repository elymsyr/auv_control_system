#include "mpc.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <filesystem>
#include "EnvironmentMap.h"

using namespace casadi;
using json = nlohmann::json;

VehicleModel::VehicleModel(const std::string& config_path, EnvironmentMap* map)
    : map_(map) {
    load_config(config_path);
    calculate_linear();
}

MX VehicleModel::barrier_function_mx(const MX& x, const MX& y) const {
    if (!map_) return MX(1.0);  // Default safe value

    // Convert world coordinates to grid indices
    MX grid_x = (x - map_->x_r_) / map_->x_r_cm_ + map_->width / 2.0;
    MX grid_y = (y - map_->y_r_) / map_->y_r_cm_ + map_->height / 2.0;
    
    // Create interpolant function
    std::vector<double> grid_x_vals(map_->width);
    std::vector<double> grid_y_vals(map_->height);
    std::vector<double> grid_values(map_->width * map_->height);
    
    for (int i = 0; i < map_->width; i++) grid_x_vals[i] = i;
    for (int i = 0; i < map_->height; i++) grid_y_vals[i] = i;
    
    for (int iy = 0; iy < map_->height; iy++) {
        for (int ix = 0; ix < map_->width; ix++) {
            uint8_t val = map_->grid[iy * map_->width + ix];
            grid_values[iy * map_->width + ix] = (val > 150) ? -1.0 : 1.0;
        }
    }
    
    Function interpolant = casadi::interpolant("barrier_interp", "linear", 
                                             {grid_x_vals, grid_y_vals}, 
                                             grid_values);
    
    return interpolant(std::vector<MX>{grid_x, grid_y})[0];
}

MX VehicleModel::skew_symmetric(const MX& a) const {
    return MX::vertcat({
        MX::horzcat({0, -a(2), a(1)}),
        MX::horzcat({a(2), 0, -a(0)}),
        MX::horzcat({-a(1), a(0), 0})
    });
}

MX VehicleModel::transformation_matrix(const MX& eta) const {
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

MX VehicleModel::coriolis_matrix(const MX& nu) const {
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

MX VehicleModel::damping_matrix(const MX& nu) const {
    Sparsity diag6_sp = Sparsity::diag(6);
    MX Dn = MX::zeros(diag6_sp);
    for(int i=0; i<6; ++i) Dn(i,i) = fabs(nu(i)) * Dn_(i,i);
    return Dl_ + Dn;
}

MX VehicleModel::restoring_forces(const MX& eta) const {
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

std::pair<MX, MX> VehicleModel::dynamics(const MX& eta, const MX& nu, const MX& tau_p) const {
    MX J_eta   = transformation_matrix(eta);
    MX eta_dot = mtimes(J_eta, nu);

    MX C      = coriolis_matrix(nu);
    MX D      = damping_matrix(nu);
    MX g      = restoring_forces(eta);

    MX tau    = mtimes(A_, tau_p);
    MX nu_dot = simplify(mtimes(M_inv_, tau - mtimes(C, nu) - mtimes(D, nu) - g));

    return {eta_dot, nu_dot};
}

MX VehicleModel::get_A_matrix() const { return A_; }
MX VehicleModel::get_M_inv() const { return M_inv_; }
double VehicleModel::get_p_front_mid_max() const { return p_front_mid_max_; }
double VehicleModel::get_p_rear_max() const { return p_rear_max_; }

void VehicleModel::load_config(const std::string& path) {
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
    rf_ = dim["rf"].get<double>();

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

void VehicleModel::calculate_linear() {
    Sparsity diag_sp = Sparsity::diag(3);
    MX A11 = MX::zeros(diag_sp);
    A11(0,0) = C_X_;
    A11(1,1) = C_Y_;
    A11(2,2) = C_Z_;

    MX A12 = MX::zeros(3,3);
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
    M_inv_ = simplify(MX::inv(M_));

    MX Dl_lin = MX::diag(MX::vertcat({MX(D_u_), MX(D_v_), MX(D_w_)}));
    MX Dl_ang = MX::diag(MX::vertcat({MX(D_p_), MX(D_q_), MX(D_r_)}));
    Dl_ = MX::blockcat({{Dl_lin, MX::zeros(3,3)}, {MX::zeros(3,3), Dl_ang}});

    MX Dn_ang = MX::diag(MX::vertcat({MX(Dn_p_), MX(Dn_q_), MX(Dn_r_)}));
    MX Dn_lin = MX::diag(MX::vertcat({MX(Dn_u_), MX(Dn_v_), MX(Dn_w_)}));
    Dn_ = MX::blockcat({{Dn_lin, MX::zeros(3,3)}, {MX::zeros(3,3), Dn_ang}});

    // Example actuator allocation matrix (update as needed)
    A_ = MX::zeros(6,8);
    // Fill A_ as needed for your system
}

NonlinearMPC::NonlinearMPC(const VehicleModel& model, int N, double dt, MX eps)
    : model_(model), N_(N), dt_(dt), nx_(12), nu_(8), prev_sol_(std::nullopt), eps_(eps) {
    setup_optimization();
}

void NonlinearMPC::reset_previous_solution() {
    prev_sol_ = std::nullopt;
}

std::pair<DM, DM> NonlinearMPC::solve(const DM& x0, const DM& x_ref) {
    opti_.set_value(x0_param_, x0);
    opti_.set_value(x_ref_param_, x_ref);

    if (prev_sol_.has_value()) {
        DM prev_X = prev_sol_->value(X_);
        DM prev_U = prev_sol_->value(U_);
        DM X_guess = prev_X + 0.1*(x_ref - prev_X);
        DM U_guess = prev_U;
        opti_.set_initial(X_, X_guess);
        opti_.set_initial(U_, U_guess);
    } else {
        DM X_guess = DM::repmat(x0, 1, N_+1) + 0.01*DM::rand(nx_, N_+1);
        DM U_guess = 0.01*DM::rand(nu_, N_);
        opti_.set_initial(X_, X_guess);
        opti_.set_initial(U_, U_guess);
    }

    try {
        auto sol = opti_.solve();
        prev_sol_ = sol;
        return {sol.value(U_)(Slice(), 0), sol.value(X_)};
    } catch (std::exception& e) {
        std::cerr << "Solver error: " << e.what() << "\n";
        if (prev_sol_.has_value()) {
            DM fallback_U = 0.5*prev_sol_->value(U_) + 0.5*DM::zeros(nu_, N_);
            DM fallback_X = 0.5*prev_sol_->value(X_) + 0.5*DM::repmat(x0, 1, N_+1);
            return {fallback_U(Slice(), 0), fallback_X};
        }
        return {DM::zeros(nu_), DM::repmat(x0, 1, N_+1)};
    }
}

void NonlinearMPC::setup_optimization() {
    X_ = opti_.variable(nx_, N_+1);
    U_ = opti_.variable(nu_, N_);
    x0_param_ = opti_.parameter(nx_);
    x_ref_param_ = opti_.parameter(nx_, N_+1);
    
    // ZCBF parameters
    MX safety_margin = opti_.parameter();
    MX alpha = opti_.parameter();
    opti_.set_value(safety_margin, 2.0);  // 2m safety margin
    opti_.set_value(alpha, 1.0);          // ZCBF parameter
    
    // Pre-build dynamics function once (using RK4 integration)
    MX eta_sym = MX::sym("eta", 6);
    MX nu_sym   = MX::sym("nu", 6);
    MX u_sym    = MX::sym("u", 8);
    auto dyn_pair = model_.dynamics(eta_sym, nu_sym, u_sym);
    MX xdot = simplify(MX::vertcat({dyn_pair.first, dyn_pair.second}));
    
    Dict external_options;
    external_options["jit"] = true;
    external_options["compiler"] = "shell";
    external_options["jit_cleanup"] = true;
    external_options["jit_options"] = Dict{{"flags", "-O3"}};
    Function dynamics_func("dynamics_func", {eta_sym, nu_sym, u_sym}, {xdot}, external_options);

    if (!std::ifstream("libdynamics_func.so")) {
        std::cout << "Generating libdynamics_func.so...";
        // Generate derivatives explicitly
        Function jac_dynamics = dynamics_func.jacobian();
        Function grad_dynamics = dynamics_func.forward(1);
    
        // Code generation with derivative support
        Dict cg_options;
        cg_options["with_header"] = true;
        // cg_options["generate_forward"] = true;
        // cg_options["generate_reverse"] = true;
        
        CodeGenerator cg("libdynamics_func", cg_options);
        cg.add(dynamics_func);
        cg.add(jac_dynamics);
        cg.add(grad_dynamics);
        cg.generate();

        std::system ("gcc -fPIC -shared -O3 libdynamics_func.c -o libdynamics_func.so");
    }
    dynamics_func = external("dynamics_func", "./libdynamics_func.so", external_options);

    // // Barrier function (will be implemented separately)
    // MX x_sym = MX::sym("x");
    // MX y_sym = MX::sym("y");
    // Function barrier_func = Function::callback(
    //     "barrier_func",                         // Name of this callback
    //     { x_sym, y_sym },                       // Input MX symbols
    //     { MX::zeros(1,1) },                     // Dummy “template” output MX (1×1)
    //     // Now the C++ lambda that implements it:
    //     [this](const std::vector<DM>& in)->std::vector<DM> {
    //         // Call your C++ member function:
    //         float h = this->model_.barrier_function(
    //                     static_cast<float>(in[0].scalar()),
    //                     static_cast<float>(in[1].scalar()));
    //         // Return a single‐entry DM containing h:
    //         return { DM(h) };
    //     }
    // );

    MX Q = MX::diag(MX::vertcat({2.0, 2.0, 2.0, 1.0, 1.0, 1.0,
                                    0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
    MX R = MX::diag(MX(std::vector<double>(8, 0.1)));

    MX cost = 0;
    for (int k = 0; k <= N_; ++k) {
        MX state_error = X_(Slice(), k) - x_ref_param_(Slice(), k);
        cost += mtimes(mtimes(state_error.T(), Q), state_error);
        if (k < N_) {
            cost += mtimes(mtimes(U_(Slice(), k).T(), R), U_(Slice(), k));
        }
    }
    
    for (int k = 0; k < N_; ++k) {
        MX x_k = X_(Slice(), k);
        MX u_k = U_(Slice(), k);
        MX eta_k = x_k(Slice(0, 6));
        MX nu_k   = x_k(Slice(6, 12));

        // Position in world frame
        MX x_pos = eta_k(0);
        MX y_pos = eta_k(1);
        
        // // Get barrier function value
        // MX h_val = barrier_func(std::vector<MX>{x_pos, y_pos})[0];

        // MX h_val_dx = barrier_func(std::vector<MX>{x_pos + eps, y_pos})[0];
        // MX h_val_dy = barrier_func(std::vector<MX>{x_pos, y_pos + eps})[0];
        // MX h_dx = (h_val_dx - h_val) / eps;
        // MX h_dy = (h_val_dy - h_val) / eps;
        MX h_val = MX(1.0);
        MX h_dx = MX(0.0);
        MX h_dy = MX(0.0);

        if (model_.has_map()) {
            // Use the same variables but assign new values
            h_val = model_.barrier_function_mx(x_pos, y_pos);
            MX h_val_dx = model_.barrier_function_mx(x_pos + eps_, y_pos);
            MX h_val_dy = model_.barrier_function_mx(x_pos, y_pos + eps_);
            h_dx = (h_val_dx - h_val) / eps_;
            h_dy = (h_val_dy - h_val) / eps_;
        }

        // Dynamics integration (RK4)
        MX k1 = dynamics_func({eta_k, nu_k, u_k})[0];
        MX k2 = dynamics_func({eta_k + (dt_ / 2) * k1(Slice(0, 6)),
                            nu_k + (dt_ / 2) * k1(Slice(6, 12)),
                            u_k})[0];
        MX k3 = dynamics_func({eta_k + (dt_ / 2) * k2(Slice(0, 6)),
                            nu_k + (dt_ / 2) * k2(Slice(6, 12)),
                            u_k})[0];
        MX k4 = dynamics_func({eta_k + dt_ * k3(Slice(0, 6)),
                            nu_k + dt_ * k3(Slice(6, 12)),
                            u_k})[0];

        MX x_next = X_(Slice(), k) + (dt_ / 6) * vertcat(
            k1(Slice(0, 6)) + 2 * k2(Slice(0, 6)) + 2 * k3(Slice(0, 6)) + k4(Slice(0, 6)),
            k1(Slice(6, 12)) + 2 * k2(Slice(6, 12)) + 2 * k3(Slice(6, 12)) + k4(Slice(6, 12))
        );
        opti_.subject_to(X_(Slice(), k + 1) == x_next);

        // ZCBF constraint
        MX dx_dt = nu_k(0);
        MX dy_dt = nu_k(1);
        MX h_dot = h_dx*dx_dt + h_dy*dy_dt;
        opti_.subject_to(h_dot + alpha*h_val >= -safety_margin);
    }

    opti_.subject_to(X_(Slice(), 0) == x0_param_);
    double p_front = model_.get_p_front_mid_max();
    double p_rear  = model_.get_p_rear_max();
    for (int k = 0; k < N_; ++k) {
        opti_.subject_to(opti_.bounded(-p_front, U_(Slice(0, 2), k), p_front));
        opti_.subject_to(opti_.bounded(-p_rear, U_(Slice(2, 4), k), p_rear));
        opti_.subject_to(opti_.bounded(-p_front, U_(Slice(4, 8), k), p_front));
    }

    opti_.minimize(cost);
    Dict opts = {
        {"ipopt.print_level", 0},
        {"print_time", 0},
        {"ipopt.sb", "yes"},
        {"ipopt.max_iter", 1000},
        {"ipopt.tol", 1e-4},
        {"ipopt.linear_solver", "mumps"},
        {"ipopt.mu_init", 1e-2},
        {"ipopt.acceptable_tol", 1e-4},
        {"ipopt.acceptable_iter", 10},
        {"ipopt.hessian_approximation", "limited-memory"},
        {"ipopt.nlp_scaling_method", "gradient-based"}
    };
    opti_.solver("ipopt", opts);
}
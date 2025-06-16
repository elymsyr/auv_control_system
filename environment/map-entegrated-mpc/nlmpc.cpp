#include <casadi/casadi.hpp>
#include "nlmpc.h"
#include <vector>
#include <optional>
#include <fstream>
#include "vehicle_model.h"

using namespace casadi;

NonlinearMPC::NonlinearMPC(const VehicleModel& model, int N, double dt)
    : model_(model), N_(N), dt_(dt), nx_(12), nu_(8), 
      prev_sol_(std::nullopt), obstacle_weight_(8.0), 
      obstacle_epsilon_(1e-4), num_obstacles_(5), min_dist_(3.0) {
    setup_optimization();
}

std::pair<DM, DM> NonlinearMPC::solve(const DM& x0, const DM& x_ref, const DM& obs_positions) {
    opti_.set_value(x0_param_, x0);
    opti_.set_value(x_ref_param_, x_ref);
    
    // Set obstacle positions if provided
    if (obs_positions.size1() == 2 && obs_positions.size2() == num_obstacles_) {
        opti_.set_value(obs_param_, obs_positions);
    } else {
        // Default to distant obstacles if no valid positions
        DM default_obs = DM::repmat(DM(std::vector<double>{10000.0, 10000.0}), 1, num_obstacles_);
        opti_.set_value(obs_param_, default_obs);
    }

    // Improved warm-start initialization
    if (prev_sol_.has_value()) {
        DM prev_X = prev_sol_->value(X_);
        DM prev_U = prev_sol_->value(U_);
        
        // Linear interpolation between previous solution and reference
        DM X_guess = prev_X + 0.1*(x_ref - prev_X);
        DM U_guess = prev_U;
        
        opti_.set_initial(X_, X_guess);
        opti_.set_initial(U_, U_guess);
    }
    else {
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
            // Use previous solution with damped update
            DM fallback_U = 0.5*prev_sol_->value(U_) + 0.5*DM::zeros(nu_, N_);
            DM fallback_X = 0.5*prev_sol_->value(X_) + 0.5*DM::repmat(x0, 1, N_+1);
            return {fallback_U(Slice(), 0), fallback_X};
        }
        // if (!sol.stats().at("success")) {
        //     std::cout << "Solver status: " << sol.stats().at("return_status") << "\n";
        //     std::cout << "Final cost: " << sol.value(cost) << "\n";
        // }
        return {DM::zeros(nu_), DM::repmat(x0, 1, N_+1)};
    }
}

void NonlinearMPC::setup_optimization() {
    X_ = opti_.variable(nx_, N_+1);
    U_ = opti_.variable(nu_, N_);
    x0_param_ = opti_.parameter(nx_);
    x_ref_param_ = opti_.parameter(nx_, N_+1);

    obs_param_ = opti_.parameter(2, num_obstacles_);
    
    // Pre-build dynamics function once (using RK4 integration)
    MX eta_sym = MX::sym("eta", 6);
    MX nu_sym   = MX::sym("nu", 6);
    MX u_sym    = MX::sym("u", 8);
    auto dyn_pair = model_.dynamics(eta_sym, nu_sym, u_sym);
    MX xdot = simplify(MX::vertcat({dyn_pair.first, dyn_pair.second}));
    
    Dict external_options;
    // external_options["enable_fd"] = true;  // Use finite differences
    external_options["jit"] = true;  // Use finite differences
    external_options["compiler"] = "shell";  // Use finite differences
    external_options["jit_cleanup"] = true;  // Use finite differences
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

    MX Q = MX::diag(MX::vertcat({4.0, 4.0, 4.0, 1.0, 1.5, 3.0,
                                    0.0, 0.0, 0.0, 0.1, 0.1, 0.1}));
    MX R = MX::diag(MX(std::vector<double>(8, 0.2)));

    MX cost = 0;
    for (int k = 0; k <= N_; ++k) {
        MX state_error = X_(Slice(), k) - x_ref_param_(Slice(), k);
        cost += mtimes(mtimes(state_error.T(), Q), state_error);
        
        if (k > 0 && k < N_) {
            MX pos = X_(Slice(0, 2), k);
            cost += obstacle_weight_ * barrier_function(pos, obs_param_);
        }

        if (k < N_) {
            cost += mtimes(mtimes(U_(Slice(), k).T(), R), U_(Slice(), k));
        }
    }

    // for (int k = 0; k <= N_; ++k) {
    //     MX pos = X_(Slice(0, 2), k);
    //     avoidance_constraints(pos, obs_param_);
    // }
    
    for (int k = 0; k < N_; ++k) {
        MX x_k = X_(Slice(), k);
        MX u_k = U_(Slice(), k);
        MX eta_k = x_k(Slice(0, 6));
        MX nu_k   = x_k(Slice(6, 12));
        
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
    // Dict opts = {
    //     {"ipopt.print_level", 0},
    //     {"print_time", 0},
    //     {"ipopt.sb", "yes"},
    //     {"ipopt.max_iter", 1000},  // Increased from 400
    //     {"ipopt.tol", 1e-4},       // Loosened from 1e-5
    //     {"ipopt.linear_solver", "mumps"},
    //     {"ipopt.mu_init", 1e-2},
    //     {"ipopt.acceptable_tol", 1e-4},  // Loosened from 1e-5
    //     {"ipopt.hessian_approximation", "limited-memory"},
    //     {"ipopt.nlp_scaling_method", "gradient-based"}
    // };
    Dict opts = {
        {"ipopt.print_level", 2},
        {"print_time", 0},
        {"ipopt.sb", "yes"},
        {"ipopt.max_iter", 2000},             // Increased iterations
        {"ipopt.tol", 1e-4},                  // Looser tolerance
        {"ipopt.linear_solver", "mumps"},     // Better for nonlinear problems
        {"ipopt.mu_init", 1e-2},              // Smaller initial barrier
        {"ipopt.acceptable_tol", 1e-5},       // Looser acceptance tol
        // {"ipopt.constr_viol_tol", 1e-5},   // Constraint violation tol
        {"ipopt.hessian_approximation", "limited-memory"},
        {"ipopt.nlp_scaling_method", "gradient-based"},
    };
    opti_.solver("ipopt", opts);
}

MX NonlinearMPC::barrier_function(const MX& position, const MX& obstacles) {
    MX total_penalty = 0;
    MX x = position(0);
    MX y = position(1);
    
    for (int i = 0; i < num_obstacles_; ++i) {
        MX obs_x = obstacles(0, i);
        MX obs_y = obstacles(1, i);
        
        MX dx = x - obs_x;
        MX dy = y - obs_y;
        MX dist_sq = dx*dx + dy*dy;
        MX dist = sqrt(dist_sq + obstacle_epsilon_);
        
        // Quadratic-quadratic barrier function
        MX delta = min_dist_ - dist;
        
        // Only penalize when closer than safety distance
        total_penalty += if_else(delta > 0, 
                                 obstacle_weight_ * delta*delta / (dist_sq + obstacle_epsilon_), 
                                 0);
    }
    return total_penalty;
}

void NonlinearMPC::set_obstacle_weight(double weight) {
    obstacle_weight_ = weight;
}

void NonlinearMPC::set_obstacle_epsilon(double epsilon) {
    obstacle_epsilon_ = epsilon;
}

void NonlinearMPC::avoidance_constraints(const MX& position, const MX& obstacles) {
    MX x = position(0);
    MX y = position(1);
    
    for (int i = 0; i < num_obstacles_; ++i) {
        MX obs_x = obstacles(0, i);
        MX obs_y = obstacles(1, i);
        
        MX dx = x - obs_x;
        MX dy = y - obs_y;
        MX dist_sq = dx*dx + dy*dy;
        
        // Safety distance including vehicle size
        MX safety_dist = min_dist_;
        MX min_dist_sq = safety_dist * safety_dist;
        
        // Add constraint: distance^2 >= min_dist^2
        opti_.subject_to(dist_sq >= min_dist_sq);
    }
}

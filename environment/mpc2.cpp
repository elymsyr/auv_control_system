// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -L"${CONDA_PREFIX}/lib"     -Wl,-rpath,"${CONDA_PREFIX}/lib" mpc.cpp -lcasadi -lipopt -lzmq -o mpc_test

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
#include <cstring> // for memcpy

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

class NonlinearMPC {
    public:
        NonlinearMPC(const VehicleModel& model, int N = 10, double dt = 0.1)
            : model_(model), N_(N), dt_(dt), nx_(12), nu_(6), prev_sol_(std::nullopt) {
            setup_optimization();
        }
        
        std::pair<DM, DM> solve(const DM& x0, const DM& x_ref) {
            opti_.set_value(x0_param_, x0);
            opti_.set_value(x_ref_param_, x_ref);
            
            if (prev_sol_.has_value()) {
                DM prev_X = prev_sol_->value(X_);
                DM prev_U = prev_sol_->value(U_);
                
                // Create sparse shift operation
                DM X_guess = horzcat(prev_X(Slice(), Slice(1, N_+1)), 
                                    prev_X(Slice(), N_));
                DM U_guess = horzcat(prev_U(Slice(), Slice(1, N_)), 
                                    prev_U(Slice(), N_-1));
                
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
                    return {prev_sol_->value(U_)(Slice(), 0), prev_sol_->value(X_)};
                }
                return {DM::zeros(nu_), DM::zeros(nx_, N_+1)};
            }
        }
        
    private:
        VehicleModel model_;
        int N_, nx_, nu_;
        double dt_;
        Opti opti_;
        MX X_, U_;
        MX x0_param_, x_ref_param_;
        std::optional<OptiSol> prev_sol_;
        
        void setup_optimization() {
            X_ = opti_.variable(nx_, N_+1);
            U_ = opti_.variable(nu_, N_);
            x0_param_ = opti_.parameter(nx_);
            x_ref_param_ = opti_.parameter(nx_, N_+1);
            
            // Pre-build dynamics function once (using RK4 integration)
            MX eta_sym = MX::sym("eta", 6);
            MX nu_sym   = MX::sym("nu", 6);
            MX u_sym    = MX::sym("u", 6);
            auto dyn_pair = model_.dynamics(eta_sym, nu_sym, u_sym);
            MX xdot = simplify(MX::vertcat({dyn_pair.first, dyn_pair.second}));
            
            Dict external_options;
            // external_options["enable_fd"] = true;  // Use finite differences
            external_options["jit"] = true;  // Use finite differences
            external_options["compiler"] = "shell";  // Use finite differences
            external_options["jit_cleanup"] = true;  // Use finite differences
            external_options["jit_options"] = Dict{{"flags", "-O3"}};
            Function dynamics_func("dynamics_func", {eta_sym, nu_sym, u_sym}, {xdot}, external_options);

            if (!std::ifstream("libdynamics_func2.so")) {
                std::cout << "Generating libdynamics_func2.so...";
                // Generate derivatives explicitly
                Function jac_dynamics = dynamics_func.jacobian();
                Function grad_dynamics = dynamics_func.forward(1);
            
                // Code generation with derivative support
                Dict cg_options;
                cg_options["with_header"] = true;
                // cg_options["generate_forward"] = true;
                // cg_options["generate_reverse"] = true;
                
                CodeGenerator cg("libdynamics_func2", cg_options);
                cg.add(dynamics_func);
                cg.add(jac_dynamics);
                cg.add(grad_dynamics);
                cg.generate();

                std::system ("gcc -fPIC -shared -O3 libdynamics_func2.c -o libdynamics_func2.so");
            }
            dynamics_func = external("dynamics_func", "./libdynamics_func2.so", external_options);

            MX Q = MX::diag(MX::vertcat({1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
            MX R = MX::diag(MX(std::vector<double>(6, 0.001)));
            
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
                opti_.subject_to(opti_.bounded(-p_front, U_(Slice(4, 6), k), p_front));
            }
            
            opti_.minimize(cost);
            Dict opts = {
                {"ipopt.print_level", 0},
                {"print_time", 0},
                {"ipopt.sb", "yes"},
                {"ipopt.max_iter", 1000},
                {"ipopt.tol", 1e-5},
                {"ipopt.linear_solver", "mumps"},
                {"ipopt.mu_init", 1e-2},
                {"ipopt.acceptable_tol", 1e-5},
                {"ipopt.hessian_approximation", "limited-memory"},
                {"ipopt.nlp_scaling_method", "gradient-based"}
            };
            opti_.solver("ipopt", opts);
        }
    };

    int main() {
        try {
            // Initialize your model and MPC controller.
            VehicleModel model("config.json");
            NonlinearMPC mpc(model);
            
            // Initial state x0 and reference trajectory x_ref (as DM)
            DM x0 = DM::zeros(12);
            DM x_ref = DM::repmat(DM::vertcat({20, 15, -6, 0, 0, -1, 0, 0, 0, 0, 0, 0}), 1, 11);
    
            // // Set up ZeroMQ context and a REQ socket to communicate with Python.
            // zmq::context_t context(1);
            // zmq::socket_t socket(context, zmq::socket_type::req);
            // socket.connect("tcp://localhost:5555");
    
            // // --- Send initial state data as binary ---
            // std::vector<double> init_state(12);
            // for (int i = 0; i < 12; ++i) {
            //     init_state[i] = static_cast<double>(x0(i));
            // }
            // zmq::message_t init_msg(init_state.size() * sizeof(double));
            // memcpy(init_msg.data(), init_state.data(), init_msg.size());
            // (void)socket.send(init_msg, zmq::send_flags::none);
            
            // // Wait for a short acknowledgment from Python (if your protocol requires it)
            // zmq::message_t ack;
            // (void)socket.recv(ack, zmq::recv_flags::none);
    
            // Start overall timing
            auto total_start = std::chrono::high_resolution_clock::now();
            int ignore_first = 10;
            std::vector<double> iteration_times;
            // Main control loop (100 steps)
            for (int step = 0; step < 100; ++step) {
                // Solve MPC and time the iteration
                auto iter_start = std::chrono::high_resolution_clock::now();
                auto [u_opt, x_opt] = mpc.solve(x0, x_ref);
                auto iter_end = std::chrono::high_resolution_clock::now();
                auto iter_duration = std::chrono::duration_cast<std::chrono::milliseconds>(iter_end - iter_start).count();
                
                // Update state (for next iteration)
                x0 = x_opt(Slice(), 1);
                
                std::cout << "Step " << step << "\n"
                          << "  controls: " << u_opt << "\n"
                          << "  state: " << x0 << "\n"
                          << "  Iteration time: " << iter_duration << " ms\n";

                if (step >= ignore_first) {
                    iteration_times.push_back(iter_duration);
                }
                
                // // --- Prepare and send the new state as binary ---
                // std::vector<double> state_vec(12);
                // for (int i = 0; i < 12; ++i) {
                //     state_vec[i] = static_cast<double>(x0(i));
                // }
                // zmq::message_t state_msg(state_vec.size() * sizeof(double));
                // memcpy(state_msg.data(), state_vec.data(), state_msg.size());
                
                // // Send the binary state data to Python.
                // (void)socket.send(state_msg, zmq::send_flags::none);
                
                // // Wait for Python's reply before continuing (this is required for REQ/REP pattern).
                // zmq::message_t reply;
                // (void)socket.recv(reply, zmq::recv_flags::none);
            }
            
            auto total_end = std::chrono::high_resolution_clock::now();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
            std::cout << "Total runtime for 100 steps: " << total_duration << " ms\n";
            std::cout << "Average iteration time (excluding first 10): " 
                      << std::accumulate(iteration_times.begin(), iteration_times.end(), 0.0) / iteration_times.size() << " ms\n";
            double sum = std::accumulate(iteration_times.begin(), iteration_times.end(), 0.0);
            double mean = sum / iteration_times.size();
            double sq_sum = std::accumulate(iteration_times.begin(), iteration_times.end(), 0.0, 
                                            [mean](double acc, double val) { return acc + (val - mean) * (val - mean); });
            double stddev = std::sqrt(sq_sum / iteration_times.size());
            std::cout << "Standard deviation of iteration times (excluding first 10): " << stddev << " ms\n";
            
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            return 1;
        }
        return 0;
    }
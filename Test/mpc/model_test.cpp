#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

using json = nlohmann::json;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using DiagonalMatrix6d = Eigen::DiagonalMatrix<double, 6>;

class VehicleModel {
public:
    explicit VehicleModel(const std::string& config_path) {
        load_config(config_path);
        calculate_linear();
    }

    // Compute skew-symmetric matrix for a 3D vector
    Matrix3d skew_symmetric(const Vector3d& a) const {
        Matrix3d S;
        S <<     0,  -a.z(),   a.y(),
              a.z(),      0,  -a.x(),
             -a.y(),   a.x(),      0;
        return S;
    }

    // Compute transformation matrix from state vector eta
    Matrix6d transformation_matrix(const Vector6d& eta) const {
        const double phi = eta(3);
        const double theta = eta(4);
        const double psi = eta(5);

        // Precompute trigonometric terms
        const double cphi = std::cos(phi), sphi = std::sin(phi);
        const double ctheta = std::cos(theta), stheta = std::sin(theta);
        const double cpsi = std::cos(psi), spsi = std::sin(psi);

        // Construct rotation matrix (Z-Y-X convention)
        Matrix3d R;
        R << cpsi * ctheta, -spsi * cphi + cpsi * stheta * sphi,  spsi * sphi + cpsi * cphi * stheta,
             spsi * ctheta,  cpsi * cphi + spsi * stheta * sphi, -cpsi * sphi + spsi * stheta * cphi,
             -stheta,        ctheta * sphi,                       ctheta * cphi;

        // Transformation matrix for angular velocities
        const double ttheta = stheta / ctheta; // tan(theta)
        const double sectheta = 1.0 / ctheta;
        Matrix3d T;
        T << 1,   sphi * ttheta,    cphi * ttheta,
             0,        cphi,           -sphi,
             0, sphi * sectheta, cphi * sectheta;

        Matrix6d J;
        J << R, Matrix3d::Zero(),
             Matrix3d::Zero(), T;
        return J;
    }

    // Compute Coriolis matrix
    Matrix6d coriolis_matrix(const Vector6d& nu) const {
        const Vector3d nu1 = nu.head<3>();
        const Vector3d nu2 = nu.tail<3>();
        const Matrix3d skew_nu1 = skew_symmetric(nu1);
        const Matrix3d skew_nu2 = skew_symmetric(nu2);
        
        Matrix6d Crb = Matrix6d::Zero();
        Crb.block<3,3>(0,3) = -mass_ * skew_nu1 - skew_nu2 * skew_m_;
        Crb.block<3,3>(3,0) = -mass_ * skew_nu1 + skew_nu2 * skew_m_;
        Crb.block<3,3>(3,3) = - skew_nu2 * skew_I_;
        
        Matrix6d Ca = Matrix6d::Zero();
        Ca.block<3,3>(0,3) = - skew_A11_ * skew_nu1;
        Ca.block<3,3>(3,3) = - skew_A22_ * skew_nu2;
        
        return Crb + Ca;
    }

    // Compute damping matrix
    DiagonalMatrix6d damping_matrix(const Vector6d& nu) const {
        const Vector6d Dn_dynamic = Dn_.diagonal().cwiseProduct(nu.cwiseAbs());
        return DiagonalMatrix6d(Dl_.diagonal() + Dn_dynamic);
    }

    // Compute restoring forces
    Vector6d restoring_forces(const Vector6d& eta) const {
        const double phi = eta(3);
        const double theta = eta(4);
        const double sphi = std::sin(phi), cphi = std::cos(phi);
        const double stheta = std::sin(theta), ctheta = std::cos(theta);

        const double term1 = r_y_ * W_ - y_B_ * B_;
        const double term2 = r_z_ * W_ - z_B_ * B_;
        const double term3 = r_x_ * W_ - x_B_ * B_;

        Vector6d g_eta;
        g_eta << W_minus_B_ * stheta,
                -W_minus_B_ * ctheta * sphi,
                -W_minus_B_ * ctheta * cphi,
                -term1 * ctheta * cphi + term2 * ctheta * sphi,
                 term2 * stheta + term3 * ctheta * cphi,
                -term3 * ctheta * sphi - term1 * stheta;
        return g_eta;
    }

    // Compute dynamics (eta_dot, nu_dot)
    std::pair<Vector6d, Vector6d> dynamics(const Vector6d& eta,
                                           const Vector6d& nu,
                                           const Vector6d& tau_p) const {
        const Matrix6d J_eta = transformation_matrix(eta);
        const Vector6d eta_dot = J_eta * nu;
        
        const Matrix6d C = coriolis_matrix(nu);
        const DiagonalMatrix6d D = damping_matrix(nu);
        const Vector6d g = restoring_forces(eta);
        
        const Vector6d tau = A_ * tau_p;
        const Vector6d nu_dot = M_inv_ * (tau - C * nu - D * nu - g);
        
        return {eta_dot, nu_dot};
    }

    // Accessors
    const Matrix6d& get_A_matrix() const { return A_; }
    const Matrix6d& get_M_inv() const { return M_inv_; }
    double get_p_front_mid_max() const { return p_front_mid_max_; }
    double get_p_rear_max() const { return p_rear_max_; }

private:
    void load_config(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            throw std::runtime_error("Cannot open configuration file.");
        }
        json config = json::parse(f)["assembly_mass_properties"];

        // Inertia parameters
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

        // Moments of inertia about center of mass
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

        // Center of mass location
        auto com = config["center_of_mass"];
        r_x_ = com["X"].get<double>();
        r_y_ = com["Y"].get<double>();
        r_z_ = com["Z"].get<double>();

        // Center of buoyancy
        auto buoyancy = config["center_of_buoancy"];
        x_B_ = buoyancy["X"].get<double>();
        y_B_ = buoyancy["Y"].get<double>();
        z_B_ = buoyancy["Z"].get<double>();

        // Dimensions
        auto dim = config["dimensions"];
        w_ = dim["width"].get<double>();
        h_ = dim["height"].get<double>();
        l_ = dim["length"].get<double>();
        lm_ = dim["lm"].get<double>();
        wf_ = dim["wf"].get<double>();
        lr_ = dim["lr"].get<double>();

        // Mass and other parameters
        mass_ = config["mass"]["value"].get<double>();
        double a_deg = config["rear_propeller_angle"]["value"].get<double>();
        a_ = a_deg * M_PI / 180.0;
        volume_ = config["volume"]["value"].get<double>();

        C_X_   = config["added_mass"]["C_X"].get<double>();
        C_Y_   = config["added_mass"]["C_Y"].get<double>();
        C_Z_   = config["added_mass"]["C_Z"].get<double>();
        C_Y_r_ = config["added_mass"]["C_Y_r"].get<double>();
        C_Z_q_ = config["added_mass"]["C_Z_q"].get<double>();
        C_K_   = config["added_mass"]["C_K"].get<double>();
        C_M_   = config["added_mass"]["C_M"].get<double>();
        C_N_   = config["added_mass"]["C_N"].get<double>();

        // Dynamics parameters
        auto dynamics = config["dynamics"];
        fluid_density_ = dynamics["fluid_density"]["value"].get<double>();
        displaced_volume_ = dynamics["displaced_volume"]["value"].get<double>();
        g_ = dynamics["g"]["value"].get<double>();

        // Linear damping coefficients
        auto damping = dynamics["damping"];
        D_u_ = damping["linear"]["D_u"].get<double>();
        D_v_ = damping["linear"]["D_v"].get<double>();
        D_w_ = damping["linear"]["D_w"].get<double>();
        D_p_ = damping["angular"]["D_p"].get<double>();
        D_q_ = damping["angular"]["D_q"].get<double>();
        D_r_ = damping["angular"]["D_r"].get<double>();
        
        // Nonlinear damping coefficients
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
        // Added mass matrices setup
        Matrix3d A11 = Vector3d(C_X_, C_Y_, C_Z_).asDiagonal();
        Matrix3d A12 = Matrix3d::Zero();
        A12(1,2) = C_Z_q_;
        A12(2,1) = C_Y_r_;

        Matrix3d A21 = A12.transpose();
        Matrix3d A22 = Vector3d(C_K_, C_M_, C_N_).asDiagonal();

        Matrix6d Ma;
        Ma << A11, A12,
              A21, A22;

        // Rigid-body inertia matrix
        Matrix3d I;
        I << Lxx_, -Lxy_, -Lxz_,
            -Lyx_, Lyy_, -Lyz_,
            -Lzx_, -Lzy_, Lzz_;

        const Vector3d r_g(r_x_, r_y_, r_z_);
        skew_m_ = mass_ * skew_symmetric(r_g);
        skew_A11_ = skew_m_;
        skew_I_ = skew_symmetric(I.diagonal());

        // Rigid-body mass matrix
        Matrix6d Mrb = Matrix6d::Zero();
        Mrb.block<3,3>(0,0) = mass_ * Matrix3d::Identity();
        Mrb.block<3,3>(0,3) = -skew_m_;
        Mrb.block<3,3>(3,0) =  skew_m_;
        Mrb.block<3,3>(3,3) = I;

        M_ = Mrb + Ma;
        M_inv_ = M_.inverse();

        // Damping matrices as diagonal
        Dl_ = (Vector6d() << D_u_, D_v_, D_w_, D_p_, D_q_, D_r_).finished().asDiagonal();
        Dn_ = (Vector6d() << Dn_u_, Dn_v_, Dn_w_, Dn_p_, Dn_q_, Dn_r_).finished().asDiagonal();

        // Control matrix A
        A_.setZero();
        A_.row(0) << 1, 1, std::cos(a_), std::cos(a_), 0, 0;
        A_.row(1) << 0, 0, -std::sin(a_), std::sin(a_), 0, 0;
        A_.row(2) << 0, 0, 0, 0, 1, 1;
        A_.row(3) << 0, 0, 0, 0, lm_, -lm_;
        A_.row(5) << -wf_, wf_, lr_ * std::sin(a_), -lr_ * std::sin(a_), 0, 0;
    }

    // Member variables
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

    // Precomputed matrices
    Matrix6d A_;      // Control mapping matrix
    Matrix6d M_;      // Total mass matrix
    Matrix6d M_inv_;  // Inverse of M
    DiagonalMatrix6d Dl_;  // Linear damping matrix
    DiagonalMatrix6d Dn_;  // Nonlinear damping matrix
    Matrix3d skew_m_, skew_I_, skew_A11_, skew_A22_;
};

// Test function to simulate dynamics for 10 iterations
void test_dynamics() {
    // Example JSON config (replace with actual file path)
    std::string config_path = "config.json";  // Ensure this file exists

    VehicleModel model(config_path);

    // Initial state (position, orientation, velocity, angular velocity)
    Vector6d eta = Vector6d::Zero();  // [x, y, z, phi, theta, psi]
    Vector6d nu = Vector6d::Zero();   // [u, v, w, p, q, r]
    Vector6d tau_p = Vector6d::Zero(); // Control forces

    // Simulate for 10 iterations
    const double dt = 0.1;  // Time step (s)
    for (int i = 0; i < 10; ++i) {
        auto [eta_dot, nu_dot] = model.dynamics(eta, nu, tau_p);
        std::cout << eta_dot .transpose() << "\n";
        std::cout << nu_dot .transpose() << "\n";

        // Simple Euler integration
        eta += eta_dot * dt;
        nu += nu_dot * dt;

        // Print state
        std::cout << "Iteration " << i + 1 << ":\n";
        std::cout << "Position: " << eta.head<3>().transpose() << "\n";
        std::cout << "Orientation (RPY): " << eta.tail<3>().transpose() << "\n";
        std::cout << "Linear Velocity: " << nu.head<3>().transpose() << "\n";
        std::cout << "Angular Velocity: " << nu.tail<3>().transpose() << "\n\n";
    }
}

int main() {
    try {
        test_dynamics();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
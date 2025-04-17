import casadi as ca
import json
from render import Render
import numpy as np
from time import perf_counter
import os

class CasadiModel:
    def __init__(self, config_path: str = "rl_test/config.json"):
        """
        Initialize the motion model using CasADi symbols.
        """
        self.load_dynamics(config_path)
        self.calculate_linear()
        
    def load_dynamics(self, config_path):
        """
        Load dynamics data from the config file and convert to CasADi DM.
        """
        with open(config_path, 'r') as file:
            config = json.load(file)

        config = config["assembly_mass_properties"]

        # Load moments of inertia about the output coordinate system
        moments = config["moments_of_inertia_about_output_coordinate_system"]
        self.Ixx = ca.DM(moments["Ixx"])
        self.Ixy = ca.DM(moments["Ixy"])
        self.Ixz = ca.DM(moments["Ixz"])
        self.Iyx = ca.DM(moments["Iyx"])
        self.Iyy = ca.DM(moments["Iyy"])
        self.Iyz = ca.DM(moments["Iyz"])
        self.Izx = ca.DM(moments["Izx"])
        self.Izzy = ca.DM(moments["Izy"])
        self.Izz = ca.DM(moments["Izz"])

        # Load moments of inertia about center of mass
        moments_com = config["moments_of_inertia_about_center_of_mass"]
        self.Lxx = ca.DM(moments_com["Lxx"])
        self.Lxy = ca.DM(moments_com["Lxy"])
        self.Lxz = ca.DM(moments_com["Lxz"])
        self.Lyx = ca.DM(moments_com["Lyx"])
        self.Lyy = ca.DM(moments_com["Lyy"])
        self.Lyz = ca.DM(moments_com["Lyz"])
        self.Lzx = ca.DM(moments_com["Lzx"])
        self.Lzy = ca.DM(moments_com["Lzy"])
        self.Lzz = ca.DM(moments_com["Lzz"])

        # Load center of mass
        com = config["center_of_mass"]
        self.r_x = ca.DM(com["X"])
        self.r_y = ca.DM(com["Y"])
        self.r_z = ca.DM(com["Z"])
        self.r_g = ca.vertcat(self.r_x, self.r_y, self.r_z)

        # Center of buoyancy
        buoyancy = config["center_of_buoancy"]
        self.x_B = ca.DM(buoyancy["X"])
        self.y_B = ca.DM(buoyancy["Y"])
        self.z_B = ca.DM(buoyancy["Z"])
        self.r_B = ca.vertcat(self.x_B, self.y_B, self.z_B)

        # Dimensions
        dim = config["dimensions"]
        self.w = ca.DM(dim["width"])
        self.h = ca.DM(dim["height"])
        self.l = ca.DM(dim["length"])
        self.lm = ca.DM(dim["lm"])
        self.wf = ca.DM(dim["wf"])
        self.lr = ca.DM(dim["lr"])

        # Mass and parameters
        self.mass = ca.DM(config["mass"]["value"])
        self.a = ca.DM(config["rear_propeller_angle"]["value"] * ca.pi / 180)
        self.volume = ca.DM(config["volume"]["value"])

        self.skew_m = self.mass * self.skew_symmetric(self.r_g)

        self.added_mass_coef = {
            "C_X": ca.DM(0.1),
            "C_Y": ca.DM(0.2),
            "C_Z": ca.DM(0.2),
            "C_K": ca.DM(0.05),
            "C_M": ca.DM(0.05),
            "C_N": ca.DM(0.05),
            "C_Y_r": ca.DM(0.0),
            "C_Z_q": ca.DM(0.0) 
        }


        # Dynamics parameters
        dynamics = config["dynamics"]
        self.fluid_density = ca.DM(dynamics["fluid_density"]["value"])
        self.displaced_volume = ca.DM(dynamics["displaced_volume"]["value"])
        self.g = ca.DM(dynamics["g"]["value"])

        # Damping coefficients
        self.D_u = ca.DM(dynamics["damping"]["linear"]["D_u"])
        self.D_v = ca.DM(dynamics["damping"]["linear"]["D_v"])
        self.D_w = ca.DM(dynamics["damping"]["linear"]["D_w"])
        self.D_p = ca.DM(dynamics["damping"]["angular"]["D_p"])
        self.D_q = ca.DM(dynamics["damping"]["angular"]["D_q"])
        self.D_r = ca.DM(dynamics["damping"]["angular"]["D_r"])
        self.Dn_u = ca.DM(dynamics["damping"]["linear_n"]["Dn_u"])
        self.Dn_v = ca.DM(dynamics["damping"]["linear_n"]["Dn_v"])
        self.Dn_w = ca.DM(dynamics["damping"]["linear_n"]["Dn_w"])
        self.Dn_p = ca.DM(dynamics["damping"]["angular_n"]["Dn_p"])
        self.Dn_q = ca.DM(dynamics["damping"]["angular_n"]["Dn_q"])
        self.Dn_r = ca.DM(dynamics["damping"]["angular_n"]["Dn_r"])

        # Propeller parameters
        self.p_rear_max = ca.DM(dynamics["propeller"]["force_range_r"]["max"])
        self.p_front_mid_max = ca.DM(dynamics["propeller"]["force_range_f_m"]["max"])
        self.p_change_matrix_high = ca.DM(dynamics["propeller"]["change_matrix_high"])
        self.p_change_matrix_low = ca.DM(dynamics["propeller"]["change_matrix_low"])
        self.p_change_matrix_dt = ca.DM(dynamics["propeller"]["change_matrix_dt"])
        self.p_torque_coefficient = ca.DM(dynamics["propeller"]["torque_coefficient"])
        self.p_thrust_coefficient = ca.DM(dynamics["propeller"]["thrust_coefficient"])

        # Weight and buoyancy
        self.W = self.mass * self.g
        self.B = self.fluid_density * self.displaced_volume * self.g
        self.W_minus_B = self.W - self.B

        self.A = ca.vertcat(
            ca.horzcat(1, 1, ca.cos(self.a), ca.cos(self.a), 0, 0),  # Row 1
            ca.horzcat(0, 0, -ca.sin(self.a), ca.sin(self.a), 0, 0),  # Row 2
            ca.horzcat(0, 0, 0, 0, 1, 1),  # Row 3
            ca.horzcat(0, 0, 0, 0, self.lm, -self.lm),  # Row 4
            ca.horzcat(0, 0, 0, 0, 0, 0),  # Row 5
            ca.horzcat(-self.wf, self.wf, self.lr * ca.sin(self.a), -self.lr * ca.sin(self.a), 0, 0)  # Row 6
        )

    def skew_symmetric(self, a):
        """CasADi implementation of skew-symmetric matrix."""
        return ca.vertcat(
            ca.horzcat(0, -a[2], a[1]),
            ca.horzcat(a[2], 0, -a[0]),
            ca.horzcat(-a[1], a[0], 0)
        )

    def calculate_linear(self):
        """Precompute mass, added mass, and damping matrices."""
        self._added_mass()
        self._mass_inertia_matrix()
        self._damping_matrix()

    def _added_mass(self):
        """Added mass matrix (6x6)."""
        self.A11 = ca.diag(ca.vertcat(
            -self.added_mass_coef["C_X"],
            -self.added_mass_coef["C_Y"],
            -self.added_mass_coef["C_Z"]
        ))
        self.A12 = ca.DM.zeros(3, 3)
        self.A12[1, 2] = -self.added_mass_coef["C_Y_r"]
        self.A12[2, 1] = -self.added_mass_coef["C_Z_q"]
        self.A21 = self.A12.T
        self.A22 = ca.diag(ca.vertcat(
            -self.added_mass_coef["C_K"],
            -self.added_mass_coef["C_M"],
            -self.added_mass_coef["C_N"]
        ))
        self.Ma = ca.vertcat(
            ca.horzcat(self.A11, self.A12),
            ca.horzcat(self.A21, self.A22)
        )
        
        self.skew_A11 = self.skew_symmetric(self.A11)
        self.skew_A12 = self.skew_symmetric(self.A12)
        self.skew_A21 = self.skew_symmetric(self.A21)
        self.skew_A22 = self.skew_symmetric(self.A22)

    def _mass_inertia_matrix(self):
        """Mass-inertia matrix (6x6)."""
        self.I = ca.vertcat(
            ca.horzcat(self.Lxx, -self.Lxy, -self.Lxz),
            ca.horzcat(-self.Lyx, self.Lyy, -self.Lyz),
            ca.horzcat(-self.Lzx, -self.Lzy, self.Lzz)
        )
        
        self.skew_I = self.skew_symmetric(self.I)
        
        Mrb_top = ca.horzcat(self.mass * ca.DM.eye(3), -self.skew_m)
        Mrb_bottom = ca.horzcat(self.skew_m, self.I)
        self.Mrb = ca.vertcat(Mrb_top, Mrb_bottom)
        self.M = self.Mrb + self.Ma
        self.M_inv = ca.inv(self.M)

    def _damping_matrix(self):
        """Linear damping matrix Dl and nonlinear Dn."""
        Dl_lin = ca.diag(ca.vertcat(self.D_u, self.D_v, self.D_w))
        Dl_ang = ca.diag(ca.vertcat(self.D_p, self.D_q, self.D_r))
        self.Dl = ca.vertcat(
            ca.horzcat(Dl_lin, ca.DM.zeros(3, 3)),
            ca.horzcat(ca.DM.zeros(3, 3), Dl_ang)
        )
        Dn_lin = ca.diag(ca.vertcat(self.Dn_u, self.Dn_v, self.Dn_w))
        Dn_ang = ca.diag(ca.vertcat(self.Dn_p, self.Dn_q, self.Dn_r))
        self.Dn = ca.vertcat(
            ca.horzcat(Dn_lin, ca.DM.zeros(3, 3)),
            ca.horzcat(ca.DM.zeros(3, 3), Dn_ang)
        )

    def damping_matrix(self, nu):
        """Damping matrix D(nu)."""
        Dn = self.Dn @ ca.diag(ca.fabs(nu))
        return Dn + self.Dl

    def coriolis_centripetal_matrix(self, nu):
        """Coriolis and centripetal matrix C(nu)."""
        nu1 = nu[:3]
        nu2 = nu[3:]
        skew_nu1 = self.skew_symmetric(nu1)
        skew_nu2 = self.skew_symmetric(nu2)
        Crb_top = ca.horzcat(ca.DM.zeros(3,3), -self.mass*skew_nu1 - skew_nu2@self.skew_m)
        Crb_bottom = ca.horzcat(-self.mass*skew_nu1 + skew_nu2@self.skew_m, -skew_nu2@self.skew_I)
        Crb = ca.vertcat(Crb_top, Crb_bottom)
        Ca_top = ca.horzcat(ca.DM.zeros(3,3), -self.skew_A11@skew_nu1)
        Ca_bottom = ca.horzcat(-self.skew_A11@skew_nu1, -self.skew_A22@skew_nu2)
        Ca = ca.vertcat(Ca_top, Ca_bottom)
        return Crb + Ca

    def restoring_forces(self, eta):
        """Restoring forces and moments g(eta)."""
        phi, theta, _ = eta[3], eta[4], eta[5]
        s_phi, c_phi = ca.sin(phi), ca.cos(phi)
        s_theta, c_theta = ca.sin(theta), ca.cos(theta)
        g_eta = ca.vertcat(
            self.W_minus_B * s_theta,
            -self.W_minus_B * c_theta * s_phi,
            -self.W_minus_B * c_theta * c_phi,
            -(self.r_y*self.W - self.y_B*self.B)*c_theta*c_phi + (self.r_z*self.W - self.z_B*self.B)*c_theta*s_phi,
            (self.r_z*self.W - self.z_B*self.B)*s_theta + (self.r_x*self.W - self.x_B*self.B)*c_theta*c_phi,
            -(self.r_x*self.W - self.x_B*self.B)*c_theta*s_phi - (self.r_y*self.W - self.y_B*self.B)*s_theta
        )
        return g_eta

    def J(self, eta):
        """Transformation matrix J(eta)."""
        phi, theta, psi = eta[3], eta[4], eta[5]
        cphi, sphi = ca.cos(phi), ca.sin(phi)
        ctheta, stheta = ca.cos(theta), ca.sin(theta)
        cpsi, spsi = ca.cos(psi), ca.sin(psi)
        R = ca.vertcat(
            ca.horzcat(cpsi*ctheta, -spsi*cphi + cpsi*stheta*sphi, spsi*sphi + cpsi*cphi*stheta),
            ca.horzcat(spsi*ctheta, cpsi*cphi + spsi*stheta*sphi, -cpsi*sphi + spsi*stheta*cphi),
            ca.horzcat(-stheta, ctheta*sphi, ctheta*cphi)
        )
        T = ca.vertcat(
            ca.horzcat(1, sphi*ca.tan(theta), cphi*ca.tan(theta)),
            ca.horzcat(0, cphi, -sphi),
            ca.horzcat(0, sphi/ctheta, cphi/ctheta)
        )
        return ca.blockcat(R, ca.DM.zeros(3,3), ca.DM.zeros(3,3), T)

    def dynamics(self, eta, nu, tau_p):
        """Continuous-time dynamics: eta_dot, nu_dot."""
        J_eta = self.J(eta)
        eta_dot = J_eta @ nu
        C = self.coriolis_centripetal_matrix(nu)
        D = self.damping_matrix(nu)
        g_eta = self.restoring_forces(eta)
        tau = self.A @ tau_p  # Convert propeller thrusts to forces/moments
        nu_dot = self.M_inv @ (tau - C @ nu - D @ nu - g_eta)
        return eta_dot, nu_dot

class NonlinearMPC:
    def __init__(self, model, N=10, dt=0.1):
        self.model = model
        self.N = N  # Prediction horizon
        self.dt = dt  # Time step

        # State dimensions
        self.nx = 12  # eta (6) + nu (6)
        self.nu = 6   # Propeller thrusts (F1-F6)

        # Setup optimizer
        self.opti = ca.Opti()
        
        # Decision variables
        self.X = self.opti.variable(self.nx, self.N+1)  # States over horizon
        self.U = self.opti.variable(self.nu, self.N)     # Controls over horizon
        self.x_ref = self.opti.parameter(self.nx, self.N+1)  # Reference trajectory

        # Parameters for initial state and reference
        self.x0 = self.opti.parameter(self.nx)

        # Cost function weights
        Q = ca.diag([1.0]*6 + [0.1]*6)  # Weight on state error
        R = ca.diag([0.01]*6)            # Weight on control effort

        # Build cost function
        cost = 0
        for k in range(self.N+1):
            state_error = self.X[:, k] - self.x_ref[:, k]
            cost += state_error.T @ Q @ state_error
            if k < self.N:
                cost += self.U[:, k].T @ R @ self.U[:, k]

        # Dynamics constraints using multiple shooting
        for k in range(self.N):
            x_k = self.X[:, k]
            eta = x_k[:6]
            nu = x_k[6:]
            u_k = self.U[:, k]

            # Compute next state using RK4 integration
            k1_eta, k1_nu = self.model.dynamics(eta, nu, u_k)
            k2_eta, k2_nu = self.model.dynamics(eta + self.dt/2*k1_eta, nu + self.dt/2*k1_nu, u_k)
            k3_eta, k3_nu = self.model.dynamics(eta + self.dt/2*k2_eta, nu + self.dt/2*k2_nu, u_k)
            k4_eta, k4_nu = self.model.dynamics(eta + self.dt*k3_eta, nu + self.dt*k3_nu, u_k)
            
            x_next = x_k + self.dt/6 * ca.vertcat(
                k1_eta + 2*k2_eta + 2*k3_eta + k4_eta,
                k1_nu + 2*k2_nu + 2*k3_nu + k4_nu
            )
            self.opti.subject_to(self.X[:, k+1] == x_next)

        # Initial condition constraint
        self.opti.subject_to(self.X[:, 0] == self.x0)

        # Input constraints
        for k in range(self.N):
            self.opti.subject_to(self.opti.bounded(-self.model.p_front_mid_max, self.U[:2, k], self.model.p_front_mid_max))
            self.opti.subject_to(self.opti.bounded(-self.model.p_rear_max, self.U[2:4, k], self.model.p_rear_max))
            self.opti.subject_to(self.opti.bounded(-self.model.p_front_mid_max, self.U[4:, k], self.model.p_front_mid_max))

        # Setup solver
        self.opti.minimize(cost)
        # Configure solver with JIT and efficient options
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', opts)
        self.prev_sol = None  # For warm-starting

    def solve(self, x0, x_ref):
        self.opti.set_value(self.x0, x0)
        self.opti.set_value(self.x_ref, x_ref)

        # Warm-start using previous solution
        if self.prev_sol is not None:
            prev_X = self.prev_sol.value(self.X)
            prev_U = self.prev_sol.value(self.U)
            X_guess = np.hstack([prev_X[:, 1:], prev_X[:, -1:]])
            U_guess = np.hstack([prev_U[:, 1:], prev_U[:, -1:]])
            self.opti.set_initial(self.X, X_guess)
            self.opti.set_initial(self.U, U_guess)

        try:
            sol = self.opti.solve()
            self.prev_sol = sol
            return sol.value(self.U[:, 0]), sol.value(self.X)
        except Exception as e:
            print(f"Solver failed: {e}")
            if self.prev_sol:
                return self.prev_sol.value(self.U[:, 0]), self.prev_sol.value(self.X)
            else:
                return np.zeros(self.nu), np.zeros((self.nx, self.N+1))

# Example usage
if __name__ == "__main__":
    # Initialize model and MPC
    model = CasadiModel("rl_test/config.json")
    mpc = NonlinearMPC(model, N=10, dt=0.1)

    # Initial state (eta, nu)
    x0 = ca.DM([0] * 12)
    # Reference trajectory (example: hold position)
    x_ref = ca.repmat(ca.DM([20, 15, -6, 0, 0, -1, 0, 0, 0, 0, 0, 0]), 1, 11)

    # Initialize renderer
    renderer = Render(size=(1, 0.6, 0.4))
    renderer.reset(np.array([[0, 0, 0]]))  # Initialize reference trajectory

    # Simulation loop
    for step in range(100):  # Simulate for 100 steps
        start = perf_counter()
        # Solve MPC
        u_opt, x_opt = mpc.solve(x0, x_ref)

        # Update state (simulate dynamics)
        x0 = x_opt[:, 1]  # Use the next state from the MPC solution

        # Extract position and orientation for visualization
        eta = x0[:6].flatten()  # Position and orientation
        ref = x_ref[:3, 0].full().flatten()  # Reference position
        v_world = x0[6:9].flatten()  # Velocity in world frame
        desired_yaw = x_ref[5, 0].full().flatten()[0]  # Desired yaw

        # Render the current state
        renderer.render(eta, ref, v_world, desired_yaw)
        print(perf_counter() - start)

    # Close the renderer
    renderer.close()
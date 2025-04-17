import json
from scipy.linalg import block_diag
import numpy as np
from time import sleep

# TODO: tau_p change matrix implementation

class Model:
    def __init__(self, config_path: str = "rl_test/config.json"):
        """
        Initialize the motion model.
        """
        self.load_dynamics(config_path)
        self.calculate_linear()

    def load_dynamics(self, config_path):
        """
        Load dynamics data from the config file.
        """
        with open(config_path, 'r') as file:
            config = json.load(file)

        config = config["assembly_mass_properties"]

        # Load moments of inertia about the center of mass
        moments_of_inertia_about_center_of_mass = config["moments_of_inertia_about_center_of_mass"]
        self.Lxx = moments_of_inertia_about_center_of_mass["Lxx"]
        self.Lxy = moments_of_inertia_about_center_of_mass["Lxy"]
        self.Lxz = moments_of_inertia_about_center_of_mass["Lxz"]
        self.Lyx = moments_of_inertia_about_center_of_mass["Lyx"]
        self.Lyy = moments_of_inertia_about_center_of_mass["Lyy"]
        self.Lyz = moments_of_inertia_about_center_of_mass["Lyz"]
        self.Lzx = moments_of_inertia_about_center_of_mass["Lzx"]
        self.Lzy = moments_of_inertia_about_center_of_mass["Lzy"]
        self.Lzz = moments_of_inertia_about_center_of_mass["Lzz"]

        # Load center of mass
        center_of_mass = config["center_of_mass"]
        self.r_x = center_of_mass["X"]
        self.r_y = center_of_mass["Y"]
        self.r_z = center_of_mass["Z"]
        self.r_g = np.array([self.r_x, self.r_y, self.r_z]) # .reshape(3,)

        # Load center of buoyancy relative to CG
        center_of_buoancy = config["center_of_buoancy"]
        self.x_B = center_of_buoancy["X"]
        self.y_B = center_of_buoancy["Y"]
        self.z_B = center_of_buoancy["Z"]
        self.r_B = np.array([self.x_B, self.y_B, self.z_B]) # .reshape(3,)

        # Load dimensions
        dimensions = config["dimensions"]
        self.w = dimensions["width"]
        self.h = dimensions["height"]
        self.l = dimensions["length"]
        self.lm = dimensions["lm"]
        self.wf = dimensions["wf"]
        self.lr = dimensions["lr"]

        # Load mass and rear propeller angle
        self.mass = config["mass"]["value"]
        self.a = np.radians(config["rear_propeller_angle"]["value"])
        self.volume = np.radians(config["volume"]["value"])

        self.skew_m = self.mass * self._skew_symmetric(self.r_g)

        self.added_mass_coef = config["added_mass"]

        # Load dynamics
        dynamics = config["dynamics"]

        self.fluid_density = dynamics["fluid_density"]["value"]
        self.displaced_volume = dynamics["displaced_volume"]["value"]
        self.g = dynamics["g"]["value"]

        self.D_u = dynamics["damping"]["linear"]["D_u"]
        self.D_v = dynamics["damping"]["linear"]["D_v"]
        self.D_w = dynamics["damping"]["linear"]["D_w"]

        self.D_p = dynamics["damping"]["angular"]["D_p"]
        self.D_q = dynamics["damping"]["angular"]["D_q"]
        self.D_r = dynamics["damping"]["angular"]["D_r"]


        self.Dn_u = dynamics["damping"]["linear_n"]["Dn_u"]
        self.Dn_v = dynamics["damping"]["linear_n"]["Dn_v"]
        self.Dn_w = dynamics["damping"]["linear_n"]["Dn_w"]

        self.Dn_p = dynamics["damping"]["angular_n"]["Dn_p"]
        self.Dn_q = dynamics["damping"]["angular_n"]["Dn_q"]
        self.Dn_r = dynamics["damping"]["angular_n"]["Dn_r"]

        self.propeller = dynamics["propeller"]
        self.p_rear_max = self.propeller['force_range_r']['max']
        self.p_front_mid_max = self.propeller['force_range_f_m']['max']
        self.p_change_matrix_high = self.propeller['change_matrix_high']
        self.p_change_matrix_low = self.propeller['change_matrix_low']
        self.p_change_matrix_dt = self.propeller['change_matrix_dt']
        self.p_torque_coefficient = self.propeller['torque_coefficient']
        self.p_thrust_coefficient = self.propeller['thrust_coefficient']

        # Calculate weight (self.W) and buoyant force (self.B)
        self.W = self.mass * self.g  # N
        self.B = self.fluid_density * self.displaced_volume * self.g  # N
        self.W_minus_B = self.W - self.B  # N

        self.A = np.array([
            # F1   F2   F3        F4        F5   F6
            [1,    1,    np.cos(self.a),  np.cos(self.a),  0,     0    ],  # Fx
            [0,    0,    -np.sin(self.a), np.sin(self.a),  0,     0    ],  # Fy (fixed signs)
            [0,    0,    0,               0,              1,     1    ],  # Fz
            [0,    0,    0,               0,              self.lm, -self.lm],  # Mx
            [0,    0,    0,               0,              0,     0    ],  # My
            [-self.wf, self.wf, self.lr*np.sin(self.a), -self.lr*np.sin(self.a), 0, 0]  # Mz (fixed signs)
        ], dtype=np.float64)

        # Compute the pseudo-inverse of A
        self.A_pseudo_inv = np.linalg.pinv(self.A)

    def calculate_linear(self):
        # Compute Coriolis and centripetal matrix
        self._added_mass()
        self._mass_inertia_matrix()
        self._damping_matrix()

    def _convert_tau_to_prop(self, tau):
        """
        Calculate the propeller thrusts (F1, F2, F3, F4, F5, F6) based on desired forces and moments.

        Args:
            tau (np.ndarray): A NumPy array of 6 forces and moments (Fx, Fy, Fz, Mx, My, Mz).
                                - Fx, Fy, Fz: Forces in x, y, z directions (N).
                                - Mx, My, Mz: Moments about x, y, z axes (Nm).

        Returns:
            np.ndarray: Thrust values (F1, F2, F3, F4, F5, F6) for the propellers (N).
        """

        return np.dot(self.A_pseudo_inv, tau)

    def _convert_prop_to_tau(self, tau_p):
        """
        Calculate the resulting forces (Fx, Fy, Fz) and moments (Mx, My, Mz) based on propeller thrusts.

        Args:
            tau_p (np.ndarray): A NumPy array of 6 forces and moments (F1, F2, F3, F4, F5, F6).

        Returns:
            np.ndarray: Fx, Fy, Fz, Mx, My, Mz
        """
        F1, F2, F3, F4, F5, F6 = tau_p  # N
        Fx = F1 + F2 + (F3 + F4) * np.cos(self.a)  # N
        Fy = (F3 - F4) * np.sin(self.a)  # N
        Mz = self.wf * (F2 - F1) + self.lr * (F4 - F3) * np.sin(self.a)  # Nm
        
        Fz = F5 + F6  # N
        Mx = (F5 - F6) * self.lm  # Nm
        My = 0.0  # Nm (assuming no pitch moment from thrusters)

        return np.array([float(Fx), float(Fy), float(Fz), float(Mx), float(My), float(Mz)])

    def _skew_symmetric(self, a: np.array):
        """
        Compute the skew-symmetric matrix for a given vector.

        Args:
            a (np.ndarray): Input vector (default: linear velocity from environment data) (m/s).

        Returns:
            np.ndarray: Skew-symmetric matrix.
        """
        return np.array([
            [0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]
        ])

    def _added_mass(self):
        """Return the combined added mass matrix (6x6) from chassis and body."""
        self.A11 = np.array([
            [-self.added_mass_coef["C_X"], 0, 0],
            [0, -self.added_mass_coef["C_Y"], 0],
            [0, 0, -self.added_mass_coef["C_Z"]]
        ])
        
        self.A12 = np.array([
            [0, 0, 0],
            [0, 0, -self.added_mass_coef["C_Y_r"]],
            [0, -self.added_mass_coef["C_Z_q"], 0]
        ])
        
        self.A21 = self.A12.T
        
        self.A22 = np.array([
            [-self.added_mass_coef["C_K"], 0, 0],
            [0, -self.added_mass_coef["C_M"], 0],
            [0, 0, -self.added_mass_coef["C_N"]]
        ])
        
        self.skew_A11 = self._skew_symmetric(np.diag(self.A11))
        self.skew_A22 = self._skew_symmetric(np.diag(self.A22))

        self.Ma = np.block([
            [self.A11, self.A12],
            [self.A21, self.A22]
        ])

    def _mass_inertia_matrix(self):
        """
        Compute the mass-inertia matrix (M) using the loaded dynamics.
        """
        # Corrected to use moments of inertia about the center of mass (Lxx, Lxy, etc.)
        self.I = np.array([
            [self.Lxx, -self.Lxy, -self.Lxz],
            [-self.Lyx, self.Lyy, -self.Lyz],
            [-self.Lzx, -self.Lzy, self.Lzz]
        ])  # kg·m²
        
        self.skew_I = self._skew_symmetric(np.diag(self.I))

        # Construct mass-inertia matrix
        self.Mrb = np.block([
            [self.mass * np.eye(3), -self.skew_m],
            [self.skew_m, self.I]
        ])  # kg (for mass) and kg·m² (for inertia)

        self.M = self.Mrb + self.Ma
        self.M_inv = np.linalg.inv(self.M)

    def _damping_matrix(self):
        """
        Compute the damping matrix (D) based on linear and angular damping coefficients.
        """
        # Construct linear damping matrix
        D_lin = np.diag([self.D_u, self.D_v, self.D_w])  # N·s/m

        # Construct angular damping matrix
        D_ang = np.diag([self.D_p, self.D_q, self.D_r])  # N·m·s/rad

        # Construct damping matrix
        self.Dl = np.block([
            [D_lin, np.zeros((3, 3))],
            [np.zeros((3, 3)), D_ang]
        ])  # N·s/m (linear) and N·m·s/rad (angular)

        # Construct linear damping matrix
        Dn_lin = np.diag([self.Dn_u, self.Dn_v, self.Dn_w])  # N·s/m

        # Construct angular damping matrix
        Dn_ang = np.diag([self.Dn_p, self.Dn_q, self.Dn_r])  # N·m·s/rad

        self.Dn = np.block([
            [Dn_lin, np.zeros((3, 3))],
            [np.zeros((3, 3)), Dn_ang]
        ])

    def _normalize_angle(self, angle):
        """
        Normalize an angle to the range [-π, π].

        Parameters:
            angle (float or np.ndarray): Angle in radians.

        Returns:
            normalized_angle (float or np.ndarray): Angle normalized to [-π, π].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_nu_dot(self, eta, nu, tau_p):
        # Compute forces and moments from thrusters.
        tau = self._convert_prop_to_tau(tau_p)
        C = self.coriolis_centripetal_matrix(nu)
        D = self.damping_matrix(nu)

        return self.M_inv @ (-(C @ nu) - (D @ nu) - self.restoring_forces(eta) + tau)

    def compute_force(self, eta, nu, nu_dot):
        """
        Compute the external forces and moments (tau) using the Newton-Euler equations.

        Returns:
        - tau (np.ndarray): External forces and moments as a 6-dimensional vector:
            - tau[0]: Force in the x-direction (N).
            - tau[1]: Force in the y-direction (N).
            - tau[2]: Force in the z-direction (N).
            - tau[3]: Moment about the x-axis (Nm).
            - tau[4]: Moment about the y-axis (Nm).
            - tau[5]: Moment about the z-axis (Nm).
        """

        C = self.coriolis_centripetal_matrix(nu)
        D = self.damping_matrix(nu)

        # Compute external forces and moments
        tau = (self.M @ nu_dot) + (C @ nu) - (D @ nu) + self.restoring_forces(eta)

        return tau

    def J(self, eta: np.array):
        """
        Compute the full nonlinear transformation matrix J(η) from body-fixed velocities
        to inertial frame rates.
        """
        # Assume η = [x, y, z, φ, θ, ψ] with φ, θ, ψ at indices 3, 4, 5.
        phi, theta, psi = eta[3:6]
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta, stheta = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        # Rotation matrix from body to inertial frame.
        R = np.array([
            [cpsi * ctheta, -spsi * cphi + cpsi * stheta * sphi, spsi * sphi + cpsi * cphi * stheta],
            [spsi * ctheta,  cpsi * cphi + spsi * stheta * sphi, -cpsi * sphi + spsi * stheta * cphi],
            [-stheta,        ctheta * sphi,                       ctheta * cphi]
        ])
        T = np.array([
            [1, sphi*np.tan(theta), cphi*np.tan(theta)],
            [0, cphi, -sphi],
            [0, sphi/ctheta, cphi/ctheta]
        ])
        
        self.Jacobian = block_diag(R, T)
        
        return self.Jacobian

    def restoring_forces(self, eta: np.array):
        """
        Compute the restoring forces (g_eta) due to gravity and buoyancy.

        Args:
            g (float): Acceleration due to gravity (m/s²).
        """
        roll, pitch, _ = eta[3:6]

        # Calculate trigonometric terms
        sin_phi = np.sin(roll)
        cos_phi = np.cos(roll)
        sin_theta = np.sin(pitch)
        cos_theta = np.cos(pitch)

        # Calculate restoring forces
        self.g_eta = np.array([
            self.W_minus_B * sin_theta,  # Force in x-direction (N)
            -self.W_minus_B * cos_theta * sin_phi,  # Force in y-direction (N)
            -self.W_minus_B * cos_theta * cos_phi,  # Force in z-direction (N)
            -(self.r_y * self.W - self.y_B * self.B) * cos_theta * cos_phi + (self.r_z * self.W - self.z_B * self.B) * cos_theta * sin_phi,  # Moment about x-axis (Nm)
            (self.r_z * self.W - self.z_B * self.B) * sin_theta + (self.r_x * self.W - self.x_B * self.B) * cos_theta * cos_phi,  # Moment about y-axis (Nm)
            -(self.r_x * self.W - self.x_B * self.B) * cos_theta * sin_phi + -(self.r_y * self.W - self.y_B * self.B) * sin_theta  # Moment about z-axis (Nm)
        ])
        
        return self.g_eta

    def damping_matrix(self, nu):
        """
        Compute the damping matrix (D) based on linear and angular damping coefficients.
        """
        # Construct damping matrix
        Dn = (self.Dn @ np.diag(np.abs(nu)))
        self.D = Dn + self.Dl
        
        return self.D

    def coriolis_centripetal_matrix(self, nu):
        """
        Compute the Coriolis and centripetal matrix (C) based on the vehicle's velocity and inertia.
        """
        skew_nu1 = self._skew_symmetric(nu[:3])
        skew_nu2 = self._skew_symmetric(nu[3:6])
        
        self.Crb = np.block([
            [np.zeros((3,3)), - (self.mass * skew_nu1) - (skew_nu2 * self.skew_m)],
            [- (self.mass * skew_nu1) + (skew_nu2 * self.skew_m), -(skew_nu2 * self.skew_I)]
        ])
        
        # Correct Coriolis matrix
        self.Ca = np.block([
            [np.zeros((3,3)), -self.skew_A11*skew_nu1],
            [-self.skew_A11*skew_nu1,  -self.skew_A22*skew_nu2]
        ])
        
        self.C = self.Crb + self.Ca
        
        return self.C

    def update_state(self, eta, nu, nu_dot, dt):

        # Update velocity states.
        nu += nu_dot * dt

        # Update position/orientation states using the full transformation.
        eta += (self.J(eta) @ nu.reshape(-1, 1)).flatten() * dt

        eta = np.concatenate([eta[:3], self._normalize_angle(eta[3:])])

        return eta, nu

    def step(self, eta, nu, tau_p, dt):
        nu_dot = self.compute_nu_dot(eta, nu, tau_p)
        eta, nu = self.update_state(eta, nu, nu_dot, dt)
        return eta, nu, nu_dot

    def update_propeller(self, prop_f, prop_r, prop_m, tau_p_change):
        prop_f += tau_p_change[:2]
        prop_r += tau_p_change[2:4]
        prop_m += tau_p_change[4:]
        prop_f = np.clip(prop_f, -self.p_front_mid_max, self.p_front_mid_max)
        prop_r = np.clip(prop_r, -self.p_rear_max, self.p_rear_max)
        prop_m = np.clip(prop_m, -self.p_front_mid_max, self.p_front_mid_max)
        return np.concatenate([prop_f, prop_r, prop_m])









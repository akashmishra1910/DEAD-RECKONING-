r"""
idr_pipeline/eskf_fusion.py
===========================
15-State Error-State Kalman Filter (ESKF) for 3D Vehicle Dead Reckoning.

Addresses Claim 2:
"Physics constraints + sensor fusion bound the error: Combining the AI speed estimate
with non-holonomic vehicle constraints inside an Error-State Kalman Filter keeps
position drift under 10% of distance traveled during a GNSS outage."

Coordinate System:
- Navigation Frame (n): Local East (x), North (y), Up (z) tangent to WGS-84 (right-handed, det = +1).
- Body Frame (b): Forward (x_b), Left (y_b), Up (z_b) (right-handed, det = +1).
- Attitude: Yaw (azimuth clockwise from North), Pitch (elevation angle), Roll (bank angle).
- DCM: R_b^n in SO(3) transforms body coordinates to navigation coordinates (det(R_b^n) = +1.0).

States:
    delta_x = [delta_p^n, delta_v^n, delta_theta^n, delta_b_a^b, delta_b_g^b]^T in R^15

Author: Autonomous Navigation Research Team
"""

import numpy as np
from typing import Tuple, Optional


def wrap_pi(angle: float) -> float:
    """Wraps an angle in radians to [-pi, +pi]."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """Computes 3x3 skew-symmetric cross-product matrix [v]_x."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ])


def rodrigues(rot_vec: np.ndarray) -> np.ndarray:
    """Computes 3x3 rotation matrix from rotation vector using Rodrigues' formula."""
    theta = float(np.linalg.norm(rot_vec))
    if theta < 1e-12:
        return np.eye(3) + skew_symmetric(rot_vec)
    axis = rot_vec / theta
    K = skew_symmetric(axis)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Direction Cosine Matrix R_b^n from vehicle body frame (b: Forward, Left, Up)
    to navigation frame (n: East, North, Up).
    R_b^n in SO(3) with det(R_b^n) = +1.0.

    Yaw is azimuth from North (0 = North, pi/2 = East, clockwise).
    Pitch is elevation angle (positive = nose up).
    Roll is bank angle (positive = right side down / left side up).
    """
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cr = np.cos(roll)
    sr = np.sin(roll)

    # Column 0: Forward x_b vector in navigation frame
    r1 = np.array([
        sy * cp,
        cy * cp,
        sp
    ])

    # Base lateral (Left y_b) and vertical (Up z_b) columns before roll:
    # In horizontal plane, when facing azimuth yaw:
    # Forward = [sy, cy, 0]
    # Left (90 deg CCW from Forward) = [-cy, sy, 0]
    # Up = r1 x r2_base = [-sy * sp, -cy * sp, cp]
    r2_base = np.array([-cy, sy, 0.0])
    r3_base = np.array([-sy * sp, -cy * sp, cp])

    # Bank/roll rotation applied around forward axis x_b:
    r2 = cr * r2_base + sr * r3_base
    r3 = -sr * r2_base + cr * r3_base

    return np.column_stack([r1, r2, r3])


def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """
    Extracts roll, pitch, yaw (azimuth clockwise from North) from R_b^n (FLU -> ENU).
    """
    pitch = float(np.arcsin(np.clip(R[2, 0], -1.0, 1.0)))
    yaw = float(np.arctan2(R[0, 0], R[1, 0]))
    roll = float(np.arctan2(R[2, 1], R[2, 2]))
    return roll, pitch, yaw


def renormalize_rotation_matrix(R: np.ndarray) -> np.ndarray:
    """
    Re-orthogonalizes R using SVD projection to the nearest SO(3) matrix.
    Guarantees R @ R.T = I and det(R) = +1.0.
    """
    U, _, Vt = np.linalg.svd(R)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0.0:
        U[:, -1] *= -1.0
        R_ortho = U @ Vt
    return R_ortho


class ErrorStateKalmanFilter:
    """
    15-State Error-State Kalman Filter for 3D land vehicle dead reckoning.
    Supports complete 3D attitude propagation and Kalman correction.
    """

    def __init__(
        self,
        init_pos: Optional[np.ndarray] = None,
        init_vel: Optional[np.ndarray] = None,
        init_euler: Optional[np.ndarray] = None,  # [roll, pitch, yaw]
        init_ba: Optional[np.ndarray] = None,
        init_bg: Optional[np.ndarray] = None,
        acc_noise: float = 0.15,
        gyro_noise: float = 0.008,
        acc_bias_noise: float = 1e-4,
        gyro_bias_noise: float = 1e-5
    ):
        # 1. Nominal States
        self.p = np.zeros(3) if init_pos is None else init_pos.astype(float).copy()    # [East, North, Up] in meters
        self.v = np.zeros(3) if init_vel is None else init_vel.astype(float).copy()    # [vx, vy, vz] in m/s (nav frame)
        euler = np.zeros(3) if init_euler is None else init_euler.astype(float)
        self.roll = float(euler[0])
        self.pitch = float(euler[1])
        self.yaw = float(euler[2])
        self.R = euler_to_rotation_matrix(self.roll, self.pitch, self.yaw)

        self.ba = np.zeros(3) if init_ba is None else init_ba.astype(float).copy()
        self.bg = np.zeros(3) if init_bg is None else init_bg.astype(float).copy()

        # Gravity in navigation frame (ENU: gravity points downwards along -Up)
        self.g_nav = np.array([0.0, 0.0, -9.80665])

        # 2. Error State Covariance Matrix P (15x15)
        self.P = np.diag([
            1.0, 1.0, 2.0,       # pos cov (m^2)
            0.2, 0.2, 0.2,       # vel cov ((m/s)^2)
            0.005, 0.005, 0.01,  # att cov (rad^2)
            0.01, 0.01, 0.01,    # acc bias cov ((m/s^2)^2)
            1e-5, 1e-5, 1e-5     # gyr bias cov ((rad/s)^2)
        ])

        # Process Noise spectral densities
        self.q_acc = acc_noise**2
        self.q_gyr = gyro_noise**2
        self.q_ba = acc_bias_noise**2
        self.q_bg = gyro_bias_noise**2

    def predict(self, accel_meas: np.ndarray, gyro_meas: np.ndarray, dt: float):
        """
        3D Mechanization and Error State Covariance Propagation.
        Propagates full 3-axis attitude using body angular rate vector.
        """
        # Unbias measurements
        f_b = accel_meas - self.ba
        w_b = gyro_meas - self.bg

        # --- A. Nominal State Mechanization ---
        # Specific force transformed to navigation frame
        f_n = self.R @ f_b
        # Linear acceleration: a^n = f^n + g^n
        a_n = f_n + self.g_nav

        # Trapezoidal position and velocity update
        self.p += self.v * dt + 0.5 * a_n * dt**2
        self.v += a_n * dt

        # Full 3D attitude propagation using Rodrigues rotation formula
        # Continuous DCM kinematics: dR/dt = R @ [w_b]_x
        # Discrete step: R(t + dt) = R(t) @ Exp([w_b * dt]_x)
        dtheta_b = w_b * dt
        self.R = self.R @ rodrigues(dtheta_b)
        self.R = renormalize_rotation_matrix(self.R)

        # Update Euler angles to maintain synchronization and API compatibility
        self.roll, self.pitch, self.yaw = rotation_matrix_to_euler(self.R)

        # --- B. Continuous Error State Matrix F (15x15) ---
        # Error states: [delta_p^n, delta_v^n, delta_theta^n, delta_b_a^b, delta_b_g^b]
        # In navigation frame tangent plane (omega_in^n ≈ 0):
        # dot(delta_p) = delta_v
        # dot(delta_v) = -[f_n]_x * delta_theta^n - R * delta_b_a
        # dot(delta_theta^n) = -[omega_in^n]_x * delta_theta^n - R * delta_b_g
        F = np.zeros((15, 15))
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -skew_symmetric(f_n)
        F[3:6, 9:12] = -self.R
        # Attitude error dynamics: -R maps body gyro bias error into nav frame
        F[6:9, 12:15] = -self.R

        Phi = np.eye(15) + F * dt

        # Process noise covariance Q_d
        Q_d = np.zeros((15, 15))
        Q_d[3:6, 3:6] = np.eye(3) * self.q_acc * dt
        Q_d[6:9, 6:9] = np.eye(3) * self.q_gyr * dt
        Q_d[9:12, 9:12] = np.eye(3) * self.q_ba * dt
        Q_d[12:15, 12:15] = np.eye(3) * self.q_bg * dt

        self.P = Phi @ self.P @ Phi.T + Q_d
        self.P = 0.5 * (self.P + self.P.T)  # Minimal numerical symmetrization

    def update_gnss(self, gnss_pos: np.ndarray, r_gnss: float = 2.5):
        """Measurement update using GNSS absolute position in navigation frame (ENU)."""
        dz = gnss_pos - self.p
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        R_cov = np.eye(3) * (r_gnss**2)
        self._apply_kalman_update(dz, H, R_cov)

    def update_gnss_velocity(self, gnss_vel: np.ndarray, r_vel: float = 0.2):
        """Measurement update using GNSS velocity in navigation frame (ENU)."""
        dz = gnss_vel - self.v
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        R_cov = np.eye(3) * (r_vel**2)
        self._apply_kalman_update(dz, H, R_cov)

    def update_ai_velocity(self, v_ai: float, r_ai: float = 0.35):
        """
        Measurement update using AI forward speed estimate.
        Forward speed in vehicle body frame: v_x^b = [1, 0, 0] @ R.T @ v^n.
        """
        v_b = self.R.T @ self.v
        dz = np.array([v_ai - v_b[0]])

        H = np.zeros((1, 15))
        H[0, 3:6] = self.R[:, 0]  # First column of R is forward body axis in nav frame
        H[0, 6:9] = self.R[:, 0] @ skew_symmetric(self.v)

        R_cov = np.array([[r_ai**2]])
        self._apply_kalman_update(dz, H, R_cov)

    def update_nhc(self, r_lat: float = 0.05, r_vert: float = 0.05):
        """
        Non-Holonomic Constraints (NHC) measurement update.
        Enforces lateral velocity v_y^b = 0 and vertical velocity v_z^b = 0.
        """
        v_b = self.R.T @ self.v
        dz = np.array([-v_b[1], -v_b[2]])

        H = np.zeros((2, 15))
        H[0, 3:6] = self.R[:, 1]  # Lateral axis
        H[0, 6:9] = self.R[:, 1] @ skew_symmetric(self.v)

        H[1, 3:6] = self.R[:, 2]  # Vertical axis
        H[1, 6:9] = self.R[:, 2] @ skew_symmetric(self.v)

        R_cov = np.diag([r_lat**2, r_vert**2])
        self._apply_kalman_update(dz, H, R_cov)

    def update_zupt(self, r_zupt: float = 0.01):
        """Zero Velocity Updates (ZUPT)."""
        dz = np.array([0.0, 0.0, 0.0]) - self.v
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        R_cov = np.eye(3) * (r_zupt**2)
        self._apply_kalman_update(dz, H, R_cov)

    def update_altitude(self, alt_meas: float, r_alt: float = 0.8):
        """Barometric altitude measurement update."""
        dz = np.array([alt_meas - self.p[2]])
        H = np.zeros((1, 15))
        H[0, 2] = 1.0
        R_cov = np.array([[r_alt**2]])
        self._apply_kalman_update(dz, H, R_cov)

    def update_heading(self, heading_meas: float, r_yaw: float = 0.05, chi2_gate: Optional[float] = None) -> Tuple[bool, float]:
        """
        Magnetometer / compass / GNSS course tilt-compensated heading measurement update.
        Constrains heading drift in vehicle yaw state.
        
        Args:
            heading_meas: Azimuth in radians clockwise from North [0, 2*pi)
            r_yaw: 1-sigma measurement noise in radians
            chi2_gate: Optional chi-square threshold for innovation gating
            
        Returns:
            (is_accepted, gamma): Gating acceptance status and normalized innovation
        """
        # Smallest signed angle difference in [-pi, +pi]
        dyaw = float(np.arctan2(np.sin(heading_meas - self.yaw), np.cos(heading_meas - self.yaw)))
        dz = np.array([dyaw])
        H = np.zeros((1, 15))
        # In ENU (Up = +z), delta_theta_z^n is CCW while yaw azimuth is CW from North, so delta_yaw = -delta_theta_z^n
        H[0, 8] = -1.0
        R_cov = np.array([[r_yaw**2]])
        return self._apply_kalman_update(dz, H, R_cov, chi2_gate=chi2_gate)

    def _apply_kalman_update(
        self,
        dz: np.ndarray,
        H: np.ndarray,
        R_cov: np.ndarray,
        chi2_gate: Optional[float] = None
    ) -> Tuple[bool, float]:
        """
        Applies Joseph-form covariance update, performs full 3D attitude correction,
        and executes consistent error-state reset.
        
        Args:
            dz: (m,) Innovation vector
            H: (m, 15) Measurement Jacobian matrix
            R_cov: (m, m) Measurement noise covariance matrix
            chi2_gate: Optional Chi-Square gate threshold (gamma <= chi2_gate)
            
        Returns:
            (is_accepted, gamma): Whether measurement passed gate, and normalized innovation
        """
        S = H @ self.P @ H.T + R_cov
        S_inv = np.linalg.inv(S)
        gamma = float(dz.T @ S_inv @ dz)

        if chi2_gate is not None and gamma > chi2_gate:
            return False, gamma

        K = self.P @ H.T @ S_inv

        dx = K @ dz

        # 1. Inject into nominal translational states
        self.p += dx[0:3]
        self.v += dx[3:6]

        # 2. Inject full 3D attitude error dx[6:9] into nominal attitude
        # In navigation frame error-state convention:
        # R_nominal_corrected = Exp([dx[6:9]]_x) @ R_nominal
        dtheta_n = dx[6:9]
        self.R = rodrigues(dtheta_n) @ self.R
        self.R = renormalize_rotation_matrix(self.R)
        self.roll, self.pitch, self.yaw = rotation_matrix_to_euler(self.R)

        # 3. Inject bias corrections
        self.ba += dx[9:12]
        self.bg += dx[12:15]

        # 4. Joseph-form covariance update
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_cov @ K.T

        # 5. ESKF Error-State Reset Jacobian
        # Higher-order attitude error reset: G_reset = I - 0.5 * skew(dtheta_n)
        G_reset = np.eye(15)
        G_reset[6:9, 6:9] = np.eye(3) - 0.5 * skew_symmetric(dtheta_n)
        self.P = G_reset @ self.P @ G_reset.T

        # Guarantee numerical symmetry of covariance matrix
        self.P = 0.5 * (self.P + self.P.T)
        return True, gamma

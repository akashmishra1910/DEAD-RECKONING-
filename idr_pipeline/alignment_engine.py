"""
idr_pipeline/alignment_engine.py
================================
In-Vehicle Alignment & Calibration Engine.

Automatically determines the smartphone's pitch, roll, and yaw relative to the
vehicle's driving direction, whether the phone is mounted in a dashboard cradle,
placed in an angled mobile holder, or shifted during driving.

Algorithmic Phases:
-------------------
1. Coarse Leveling Phase (Pitch & Roll):
   During initial vehicle stationary periods (detected via ZUPT or engine idle):
       g^p = (1/M) * sum(f_k^p)
       z_b = g^p / ||g^p||
   This isolates the local vertical gravity axis in the phone frame.
   Roll (phi) and Pitch (theta) are derived directly from z_b.

2. Forward Heading Phase (Longitudinal Yaw Alignment):
   During initial straight-line acceleration or braking transients:
       Delta f^p = f_k^p - g^p
       Project Delta f^p onto horizontal plane:
       f_horiz^p = Delta f^p - (Delta f^p . z_b) * z_b
       x_b = f_horiz^p / ||f_horiz^p||  (aligned with positive forward acceleration)
   The lateral axis is uniquely defined via right-hand cross product:
       y_b = z_b x x_b
   The phone-to-vehicle rotation matrix is:
       R_p2b = [x_b, y_b, z_b]^T
   and vehicle-to-phone rotation matrix is:
       R_b2p = R_p2b^T = [x_b, y_b, z_b]

3. Online Slip & Cradle Shift Watchdog:
   Continuously tracks a low-pass filtered gravity vector. If a physical bump
   or mount slip alters the orientation by more than a threshold angle (e.g. 5 deg),
   triggers autonomous online re-calibration without halting navigation.
"""

import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass


@dataclass
class AlignmentState:
    is_leveled: bool = False
    is_aligned: bool = False
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    R_p2b: Optional[np.ndarray] = None  # (3, 3) transforms phone frame -> vehicle body frame
    R_b2p: Optional[np.ndarray] = None  # (3, 3) transforms vehicle body frame -> phone frame
    confidence: float = 0.0


class InVehicleAlignmentEngine:
    """
    Autonomous in-vehicle orientation calibration engine.
    Converts phone-frame inertial measurements to vehicle body coordinates:
        f^b = R_p2b @ f^p
        omega^b = R_p2b @ omega^p
    """

    def __init__(
        self,
        g_nominal: float = 9.80665,
        leveling_samples: int = 25,
        min_accel_transient: float = 0.4, # min m/s^2 forward surge to reliably identify forward axis
        slip_threshold_deg: float = 5.0
    ):
        self.g_nominal = g_nominal
        self.leveling_samples = leveling_samples
        self.min_accel_transient = min_accel_transient
        self.slip_threshold_deg = slip_threshold_deg

        # Internal state buffers
        self._stationary_accels: List[np.ndarray] = []
        self._transient_accels: List[np.ndarray] = []
        self._lp_gravity = np.array([0.0, 0.0, g_nominal])
        self._last_calibrated_gravity = np.array([0.0, 0.0, g_nominal])

        # Unit axes in phone frame
        self._z_b_phone = np.array([0.0, 0.0, 1.0])
        self._x_b_phone = np.array([1.0, 0.0, 0.0])
        self._y_b_phone = np.array([0.0, 1.0, 0.0])

        # Rotation matrices
        self.R_p2b = np.eye(3)
        self.R_b2p = np.eye(3)

        self.state = AlignmentState(
            is_leveled=False,
            is_aligned=False,
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
            R_p2b=np.eye(3),
            R_b2p=np.eye(3),
            confidence=0.0
        )

    def process_sample(
        self,
        accel_p: np.ndarray,
        gyro_p: np.ndarray,
        is_stationary: bool,
        gnss_speed: Optional[float] = None
    ) -> AlignmentState:
        """
        Processes a single IMU sample in the phone frame.
        
        Args:
            accel_p: (3,) accelerometer reading in phone frame (m/s^2)
            gyro_p: (3,) gyroscope reading in phone frame (rad/s)
            is_stationary: bool flag from ZUPT / engine idle detector
            gnss_speed: Optional GNSS speed in m/s (if available, accelerates heading alignment)
        """
        # Update low-pass gravity filter: alpha = 0.02
        self._lp_gravity = 0.98 * self._lp_gravity + 0.02 * accel_p

        # --- Phase 1: Leveling (Gravity Vector Estimation) ---
        if not self.state.is_leveled:
            if is_stationary:
                self._stationary_accels.append(accel_p)
                if len(self._stationary_accels) >= self.leveling_samples:
                    self._perform_leveling()
            return self.state

        # --- Check Watchdog for Phone Mount Slip / Bump ---
        if self.state.is_aligned and is_stationary:
            angle_diff = self._angular_difference(self._lp_gravity, self._last_calibrated_gravity)
            if angle_diff > self.slip_threshold_deg:
                # Phone was bumped or shifted in the cradle! Trigger automatic fast re-calibration
                self.state.is_aligned = False
                self.state.is_leveled = False
                self._stationary_accels = [accel_p]
                self._transient_accels = []
                return self.state

        # --- Phase 2: Forward Axis Determination via Acceleration Transients ---
        if self.state.is_leveled and not self.state.is_aligned:
            delta_f = accel_p - self._lp_gravity
            horiz_f = delta_f - np.dot(delta_f, self._z_b_phone) * self._z_b_phone
            horiz_mag = np.linalg.norm(horiz_f)

            if horiz_mag >= self.min_accel_transient:
                self._transient_accels.append(horiz_f)
                if len(self._transient_accels) >= 15:
                    self._perform_forward_alignment()

        return self.state

    def _perform_leveling(self):
        """Computes vertical axis and mount roll/pitch."""
        mean_g = np.mean(self._stationary_accels, axis=0)
        norm_g = np.linalg.norm(mean_g)
        if norm_g < 1e-3:
            return

        self._z_b_phone = mean_g / norm_g
        self._last_calibrated_gravity = mean_g.copy()
        self._lp_gravity = mean_g.copy()

        # Compute mount roll and pitch angles
        pitch = -np.arcsin(np.clip(self._z_b_phone[1], -1.0, 1.0))
        roll = np.arctan2(self._z_b_phone[0], self._z_b_phone[2])




        self.state.is_leveled = True
        self.state.roll_deg = float(np.degrees(roll))
        self.state.pitch_deg = float(np.degrees(pitch))
        self.state.confidence = 0.5

    def _perform_forward_alignment(self):
        """Computes forward longitudinal axis and builds orthogonal R_p2b."""
        X = np.array(self._transient_accels)
        mean_surge = np.mean(X, axis=0)
        forward_cand = mean_surge - np.dot(mean_surge, self._z_b_phone) * self._z_b_phone
        norm_cand = np.linalg.norm(forward_cand)
        if norm_cand < 1e-3:
            return
        forward_cand = forward_cand / norm_cand

        self._x_b_phone = forward_cand

        self._y_b_phone = np.cross(self._z_b_phone, self._x_b_phone)
        self._y_b_phone /= np.linalg.norm(self._y_b_phone)

        self.R_b2p = np.column_stack([self._x_b_phone, self._y_b_phone, self._z_b_phone])
        self.R_p2b = self.R_b2p.T

        yaw = np.arctan2(self.R_p2b[0, 1], self.R_p2b[0, 0])
        self.state.yaw_deg = float(np.degrees(yaw))
        self.state.R_p2b = self.R_p2b.copy()
        self.state.R_b2p = self.R_b2p.copy()
        self.state.is_aligned = True
        self.state.confidence = 1.0

    def transform_phone_to_vehicle(self, vec_phone: np.ndarray) -> np.ndarray:
        """Transforms a 3D vector (accel or gyro) from phone frame to vehicle body frame."""
        return self.R_p2b @ vec_phone

    @staticmethod
    def _angular_difference(v1: np.ndarray, v2: np.ndarray) -> float:
        """Returns angle in degrees between two 3D vectors."""
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_theta = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        return float(np.degrees(np.arccos(cos_theta)))

    @staticmethod
    def estimate_mounting_matrix_batch(
        accel_phone: np.ndarray,
        gyro_phone: Optional[np.ndarray] = None,
        speed: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Computes the static mounting matrix R_p2b (phone frame -> vehicle body frame)
        over an entire drive via gravity leveling and PCA longitudinal alignment.
        """
        mean_a = np.mean(accel_phone, axis=0)
        norm_a = np.linalg.norm(mean_a)
        if norm_a < 1e-3:
            return np.eye(3)
        z_b = mean_a / norm_a

        diff_a = accel_phone - mean_a
        a_vert = np.dot(diff_a, z_b)
        a_horiz = diff_a - np.outer(a_vert, z_b)

        cov_h = np.cov(a_horiz.T)
        eigvals, eigvecs = np.linalg.eigh(cov_h)
        x_b = eigvecs[:, -1]
        x_b = x_b - np.dot(x_b, z_b) * z_b
        norm_x = np.linalg.norm(x_b)
        if norm_x > 1e-3:
            x_b = x_b / norm_x
        else:
            cand = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], z_b) * z_b
            x_b = cand / (np.linalg.norm(cand) + 1e-6)

        # Resolve forward sign
        if speed is not None and len(speed) > 10:
            dv = np.gradient(speed)
            a_x = np.dot(a_horiz, x_b)
            valid = (np.abs(dv) > 0.05) & (np.abs(a_x) > 0.1)
            if np.sum(valid) > 20:
                if np.corrcoef(a_x[valid], dv[valid])[0, 1] < 0:
                    x_b = -x_b
        else:
            # Vehicle braking decelerations are sharper than forward acceleration (negative skewness)
            a_x = np.dot(a_horiz, x_b)
            skew_x = np.mean((a_x - np.mean(a_x))**3) / (np.std(a_x)**3 + 1e-6)
            if skew_x > 0.1:
                x_b = -x_b

        y_b = np.cross(z_b, x_b)
        norm_y = np.linalg.norm(y_b)
        if norm_y > 1e-3:
            y_b = y_b / norm_y

        R_p2b = np.vstack([x_b, y_b, z_b])
        return R_p2b

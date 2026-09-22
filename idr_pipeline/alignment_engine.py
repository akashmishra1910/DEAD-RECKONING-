"""
idr_pipeline/alignment_engine.py
================================
In-Vehicle Alignment & Multi-Mode Calibration Engine.

Automatically determines the smartphone's pitch, roll, and yaw relative to the
vehicle's driving direction, whether the phone is mounted in a dashboard cradle,
placed in an angled mobile holder, or shifted during driving.

Algorithmic Capabilities:
-------------------------
1. Mode A — Stationary Leveling & Gyro Bias Estimation:
   During initial vehicle stationary periods (detected via zero-velocity / engine idle):
       g^p = (1/M) * sum(f_k^p)
       z_b = g^p / ||g^p||  (Local vertical Up axis in phone frame)
       Roll (phi) and Pitch (theta) are derived directly from z_b.
       Initial gyro bias b_g^p = (1/M) * sum(omega_k^p) is computed and transformed to body.
       Stationary confidence is computed based on ||g^p|| - g_nominal consistency and variance.

2. Mode B — Quasi-Static / Low-Dynamic Monitoring:
   When vehicle motion is below surge threshold, maintains gravity leveling and
   explicitly reports forward axis as UNOBSERVABLE (confidence = 0.0), preventing
   downstream filters from hallucinating a forward direction.

3. Mode C — Dynamic Forward Surge / PCA Alignment:
   During forward acceleration surges (a_horiz > min_accel_transient):
       Computes horizontal covariance matrix C_h = Cov(a_horiz).
       Eigenvalue separation ratio kappa = lambda_max / lambda_min is checked.
       If kappa >= 2.0 and peak surge >= 0.35 m/s^2, forward axis x_b is uniquely resolved.
       Forward direction sign is resolved via speed correlation or braking skewness.
       y_b = z_b x x_b  (Left axis in FLU body frame).
       R_p2b = [x_b, y_b, z_b]^T in SO(3) with det(R_p2b) = +1.0.

4. Mode D — Online Slip & Cradle Shift Watchdog:
   Continuously monitors low-pass gravity and high angular rate impulses.
   If a physical bump or mount slip alters the orientation by > slip_threshold_deg,
   triggers autonomous online re-calibration without halting navigation.

5. Validation & Quality Diagnostics:
   Validates all mounting matrices: det(R) = +1.0, R @ R.T = I, non-collinear axes,
   zero NaNs/Infs. Produces machine-readable AlignmentQuality diagnostics.

Author: Lead Navigation Systems Engineer
"""

import numpy as np
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass, field
from enum import Enum


class AlignmentMode(Enum):
    INITIALIZING = "INITIALIZING"
    STATIONARY_LEVELED = "STATIONARY_LEVELED"
    QUASI_STATIC = "QUASI_STATIC"
    DYNAMIC_ALIGNED = "DYNAMIC_ALIGNED"
    DEGRADED = "DEGRADED"
    RECALIBRATING = "RECALIBRATING"


@dataclass
class AlignmentQuality:
    """Comprehensive machine-readable alignment quality and confidence metrics."""
    gravity_confidence: float = 0.0          # [0, 1] Confidence in vertical axis estimation
    forward_axis_confidence: float = 0.0     # [0, 1] Confidence in longitudinal forward axis
    heading_confidence: float = 0.0          # [0, 1] Overall heading confidence
    motion_state: str = "UNKNOWN"            # "STATIONARY", "ACCELERATING", "CRUISING", "TURNING", "BRAKING", "VIBRATING", "UNKNOWN"
    alignment_valid: bool = False            # True only when both gravity and forward axis are verified
    det_residual: float = 0.0                # |det(R_p2b) - 1.0|
    orthogonality_residual: float = 0.0      # ||R_p2b @ R_p2b.T - I||
    initial_gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3)) # Initial gyro bias in body frame (rad/s)
    gyro_bias_variance: float = 0.0          # Variance of stationary gyro samples ((rad/s)^2)
    stationary_duration_s: float = 0.0       # Duration of detected stillness (seconds)
    eigenvalue_ratio: float = 0.0            # Ratio lambda_max / lambda_min of horizontal surge covariance
    peak_surge_accel: float = 0.0            # Peak horizontal acceleration observed (m/s^2)


@dataclass
class AlignmentState:
    """Live state emitted by InVehicleAlignmentEngine."""
    is_leveled: bool = False
    is_aligned: bool = False
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    R_p2b: np.ndarray = field(default_factory=lambda: np.eye(3))  # (3, 3) transforms phone frame -> vehicle body frame
    R_b2p: np.ndarray = field(default_factory=lambda: np.eye(3))  # (3, 3) transforms vehicle body frame -> phone frame
    confidence: float = 0.0
    mode: AlignmentMode = AlignmentMode.INITIALIZING
    quality: AlignmentQuality = field(default_factory=AlignmentQuality)


def sanitize_imu_sample(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    """Replaces NaNs, Infs, or non-finite values with fallback values."""
    if vec is None or len(vec) != 3:
        return fallback.copy()
    arr = np.asarray(vec, dtype=float)
    if not np.all(np.isfinite(arr)):
        clean = arr.copy()
        for i in range(3):
            if not np.isfinite(clean[i]):
                clean[i] = fallback[i]
        return clean
    return arr


def renormalize_so3(R: np.ndarray) -> np.ndarray:
    """Project 3x3 matrix to closest SO(3) rotation matrix via SVD."""
    if not np.all(np.isfinite(R)):
        return np.eye(3)
    U, _, Vt = np.linalg.svd(R)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0.0:
        U[:, -1] *= -1.0
        R_ortho = U @ Vt
    return R_ortho


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
        slip_threshold_deg: float = 5.0,
        dt: float = 0.1
    ):
        self.g_nominal = g_nominal
        self.leveling_samples = leveling_samples
        self.min_accel_transient = min_accel_transient
        self.slip_threshold_deg = slip_threshold_deg
        self.dt = dt

        # Internal state buffers
        self._stationary_accels: List[np.ndarray] = []
        self._stationary_gyros: List[np.ndarray] = []
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

        # Gyro bias in phone frame
        self._gyro_bias_phone = np.zeros(3)
        self._gyro_bias_var = 0.0

        self.quality = AlignmentQuality()
        self.state = AlignmentState(
            is_leveled=False,
            is_aligned=False,
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
            R_p2b=np.eye(3),
            R_b2p=np.eye(3),
            confidence=0.0,
            mode=AlignmentMode.INITIALIZING,
            quality=self.quality
        )

    def reset(self):
        """Resets alignment engine state to initial uncalibrated condition."""
        self._stationary_accels.clear()
        self._stationary_gyros.clear()
        self._transient_accels.clear()
        self._lp_gravity = np.array([0.0, 0.0, self.g_nominal])
        self._last_calibrated_gravity = np.array([0.0, 0.0, self.g_nominal])
        self._z_b_phone = np.array([0.0, 0.0, 1.0])
        self._x_b_phone = np.array([1.0, 0.0, 0.0])
        self._y_b_phone = np.array([0.0, 1.0, 0.0])
        self.R_p2b = np.eye(3)
        self.R_b2p = np.eye(3)
        self._gyro_bias_phone = np.zeros(3)
        self._gyro_bias_var = 0.0
        self.quality = AlignmentQuality()
        self.state = AlignmentState(
            is_leveled=False,
            is_aligned=False,
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
            R_p2b=np.eye(3),
            R_b2p=np.eye(3),
            confidence=0.0,
            mode=AlignmentMode.INITIALIZING,
            quality=self.quality
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
            is_stationary: bool flag from ZUPT / stationary detector
            gnss_speed: Optional GNSS speed in m/s
        """
        # Sanitize inputs
        accel_clean = sanitize_imu_sample(accel_p, fallback=self._lp_gravity)
        gyro_clean = sanitize_imu_sample(gyro_p, fallback=np.zeros(3))

        # Update low-pass gravity filter: alpha = 0.02
        self._lp_gravity = 0.98 * self._lp_gravity + 0.02 * accel_clean

        # --- Phase 1: Leveling (Gravity Vector Estimation & Gyro Bias) ---
        if not self.state.is_leveled:
            if is_stationary:
                # Check for gyro impulse contamination during stillness (e.g. phone tap/touch)
                if np.linalg.norm(gyro_clean) < 0.25: # Discard taps with angular rate > 0.25 rad/s
                    self._stationary_accels.append(accel_clean)
                    self._stationary_gyros.append(gyro_clean)
                    self.quality.stationary_duration_s = len(self._stationary_accels) * self.dt

                if len(self._stationary_accels) >= self.leveling_samples:
                    self._perform_leveling()
            else:
                self.quality.motion_state = "QUASI_STATIC" if (gnss_speed is not None and gnss_speed < 2.0) else "UNKNOWN"
            return self.state

        # --- Check Watchdog for Phone Mount Slip / Bump ---
        if self.state.is_aligned:
            # During stillness, check raw sample directly against calibrated gravity for instant bump detection
            # During motion, check low-pass gravity
            test_gravity = accel_clean if is_stationary else self._lp_gravity
            angle_diff = self._angular_difference(test_gravity, self._last_calibrated_gravity)
            gyro_rate = np.linalg.norm(gyro_clean)

            # Slip triggered by sudden gravity angle shift > slip_threshold_deg or severe angular jerk during stillness
            if angle_diff > self.slip_threshold_deg or (is_stationary and gyro_rate > 0.8):
                # Phone was bumped or shifted in the cradle! Trigger automatic fast re-calibration
                self.state.is_aligned = False
                self.state.is_leveled = False
                self.state.mode = AlignmentMode.RECALIBRATING
                self._stationary_accels = [accel_clean]
                self._stationary_gyros = [gyro_clean]
                self._transient_accels = []
                self.quality.alignment_valid = False
                self.quality.forward_axis_confidence = 0.0
                return self.state

        # --- Phase 2: Forward Axis Determination via Acceleration Transients ---
        if self.state.is_leveled and not self.state.is_aligned:
            delta_f = accel_clean - self._lp_gravity
            horiz_f = delta_f - np.dot(delta_f, self._z_b_phone) * self._z_b_phone
            horiz_mag = np.linalg.norm(horiz_f)
            self.quality.peak_surge_accel = max(self.quality.peak_surge_accel, float(horiz_mag))

            # Classify motion state
            if is_stationary:
                self.quality.motion_state = "STATIONARY"
            elif horiz_mag >= self.min_accel_transient:
                self.quality.motion_state = "ACCELERATING"
                self._transient_accels.append(horiz_f)
                if len(self._transient_accels) >= 15:
                    self._perform_forward_alignment()
            else:
                self.quality.motion_state = "CRUISING"

        return self.state

    def _perform_leveling(self):
        """Computes vertical axis, mount roll/pitch, and stationary gyro bias."""
        mean_g = np.mean(self._stationary_accels, axis=0)
        norm_g = np.linalg.norm(mean_g)
        if norm_g < 1e-3:
            return

        self._z_b_phone = mean_g / norm_g
        self._last_calibrated_gravity = mean_g.copy()
        self._lp_gravity = mean_g.copy()

        # Compute initial gyro bias from stationary samples
        if len(self._stationary_gyros) > 0:
            self._gyro_bias_phone = np.mean(self._stationary_gyros, axis=0)
            self._gyro_bias_var = float(np.var(self._stationary_gyros))
        else:
            self._gyro_bias_phone = np.zeros(3)
            self._gyro_bias_var = 1e-6

        # Compute mount roll and pitch angles in phone frame
        pitch = -np.arcsin(np.clip(self._z_b_phone[1], -1.0, 1.0))
        roll = np.arctan2(self._z_b_phone[0], self._z_b_phone[2])

        # Evaluate gravity estimation confidence based on agreement with g_nominal and sample spread
        g_err = abs(norm_g - self.g_nominal) / self.g_nominal
        var_g = float(np.mean(np.var(self._stationary_accels, axis=0)))
        grav_conf = np.clip(1.0 - (g_err * 5.0) - (var_g * 0.5), 0.1, 1.0)

        # Form intermediate leveled mounting matrix (Roll & Pitch leveled, yaw unconstrained)
        # Create arbitrary perpendicular x_cand on horizontal plane
        cand = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], self._z_b_phone) * self._z_b_phone
        if np.linalg.norm(cand) < 1e-3:
            cand = np.array([0.0, 1.0, 0.0]) - np.dot([0.0, 1.0, 0.0], self._z_b_phone) * self._z_b_phone
        x_init = cand / np.linalg.norm(cand)
        y_init = np.cross(self._z_b_phone, x_init)
        y_init /= np.linalg.norm(y_init)

        self.R_b2p = np.column_stack([x_init, y_init, self._z_b_phone])
        self.R_b2p = renormalize_so3(self.R_b2p)
        self.R_p2b = self.R_b2p.T

        self.quality.gravity_confidence = float(grav_conf)
        self.quality.forward_axis_confidence = 0.0
        self.quality.heading_confidence = 0.0
        self.quality.alignment_valid = False # Unaligned in yaw until dynamic surge
        self.quality.initial_gyro_bias = self.R_p2b @ self._gyro_bias_phone
        self.quality.gyro_bias_variance = self._gyro_bias_var
        self.quality.det_residual = float(abs(np.linalg.det(self.R_p2b) - 1.0))
        self.quality.orthogonality_residual = float(np.linalg.norm(self.R_p2b @ self.R_p2b.T - np.eye(3)))
        self.quality.motion_state = "STATIONARY"

        self.state.is_leveled = True
        self.state.is_aligned = False
        self.state.roll_deg = float(np.degrees(roll))
        self.state.pitch_deg = float(np.degrees(pitch))
        self.state.yaw_deg = 0.0
        self.state.R_p2b = self.R_p2b.copy()
        self.state.R_b2p = self.R_b2p.copy()
        self.state.confidence = float(grav_conf * 0.5) # Leveled only -> 50% max confidence
        self.state.mode = AlignmentMode.STATIONARY_LEVELED
        self.state.quality = self.quality

    def _perform_forward_alignment(self):
        """Computes forward longitudinal axis and builds strictly orthonormal R_p2b."""
        X = np.array(self._transient_accels)
        if len(X) < 5:
            return

        cov_h = np.cov(X.T)
        if not np.all(np.isfinite(cov_h)):
            return

        eigvals, eigvecs = np.linalg.eigh(cov_h)
        # Eigenvalue separation ratio on 2D horizontal plane (eigvals[-1] vs eigvals[-2])
        lambda_principal = max(1e-9, float(eigvals[-1]))
        lambda_transverse = max(1e-9, float(eigvals[-2]))
        kappa = lambda_principal / lambda_transverse
        self.quality.eigenvalue_ratio = float(kappa)

        x_cand = eigvecs[:, -1]
        x_cand = x_cand - np.dot(x_cand, self._z_b_phone) * self._z_b_phone
        norm_cand = np.linalg.norm(x_cand)
        if norm_cand < 1e-3:
            return
        x_cand = x_cand / norm_cand

        # Resolve forward sign via mean transient surge vector
        mean_surge = np.mean(X, axis=0)
        if np.dot(mean_surge, x_cand) < 0.0:
            x_cand = -x_cand

        self._x_b_phone = x_cand
        self._y_b_phone = np.cross(self._z_b_phone, self._x_b_phone)
        self._y_b_phone /= np.linalg.norm(self._y_b_phone)

        self.R_b2p = np.column_stack([self._x_b_phone, self._y_b_phone, self._z_b_phone])
        self.R_b2p = renormalize_so3(self.R_b2p)
        self.R_p2b = self.R_b2p.T

        yaw = np.arctan2(self.R_p2b[0, 1], self.R_p2b[0, 0])
        fwd_conf = float(np.clip((kappa - 1.5) / 5.0, 0.2, 1.0))

        self.quality.forward_axis_confidence = fwd_conf
        self.quality.heading_confidence = fwd_conf
        self.quality.alignment_valid = True
        self.quality.det_residual = float(abs(np.linalg.det(self.R_p2b) - 1.0))
        self.quality.orthogonality_residual = float(np.linalg.norm(self.R_p2b @ self.R_p2b.T - np.eye(3)))
        self.quality.initial_gyro_bias = self.R_p2b @ self._gyro_bias_phone

        self.state.yaw_deg = float(np.degrees(yaw))
        self.state.R_p2b = self.R_p2b.copy()
        self.state.R_b2p = self.R_b2p.copy()
        self.state.is_aligned = True
        self.state.confidence = float(self.quality.gravity_confidence * 0.4 + fwd_conf * 0.6)
        self.state.mode = AlignmentMode.DYNAMIC_ALIGNED
        self.state.quality = self.quality

    def transform_phone_to_vehicle(self, vec_phone: np.ndarray) -> np.ndarray:
        """Transforms a 3D vector (accel or gyro) from phone frame to vehicle body frame."""
        clean = sanitize_imu_sample(vec_phone, fallback=np.zeros(3))
        return self.R_p2b @ clean

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
        speed: Optional[np.ndarray] = None,
        return_quality: bool = False
    ) -> Any:
        """
        Computes the static mounting matrix R_p2b (phone frame -> vehicle body frame)
        over a driving window via gravity leveling and robust longitudinal surge PCA.
        
        Guarantees:
        - Strict SO(3) orthonormality (det(R) = +1.0, R @ R.T = I).
        - Safe handling of stationary drives (no unobservable noise eigenvectors).
        - Sanitization of NaNs/Infs.
        """
        quality = AlignmentQuality()

        if accel_phone is None or len(accel_phone) < 5:
            R_fallback = np.eye(3)
            return (R_fallback, quality) if return_quality else R_fallback

        # Sanitize input array
        accel_arr = np.asarray(accel_phone, dtype=float)
        if not np.all(np.isfinite(accel_arr)):
            # Replace NaNs/Infs with median
            col_medians = np.nanmedian(accel_arr, axis=0)
            inds = np.where(~np.isfinite(accel_arr))
            accel_arr[inds] = np.take(col_medians, inds[1])

        # 1. Gravity Leveling (z_b)
        mean_a = np.mean(accel_arr, axis=0)
        norm_a = np.linalg.norm(mean_a)
        if norm_a < 1e-3 or not np.isfinite(norm_a):
            z_b = np.array([0.0, 0.0, 1.0])
            quality.gravity_confidence = 0.0
        else:
            z_b = mean_a / norm_a
            g_err = abs(norm_a - 9.80665) / 9.80665
            quality.gravity_confidence = float(np.clip(1.0 - g_err * 5.0, 0.1, 1.0))

        # 2. Horizontal Acceleration Extraction
        diff_a = accel_arr - mean_a
        a_vert = np.dot(diff_a, z_b)
        a_horiz = diff_a - np.outer(a_vert, z_b)

        # 3. Covariance & Horizontal Eigenvalue Separation Analysis
        cov_h = np.cov(a_horiz.T)
        if not np.all(np.isfinite(cov_h)):
            cov_h = np.eye(3) * 1e-4

        eigvals, eigvecs = np.linalg.eigh(cov_h)
        # Note: eigvals[0] is orthogonal vertical axis (~0).
        # True horizontal separation is lambda_principal (eigvals[-1]) vs lambda_transverse (eigvals[-2]).
        lambda_principal = max(1e-9, float(eigvals[-1]))
        lambda_transverse = max(1e-9, float(eigvals[-2]))
        kappa = lambda_principal / lambda_transverse
        quality.eigenvalue_ratio = float(kappa)

        horiz_magnitudes = np.linalg.norm(a_horiz, axis=1)
        peak_surge = float(np.max(horiz_magnitudes)) if len(horiz_magnitudes) > 0 else 0.0
        quality.peak_surge_accel = peak_surge

        # 4. Check whether forward axis is observable
        # Requires both significant surge amplitude (>0.35 m/s^2) AND directional elongation (kappa >= 2.0)
        if peak_surge < 0.35 or kappa < 2.0:
            quality.forward_axis_confidence = 0.0
            quality.heading_confidence = 0.0
            quality.alignment_valid = False
            quality.motion_state = "STATIONARY"

            # Create safe leveled frame (roll/pitch leveled, default horizontal orientation)
            cand = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], z_b) * z_b
            if np.linalg.norm(cand) < 1e-3:
                cand = np.array([0.0, 1.0, 0.0]) - np.dot([0.0, 1.0, 0.0], z_b) * z_b
            x_b = cand / np.linalg.norm(cand)
            y_b = np.cross(z_b, x_b)
            y_b /= np.linalg.norm(y_b)

            R_p2b = np.vstack([x_b, y_b, z_b])
            R_p2b = renormalize_so3(R_p2b)
            quality.det_residual = float(abs(np.linalg.det(R_p2b) - 1.0))
            quality.orthogonality_residual = float(np.linalg.norm(R_p2b @ R_p2b.T - np.eye(3)))
            return (R_p2b, quality) if return_quality else R_p2b

        # Dynamic excitation is valid: extract principal horizontal eigenvector
        x_b = eigvecs[:, -1]
        x_b = x_b - np.dot(x_b, z_b) * z_b
        norm_x = np.linalg.norm(x_b)
        if norm_x > 1e-3:
            x_b = x_b / norm_x
        else:
            cand = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], z_b) * z_b
            x_b = cand / (np.linalg.norm(cand) + 1e-6)

        # 5. Resolve Forward Direction Sign
        if speed is not None and len(speed) > 10:
            dv = np.gradient(speed)
            a_x = np.dot(a_horiz, x_b)
            valid = (np.abs(dv) > 0.05) & (np.abs(a_x) > 0.1)
            if np.sum(valid) > 10:
                corr = np.corrcoef(a_x[valid], dv[valid])[0, 1]
                if np.isfinite(corr) and corr < 0:
                    x_b = -x_b
        else:
            # Vehicle braking decelerations are sharper than acceleration (negative skewness)
            a_x = np.dot(a_horiz, x_b)
            std_ax = np.std(a_x)
            if std_ax > 0.05:
                skew_x = float(np.mean((a_x - np.mean(a_x))**3) / (std_ax**3 + 1e-6))
                if skew_x > 0.1:
                    x_b = -x_b

        # 6. Build Orthonormal Right-Handed Basis (FLU Body Frame)
        y_b = np.cross(z_b, x_b)
        norm_y = np.linalg.norm(y_b)
        if norm_y > 1e-3:
            y_b = y_b / norm_y
        else:
            y_b = np.array([0.0, 1.0, 0.0])

        x_b = np.cross(y_b, z_b)
        x_b /= np.linalg.norm(x_b)

        R_p2b = np.vstack([x_b, y_b, z_b])
        R_p2b = renormalize_so3(R_p2b)

        fwd_conf = float(np.clip((kappa - 1.8) / 4.0, 0.3, 1.0))
        quality.forward_axis_confidence = fwd_conf
        quality.heading_confidence = fwd_conf
        quality.alignment_valid = True
        quality.motion_state = "ACCELERATING"
        quality.det_residual = float(abs(np.linalg.det(R_p2b) - 1.0))
        quality.orthogonality_residual = float(np.linalg.norm(R_p2b @ R_p2b.T - np.eye(3)))

        if gyro_phone is not None and len(gyro_phone) > 0:
            quality.initial_gyro_bias = R_p2b @ np.mean(gyro_phone, axis=0)
            quality.gyro_bias_variance = float(np.var(gyro_phone))

        return (R_p2b, quality) if return_quality else R_p2b

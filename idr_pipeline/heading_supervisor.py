"""
idr_pipeline/heading_supervisor.py
==================================
Multi-Source Attitude & Heading Observability Supervisor with Disturbance Rejection,
Innovation Gating, and Gyro Bias Observability Enhancement.

Standards & Conventions:
------------------------
- Navigation Frame: Local Cartesian ENU (East=+X, North=+Y, Up=+Z).
- Vehicle Body Frame: FLU (Forward=+X, Left=+Y, Up=+Z).
- Phone Body Frame: Triple-axis accelerometer/gyro/mag sensor frame.
- Heading Angle (psi): Azimuth clockwise from true North [0, 2*pi) radians or [0, 360) degrees.
- Attitude Error State: Right-handed ENU rotation vector delta_theta^n in radians.
- Heading Measurement Jacobian: H[0, 8] = -1.0 (since delta_yaw = -delta_theta_z^n in ENU).

Author: Autonomous Navigation Engineering Team
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any, List

from .eskf_fusion import ErrorStateKalmanFilter, wrap_pi, rotation_matrix_to_euler, rodrigues


class HeadingQuality(str, Enum):
    """Quality classification for heading measurements."""
    INVALID = "INVALID"              # Missing, NaN, zero magnitude, or uncalibrated
    DISTURBED = "DISTURBED"          # Magnetic anomaly, gradient spike, or multipath reflection
    WEAK = "WEAK"                    # Low speed, elevated dynamic noise, or wide uncertainty
    NOMINAL = "NOMINAL"              # Trusted measurement within physical thresholds
    HIGH_ACCURACY = "HIGH_ACCURACY"  # High-confidence cruise GNSS course or multi-sensor consensus


class HeadingSourceType(str, Enum):
    """Source identifier for heading aiding."""
    GYRO_INTEGRATION = "GYRO_INTEGRATION"
    GNSS_COURSE = "GNSS_COURSE"
    MAGNETOMETER = "MAGNETOMETER"
    DUAL_ANTENNA = "DUAL_ANTENNA"
    MOTION_CONSTRAINTS = "MOTION_CONSTRAINTS"


@dataclass
class HeadingMeasurement:
    """Individual heading measurement candidate with diagnostics."""
    source: HeadingSourceType
    heading_rad: float                 # [0, 2*pi) azimuth clockwise from North
    heading_deg: float                 # [0, 360) degrees
    std_rad: float                     # Expected 1-sigma standard deviation (rad)
    quality: HeadingQuality
    raw_magnitude: float               # Context metric (e.g. speed m/s or mag field uT)
    innovation_rad: Optional[float] = None  # dz = wrap_pi(meas - filter)
    innovation_deg: Optional[float] = None
    innovation_variance: Optional[float] = None # S = H P H^T + R
    chi2_normalized: Optional[float] = None     # gamma = dz^2 / S
    is_accepted: bool = False
    rejection_reason: str = ""


@dataclass
class HeadingSupervisorTelemetry:
    """Real-time telemetry emitted by the Heading Supervisor."""
    time_s: float
    filter_heading_deg: float
    selected_source: HeadingSourceType
    selected_quality: HeadingQuality
    measurement_heading_deg: Optional[float]
    innovation_deg: Optional[float]
    chi2_normalized: Optional[float]
    is_updated: bool
    rejection_reason: str
    outage_duration_s: float
    total_updates_count: int
    total_rejections_count: int
    gnss_course_available: bool
    mag_available: bool
    mag_field_magnitude_uT: float
    mag_disturbed: bool
    gyro_bias_z_deg_s: float
    heading_uncertainty_deg: float


class MagneticDisturbanceDetector:
    """
    3D Geomagnetic Disturbance & Anomaly Detector with Tilt Compensation.
    
    Protects the ESKF from vehicle chassis steel soft/hard-iron distortions,
    bridge infrastructure, power lines, and internal speaker current spikes.
    """

    def __init__(
        self,
        reference_field_uT: float = 48.0,
        magnitude_tolerance_uT: float = 12.0,
        max_gradient_uT_s: float = 35.0,
        min_field_uT: float = 5.0,
        max_field_uT: float = 250.0,
        declination_rad: float = 0.0,
        hard_iron_bias: Optional[np.ndarray] = None,
        soft_iron_matrix: Optional[np.ndarray] = None
    ):
        self.ref_field = reference_field_uT
        self.mag_tol = magnitude_tolerance_uT
        self.max_grad = max_gradient_uT_s
        self.min_field = min_field_uT
        self.max_field = max_field_uT
        self.declination = declination_rad
        
        self.hard_iron = hard_iron_bias if hard_iron_bias is not None else np.zeros(3)
        self.soft_iron = soft_iron_matrix if soft_iron_matrix is not None else np.eye(3)

        self.last_mag_vec: Optional[np.ndarray] = None
        self.last_time_s: Optional[float] = None

    def reset(self):
        """Resets temporal tracking state."""
        self.last_mag_vec = None
        self.last_time_s = None

    def evaluate_measurement(
        self,
        mag_raw_p: np.ndarray,
        time_s: float,
        R_p2b: np.ndarray,
        R_b2n: np.ndarray
    ) -> Tuple[HeadingQuality, float, float, float, str]:
        """
        Evaluates raw 3D magnetometer sample from smartphone, checks for disturbances,
        and computes tilt-compensated magnetic heading.
        
        Returns:
            quality: HeadingQuality enum
            heading_rad: Azimuth in [0, 2*pi)
            std_rad: Measurement standard deviation
            field_norm: Total measured field magnitude in uT
            reason: Diagnostic explanation
        """
        # 1. Basic sanity checks (NaN, inf, dimensions)
        if mag_raw_p is None or len(mag_raw_p) != 3 or not np.all(np.isfinite(mag_raw_p)):
            return HeadingQuality.INVALID, 0.0, 1.0, 0.0, "INVALID_OR_NAN_INPUT"

        # 2. Hard-iron and soft-iron calibration correction
        mag_cal_p = self.soft_iron @ (mag_raw_p - self.hard_iron)
        norm_cal = float(np.linalg.norm(mag_cal_p))

        # 3. Absolute magnitude bounds check
        if norm_cal < self.min_field or norm_cal > self.max_field:
            return HeadingQuality.INVALID, 0.0, 1.0, norm_cal, f"MAGNITUDE_OUT_OF_BOUNDS_{norm_cal:.1f}uT"

        # 4. Temporal gradient check (rate-of-change)
        dt = (time_s - self.last_time_s) if self.last_time_s is not None else 0.1
        if dt > 0.0 and self.last_mag_vec is not None:
            gradient = float(np.linalg.norm(mag_cal_p - self.last_mag_vec) / dt)
            if gradient > self.max_grad:
                self.last_mag_vec = mag_cal_p.copy()
                self.last_time_s = time_s
                return HeadingQuality.DISTURBED, 0.0, 1.0, norm_cal, f"EXCESSIVE_GRADIENT_{gradient:.1f}uT/s"

        self.last_mag_vec = mag_cal_p.copy()
        self.last_time_s = time_s

        # 5. Reference field anomaly check
        mag_delta = abs(norm_cal - self.ref_field)
        if mag_delta > self.mag_tol:
            return HeadingQuality.DISTURBED, 0.0, 1.0, norm_cal, f"MAGNITUDE_ANOMALY_DELTA_{mag_delta:.1f}uT"

        # 6. Transform calibrated magnetometer to body frame, then navigation frame (tilt compensation)
        mag_b = R_p2b @ mag_cal_p
        mag_n = R_b2n @ mag_b

        # In ENU frame:
        # East = +X, North = +Y, Up = +Z.
        # Earth magnetic field vector in Northern hemisphere points North (mag_n[1] > 0) and Down (mag_n[2] < 0).
        # Heading psi is clockwise from North:
        # If facing North, vehicle +X points North, mag_n = [0, B_h, -B_v] -> psi = 0.
        # If facing East, vehicle +X points East, mag_n = [0, B_h, -B_v] in body frame...
        # In navigation frame, the vehicle attitude R_b2n transforms body vector to ENU.
        # Tilt-compensated horizontal magnetic components in body horizontal plane:
        # Using Euler roll (phi) and pitch (theta):
        # We can directly extract heading from the horizontal projection in navigation frame:
        # For a level sensor facing azimuth psi:
        # mag_b = R_z(-psi) * [0, B_h, -B_v]^T = [B_h * sin(psi), B_h * cos(psi), -B_v]^T
        # Thus: B_bx / B_by = sin(psi) / cos(psi) = tan(psi) -> psi = atan2(B_bx_level, B_by_level)
        # Using full 3D attitude matrix R_b2n to level mag_b:
        # mag_level = (R_horizontal_leveling) @ mag_b
        # Or equivalently from R_b2n:
        # In FLU body frame: X=Forward, Y=Left, Z=Up
        # Forward vector in horizontal plane has components along mag_n:
        # Standard tilt-compensated horizontal components from body vector:
        Bx, By, Bz = mag_b[0], mag_b[1], mag_b[2]
        pitch = float(np.arcsin(np.clip(R_b2n[2, 0], -1.0, 1.0)))
        roll = float(np.arctan2(R_b2n[2, 1], R_b2n[2, 2]))

        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)

        # Tilt compensation in FLU body frame:
        # Leveling forward component:
        B_fx = Bx * cp + By * sp * sr + Bz * sp * cr
        # Leveling left component:
        B_fy = By * cr - Bz * sr

        # In FLU body frame, Left (+Y) is 90 deg CCW from Forward (+X).
        # When facing North (psi=0), Earth magnetic field points along Forward (+X), so B_fx > 0, B_fy = 0.
        # When facing East (psi=90 deg), Earth magnetic field points North, which is along Left (+Y), so B_fy > 0.
        # Azimuth clockwise from North: psi_mag = atan2(B_fy, B_fx)
        psi_mag = float(np.arctan2(B_fy, B_fx)) + self.declination
        psi_mag = float(psi_mag % (2.0 * np.pi))

        # 7. Assign quality and standard deviation
        if mag_delta < 4.0:
            quality = HeadingQuality.NOMINAL
            std_rad = float(np.radians(4.0)) # ~0.07 rad (4 deg 1-sigma)
        else:
            quality = HeadingQuality.WEAK
            std_rad = float(np.radians(10.0)) # ~0.17 rad (10 deg 1-sigma)

        return quality, psi_mag, std_rad, norm_cal, "VALID_MAGNETIC_MEASUREMENT"


class GNSSCourseDeriver:
    """
    Physically-Justified GNSS Velocity Course-Over-Ground (COG) Deriver.
    
    Derives vehicle heading directly from East-North velocity components with
    rigorous speed thresholds and dynamic yaw rate gating.
    """

    def __init__(
        self,
        min_speed_mps: float = 2.5,          # 9 km/h minimum speed for course observability
        max_yaw_rate_rad_s: float = 0.75,     # Reject or widen noise during fast cornering (phase lag)
        base_velocity_noise_mps: float = 0.15 # Baseline GNSS velocity accuracy
    ):
        self.min_speed = min_speed_mps
        self.max_yaw_rate = max_yaw_rate_rad_s
        self.base_vel_noise = base_velocity_noise_mps

    def derive_course(
        self,
        vel_enu: np.ndarray,
        current_yaw_rate: float = 0.0,
        is_forward: bool = True
    ) -> Tuple[HeadingQuality, float, float, float, str]:
        """
        Computes course heading from ENU velocity vector [v_east, v_north, v_up].
        
        Returns:
            quality: HeadingQuality enum
            course_rad: Azimuth in [0, 2*pi)
            std_rad: Measurement noise standard deviation
            speed_mps: Horizontal speed
            reason: Diagnostic explanation
        """
        if vel_enu is None or len(vel_enu) < 2 or not np.all(np.isfinite(vel_enu[:2])):
            return HeadingQuality.INVALID, 0.0, 1.0, 0.0, "INVALID_VELOCITY_VECTOR"

        ve, vn = float(vel_enu[0]), float(vel_enu[1])
        speed_horiz = float(np.sqrt(ve**2 + vn**2))

        # Speed threshold gating: At low speed, heading noise sigma = sigma_v / v blows up
        if speed_horiz < self.min_speed:
            return (
                HeadingQuality.INVALID,
                0.0,
                1.0,
                speed_horiz,
                f"REJECTED_LOW_SPEED_{speed_horiz:.2f}m/s_BELOW_{self.min_speed}m/s"
            )

        # Azimuth clockwise from North in ENU: atan2(v_east, v_north)
        course_rad = float(np.arctan2(ve, vn))
        if not is_forward:
            # If vehicle is reversing, course is opposite to vehicle heading
            course_rad += np.pi
        course_rad = float(course_rad % (2.0 * np.pi))

        # Analytical course noise: sigma_psi = sigma_v / v_h
        sigma_psi = float(np.clip(self.base_vel_noise / speed_horiz, np.radians(1.0), np.radians(15.0)))

        # Turn dynamic check
        if abs(current_yaw_rate) > self.max_yaw_rate:
            return (
                HeadingQuality.WEAK,
                course_rad,
                sigma_psi * 2.5,
                speed_horiz,
                f"HIGH_YAW_RATE_DYNAMICS_{abs(current_yaw_rate):.2f}rad/s"
            )

        quality = HeadingQuality.HIGH_ACCURACY if speed_horiz > 8.0 else HeadingQuality.NOMINAL
        return quality, course_rad, sigma_psi, speed_horiz, "VALID_GNSS_COURSE"


class HeadingSupervisor:
    """
    Multi-Source Heading Supervisor & Innovation Gating Engine.
    
    Orchestrates GNSS Course, Magnetometer, and Gyro Dead Reckoning aiding
    with Chi-Square innovation gating, angular wrap protection, and
    gyro z bias observability enhancement.
    """

    def __init__(
        self,
        chi2_gate: float = 6.635,            # 99% confidence for 1-DOF heading innovation
        min_gnss_speed_mps: float = 2.5,
        ref_mag_field_uT: float = 48.0,
        enable_magnetometer: bool = True,
        enable_gnss_course: bool = True
    ):
        self.chi2_gate = chi2_gate
        self.enable_mag = enable_magnetometer
        self.enable_gnss = enable_gnss_course

        self.mag_detector = MagneticDisturbanceDetector(reference_field_uT=ref_mag_field_uT)
        self.gnss_deriver = GNSSCourseDeriver(min_speed_mps=min_gnss_speed_mps)

        # Statistics & telemetry
        self.total_updates = 0
        self.total_rejections = 0
        self.last_update_time_s = 0.0
        self.outage_duration_s = 0.0
        self.reacquisition_count = 0
        self.last_candidate_heading_rad: Optional[float] = None
        self.rejection_history: List[str] = []

    def reset(self):
        """Resets supervisor state and counters."""
        self.mag_detector.reset()
        self.total_updates = 0
        self.total_rejections = 0
        self.last_update_time_s = 0.0
        self.outage_duration_s = 0.0
        self.reacquisition_count = 0
        self.last_candidate_heading_rad = None
        self.rejection_history.clear()

    def process_step(
        self,
        time_s: float,
        eskf: ErrorStateKalmanFilter,
        R_p2b: np.ndarray,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        mag_raw_p: Optional[np.ndarray] = None,
        current_yaw_rate: float = 0.0,
        is_forward: bool = True
    ) -> HeadingSupervisorTelemetry:
        """
        Executes one heading supervision cycle.
        
        Evaluates candidate sources, selects optimal measurement, performs
        Chi-Square innovation gating with angular wrapping, and updates ESKF.
        """
        candidates: List[HeadingMeasurement] = []
        mag_field_norm = 0.0
        mag_is_disturbed = False

        # 1. Evaluate GNSS Course Aiding
        if self.enable_gnss and gnss_valid and gnss_vel is not None:
            g_qual, g_course, g_std, g_spd, g_reason = self.gnss_deriver.derive_course(
                vel_enu=gnss_vel,
                current_yaw_rate=current_yaw_rate,
                is_forward=is_forward
            )
            if g_qual in (HeadingQuality.NOMINAL, HeadingQuality.HIGH_ACCURACY, HeadingQuality.WEAK):
                candidates.append(HeadingMeasurement(
                    source=HeadingSourceType.GNSS_COURSE,
                    heading_rad=g_course,
                    heading_deg=float(np.degrees(g_course)),
                    std_rad=g_std,
                    quality=g_qual,
                    raw_magnitude=g_spd,
                    rejection_reason=g_reason
                ))

        # 2. Evaluate Magnetometer Aiding
        if self.enable_mag and mag_raw_p is not None:
            m_qual, m_hdg, m_std, m_norm, m_reason = self.mag_detector.evaluate_measurement(
                mag_raw_p=mag_raw_p,
                time_s=time_s,
                R_p2b=R_p2b,
                R_b2n=eskf.R
            )
            mag_field_norm = m_norm
            mag_is_disturbed = (m_qual == HeadingQuality.DISTURBED)

            if m_qual in (HeadingQuality.NOMINAL, HeadingQuality.WEAK):
                candidates.append(HeadingMeasurement(
                    source=HeadingSourceType.MAGNETOMETER,
                    heading_rad=m_hdg,
                    heading_deg=float(np.degrees(m_hdg)),
                    std_rad=m_std,
                    quality=m_qual,
                    raw_magnitude=m_norm,
                    rejection_reason=m_reason
                ))

        # 3. Source Selection Arbitration
        # Preference: HIGH_ACCURACY GNSS > NOMINAL GNSS > NOMINAL MAG > WEAK GNSS > WEAK MAG
        selected: Optional[HeadingMeasurement] = None
        if candidates:
            # Sort by quality precedence, then smallest std_rad
            quality_rank = {
                HeadingQuality.HIGH_ACCURACY: 0,
                HeadingQuality.NOMINAL: 1,
                HeadingQuality.WEAK: 2
            }
            candidates.sort(key=lambda c: (quality_rank.get(c.quality, 99), c.std_rad))
            selected = candidates[0]

        # 4. Innovation Gating & Kalman Update
        is_updated = False
        rejection_reason = "NO_VALID_HEADING_SOURCE"
        selected_source = HeadingSourceType.GYRO_INTEGRATION
        selected_quality = HeadingQuality.INVALID
        meas_hdg_deg = None
        innov_deg = None
        gamma_val = None

        if selected is not None:
            selected_source = selected.source
            selected_quality = selected.quality
            meas_hdg_deg = selected.heading_deg

            # Angular innovation with [-pi, +pi] wrapping
            dyaw = float(np.arctan2(np.sin(selected.heading_rad - eskf.yaw), np.cos(selected.heading_rad - eskf.yaw)))
            innov_deg = float(np.degrees(dyaw))

            # Innovation covariance: S = H P H^T + R
            # With H[0, 8] = -1.0, S = (-1)^2 * P[8, 8] + std_rad^2 = P[8, 8] + std_rad^2
            H = np.zeros((1, 15))
            H[0, 8] = -1.0
            R_cov = np.array([[selected.std_rad**2]])
            S = float((H @ eskf.P @ H.T + R_cov)[0, 0])
            gamma = float((dyaw**2) / S)
            gamma_val = gamma

            selected.innovation_rad = dyaw
            selected.innovation_deg = innov_deg
            selected.innovation_variance = S
            selected.chi2_normalized = gamma

            # Chi-Square Innovation Gate check with multi-epoch re-acquisition confirmation
            if gamma <= self.chi2_gate:
                # Accept and apply Kalman update to ESKF
                dz = np.array([dyaw])
                eskf._apply_kalman_update(dz, H, R_cov)
                is_updated = True
                selected.is_accepted = True
                self.total_updates += 1
                self.last_update_time_s = time_s
                self.outage_duration_s = 0.0
                self.reacquisition_count = 0
                rejection_reason = "ACCEPTED_AND_UPDATED"
            else:
                # Innovation exceeded gate: check if recovering from outage
                if self.outage_duration_s > 1.5:
                    if (
                        self.last_candidate_heading_rad is not None
                        and abs(wrap_pi(selected.heading_rad - self.last_candidate_heading_rad)) < np.radians(10.0)
                    ):
                        self.reacquisition_count += 1
                    else:
                        self.reacquisition_count = 1
                    self.last_candidate_heading_rad = selected.heading_rad

                    if self.reacquisition_count >= 2:
                        # 2 consecutive consistent fixes confirm legitimate post-outage turn
                        dz = np.array([dyaw])
                        eskf._apply_kalman_update(dz, H, R_cov)
                        is_updated = True
                        selected.is_accepted = True
                        self.total_updates += 1
                        self.last_update_time_s = time_s
                        self.outage_duration_s = 0.0
                        self.reacquisition_count = 0
                        rejection_reason = "ACCEPTED_REACQUISITION_CONFIRMED"
                    else:
                        self.total_rejections += 1
                        rejection_reason = f"REACQUISITION_PENDING_CONFIRMATION_COUNT_{self.reacquisition_count}"
                        self.rejection_history.append(rejection_reason)
                        selected.is_accepted = False
                else:
                    self.reacquisition_count = 0
                    self.total_rejections += 1
                    rejection_reason = f"REJECTED_CHI2_GATE_EXCEEDED_GAMMA_{gamma:.2f}_GT_{self.chi2_gate:.2f}"
                    self.rejection_history.append(rejection_reason)
                    selected.is_accepted = False

        if not is_updated:
            if self.last_update_time_s > 0.0:
                self.outage_duration_s = float(time_s - self.last_update_time_s)
            else:
                self.outage_duration_s = float(time_s)

        # Gyro z bias and heading uncertainty
        gyro_bias_z_deg_s = float(np.degrees(eskf.bg[2]))
        heading_unc_deg = float(np.degrees(np.sqrt(max(0.0, eskf.P[8, 8]))))

        return HeadingSupervisorTelemetry(
            time_s=time_s,
            filter_heading_deg=float(np.degrees(eskf.yaw) % 360.0),
            selected_source=selected_source,
            selected_quality=selected_quality,
            measurement_heading_deg=meas_hdg_deg,
            innovation_deg=innov_deg,
            chi2_normalized=gamma_val,
            is_updated=is_updated,
            rejection_reason=rejection_reason,
            outage_duration_s=self.outage_duration_s,
            total_updates_count=self.total_updates,
            total_rejections_count=self.total_rejections,
            gnss_course_available=(len(candidates) > 0 and candidates[0].source == HeadingSourceType.GNSS_COURSE),
            mag_available=(mag_raw_p is not None),
            mag_field_magnitude_uT=mag_field_norm,
            mag_disturbed=mag_is_disturbed,
            gyro_bias_z_deg_s=gyro_bias_z_deg_s,
            heading_uncertainty_deg=heading_unc_deg
        )

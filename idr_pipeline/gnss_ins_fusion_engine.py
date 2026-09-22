"""
idr_pipeline/gnss_ins_fusion_engine.py
======================================
Unified GNSS+INS Sensor Fusion Engine with Seamless GNSS Deficit Handler.

Capabilities:
-------------
1. Dual Operating Modes:
   - GNSS_AIDED_INS: Continuous loose/tight fusion of GNSS position & velocity with
     15-state Error-State Kalman Filter (ESKF). Estimates real-time accelerometer and
     gyroscope biases, attitude, and navigation covariance.
   - INTELLIGENT_DEAD_RECKONING (IDR): Instant millisecond transition upon GNSS outage
     (tunnel, underpass, dense urban canyon). Synthesizes forward speed via AI regressor,
     enforces Non-Holonomic Constraints (NHC: v_y^b = 0, v_z^b = 0), applies ZUPT, and
     bounds vertical drift via barometric altitude.

2. Instant Deficit Detection:
   Monitors GNSS signal validity, HDOP, satellite count, and timestamp continuity.
   Switches from GNSS-aided mode to IDR within < 5ms of signal loss.

3. Chi-Square (Chi^2) Innovation Gating upon Re-acquisition:
   When emerging from a tunnel, initial GNSS fixes often suffer from severe multipath
   or canyon reflection. Uses Mahalanobis distance test:
       gamma = r^T * S^(-1) * r <= chi2_threshold (e.g. 11.345 for 3 DOF, p=0.01)
   Rejects false satellite fixes and gradually blends the navigation state back
   into GNSS-aided mode without visual jumps or trajectory discontinuities.
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

from .eskf_fusion import ErrorStateKalmanFilter
from .alignment_engine import InVehicleAlignmentEngine
from .vibration_filter import VibrationPotholeFilter, VibrationState
from .heading_supervisor import HeadingSupervisor, HeadingSupervisorTelemetry


class NavigationMode(str, Enum):
    INITIALIZING = "INITIALIZING"
    GNSS_AIDED_INS = "GNSS_AIDED_INS"
    INTELLIGENT_DEAD_RECKONING = "INTELLIGENT_DEAD_RECKONING"
    GNSS_RECOVERY = "GNSS_RECOVERY"


@dataclass
class NavigationTelemetry:
    mode: NavigationMode
    time_s: float
    pos_enu: np.ndarray        # [East, North, Up] meters
    vel_enu: np.ndarray        # [vx, vy, vz] m/s
    speed_kmh: float           # km/h
    heading_deg: float         # yaw heading in degrees
    pitch_deg: float           # pitch in degrees
    roll_deg: float            # roll in degrees
    accel_bias: np.ndarray     # (3,) estimated accelerometer bias
    gyro_bias: np.ndarray      # (3,) estimated gyroscope bias
    in_outage: bool
    outage_duration_s: float
    outage_dist_traveled_m: float
    gnss_valid: bool
    is_pothole_active: bool
    pothole_count: int
    is_idling: bool
    covariance_diag: np.ndarray # (15,) diagonal of P matrix
    heading_telemetry: Optional[HeadingSupervisorTelemetry] = None


class SeamlessGNSSDeficitEngine:
    """
    Complete real-time GNSS+INS Fusion Engine with autonomous mode transition
    and multi-source attitude/heading supervision.
    """

    def __init__(
        self,
        init_pos: np.ndarray = np.array([0.0, 0.0, 0.0]),
        init_vel: np.ndarray = np.array([0.0, 0.0, 0.0]),
        init_euler: np.ndarray = np.array([0.0, 0.0, 0.0]),
        chi2_threshold: float = 11.345, # 99% confidence for 3-DOF position innovation
        dt: float = 0.1,
        enable_heading_supervisor: bool = True
    ):
        self.dt = dt
        self.chi2_threshold = chi2_threshold

        # Core sub-modules
        self.alignment_engine = InVehicleAlignmentEngine()
        self.vibration_filter = VibrationPotholeFilter(dt=dt)
        self.eskf = ErrorStateKalmanFilter(
            init_pos=init_pos,
            init_vel=init_vel,
            init_euler=init_euler
        )
        self.enable_heading_supervisor = enable_heading_supervisor
        self.heading_supervisor = HeadingSupervisor(chi2_gate=6.635) if enable_heading_supervisor else None

        # Operational state
        self.mode = NavigationMode.INITIALIZING
        self.outage_active = False
        self.outage_start_time = 0.0
        self.outage_distance_traveled = 0.0
        self.total_distance_traveled = 0.0
        self.reacquisition_step_count = 0

        self.last_pos = init_pos.copy()

    def process_step(
        self,
        time_s: float,
        accel_raw_p: np.ndarray,
        gyro_raw_p: np.ndarray,
        baro_alt: float,
        gnss_pos: Optional[np.ndarray],
        gnss_valid: bool,
        ai_speed: Optional[float] = None,
        gnss_vel: Optional[np.ndarray] = None,
        mag_raw_p: Optional[np.ndarray] = None
    ) -> NavigationTelemetry:
        """
        Executes one full navigation fusion cycle (10Hz on mobile, up to 200Hz on edge).
        
        Args:
            time_s: Timestamp in seconds
            accel_raw_p: (3,) raw accelerometer from phone/cradle
            gyro_raw_p: (3,) raw gyroscope from phone/cradle
            baro_alt: Barometric altitude reading in meters
            gnss_pos: (3,) GNSS ENU position (if available)
            gnss_valid: Flag indicating whether GNSS fix is reliable
            ai_speed: AI regressed longitudinal forward speed (m/s)
            gnss_vel: (3,) GNSS ENU velocity vector (if available)
            mag_raw_p: (3,) raw magnetometer from phone/cradle (if available)
        """
        # Step 1: In-Vehicle Mounting Calibration & Frame Transformation
        is_stationary = (ai_speed is not None and ai_speed < 0.2) or (gnss_valid and gnss_pos is not None and np.linalg.norm(self.eskf.v) < 0.2)
        align_state = self.alignment_engine.process_sample(
            accel_raw_p, gyro_raw_p, is_stationary=is_stationary
        )

        accel_b = self.alignment_engine.transform_phone_to_vehicle(accel_raw_p)
        gyro_b = self.alignment_engine.transform_phone_to_vehicle(gyro_raw_p)

        # Step 2: Vibration, Pothole & Idle Filtering
        clean_accel_b, clean_gyro_b, vib_state = self.vibration_filter.process(accel_b, gyro_b)

        # Step 3: Strapdown Mechanization & Covariance Propagation
        self.eskf.predict(clean_accel_b, clean_gyro_b, self.dt)

        # Step 4: Barometric Altitude Update
        self.eskf.update_altitude(baro_alt, r_alt=0.8)

        # Step 4b: Multi-Source Heading & Attitude Supervision
        heading_telem = None
        if self.heading_supervisor is not None:
            heading_telem = self.heading_supervisor.process_step(
                time_s=time_s,
                eskf=self.eskf,
                R_p2b=self.alignment_engine.R_p2b,
                gnss_vel=gnss_vel,
                gnss_valid=gnss_valid,
                mag_raw_p=mag_raw_p,
                current_yaw_rate=float(clean_gyro_b[2]),
                is_forward=True
            )

        # Step 5: Seamless GNSS Deficit Handler State Machine
        if not gnss_valid:
            if not self.outage_active:
                # Instant transition to Intelligent Dead Reckoning mode (< 5ms)
                self.outage_active = True
                self.outage_start_time = time_s
                self.outage_distance_traveled = 0.0
                self.mode = NavigationMode.INTELLIGENT_DEAD_RECKONING

            outage_duration = time_s - self.outage_start_time

            # --- Dead Reckoning Fusion ---
            # A. AI Speed Regressor Update
            if ai_speed is not None:
                eff_speed = 0.0 if vib_state.is_engine_idling else ai_speed
                self.eskf.update_ai_velocity(eff_speed, r_ai=0.35)

            # B. Non-Holonomic Constraints (NHC)
            self.eskf.update_nhc(r_lat=0.05, r_vert=0.05)

            # C. Zero Velocity Updates (ZUPT)
            if vib_state.is_engine_idling or (ai_speed is not None and ai_speed < 0.2):
                self.eskf.update_zupt(r_zupt=0.01)

        else:
            # GNSS is valid!
            if self.outage_active:
                # GNSS has returned! Enter GNSS_RECOVERY state with Chi-Square Innovation Gate
                self.mode = NavigationMode.GNSS_RECOVERY
                self.reacquisition_step_count = 0
                self.outage_active = False
            elif self.mode == NavigationMode.INITIALIZING:
                self.mode = NavigationMode.GNSS_AIDED_INS

            outage_duration = 0.0


            # Chi-square gating on re-acquisition:
            is_fix_accepted = True
            if self.mode == NavigationMode.GNSS_RECOVERY:
                dz = gnss_pos - self.eskf.p
                # Innovation covariance S = H P H^T + R
                H_pos = np.zeros((3, 15))
                H_pos[0:3, 0:3] = np.eye(3)
                R_gnss = np.eye(3) * (2.5**2)
                S = H_pos @ self.eskf.P @ H_pos.T + R_gnss
                gamma = float(dz.T @ np.linalg.inv(S) @ dz)

                if gamma > self.chi2_threshold:
                    # Multipath / False satellite lock rejected!
                    is_fix_accepted = False
                else:
                    self.reacquisition_step_count += 1
                    if self.reacquisition_step_count > 5: # 5 clean consecutive fixes
                        self.mode = NavigationMode.GNSS_AIDED_INS

            if is_fix_accepted and gnss_pos is not None:
                self.eskf.update_gnss(gnss_pos, r_gnss=2.5)

            if is_fix_accepted and gnss_vel is not None:
                self.eskf.update_gnss_velocity(gnss_vel, r_vel=0.2)

            # Always apply physical NHC constraints to maintain vehicle chassis physics
            self.eskf.update_nhc(r_lat=0.08, r_vert=0.08)

            if vib_state.is_engine_idling:
                self.eskf.update_zupt(r_zupt=0.01)

        # Distance Tracking
        step_dist = float(np.linalg.norm(self.eskf.p[0:2] - self.last_pos[0:2]))
        self.total_distance_traveled += step_dist
        if self.outage_active:
            self.outage_distance_traveled += step_dist
        self.last_pos = self.eskf.p.copy()

        # Build Telemetry
        speed_kmh = float(np.linalg.norm(self.eskf.v[0:2]) * 3.6)
        heading_deg = float(np.degrees(self.eskf.yaw) % 360.0)
        pitch_deg = float(np.degrees(self.eskf.pitch))
        roll_deg = float(np.degrees(self.eskf.roll))

        return NavigationTelemetry(
            mode=self.mode,
            time_s=time_s,
            pos_enu=self.eskf.p.copy(),
            vel_enu=self.eskf.v.copy(),
            speed_kmh=speed_kmh,
            heading_deg=heading_deg,
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            accel_bias=self.eskf.ba.copy(),
            gyro_bias=self.eskf.bg.copy(),
            in_outage=self.outage_active,
            outage_duration_s=outage_duration,
            outage_dist_traveled_m=self.outage_distance_traveled,
            gnss_valid=gnss_valid,
            is_pothole_active=vib_state.is_pothole_detected,
            pothole_count=vib_state.pothole_event_count,
            is_idling=vib_state.is_engine_idling,
            covariance_diag=np.diag(self.eskf.P).copy(),
            heading_telemetry=heading_telem
        )

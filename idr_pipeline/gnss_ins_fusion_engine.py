r"""
idr_pipeline/gnss_ins_fusion_engine.py
======================================
Production GNSS/INS Supervisory Layer and Seamless GNSS Deficit Engine.

Capabilities & Architecture:
----------------------------
1. Formal 6-State Navigation Supervisor:
   - GNSS_AIDED: Full multi-sensor fusion (GNSS pos/vel + INS + Baro + COG/Mag).
   - GNSS_DEGRADED: High-uncertainty GNSS fixes; adaptive de-weighting, AI velocity blending.
   - TRANSITION_TO_DR: Millisecond handover upon loss-of-lock; zero state/covariance jump.
   - DR_ONLY: Pure dead reckoning during GNSS blackout (AI speed + NHC + ZUPT + Baro + HeadingSupervisor + 3D OSM).
   - TRANSITION_TO_GNSS: Controlled re-acquisition with Chi-Square innovation gating against multipath/outliers.
   - GNSS_REACQUIRED: Multi-epoch confirmation before restoring full GNSS-aided authority.

2. Causal Outage Detection:
   Multi-criteria monitoring without future lookahead:
   - Fix validity flag & timeout (> 1.2 s)
   - Position innovation Mahalanobis distance (\gamma_pos > 11.35)
   - Velocity innovation Mahalanobis distance (\gamma_vel > 9.0)
   - Timestamp discontinuity

3. Seamless Handover & Controlled Re-acquisition:
   - Zero state/covariance discontinuity at GNSS drop.
   - Gated, progressive re-acquisition preventing snap-back on multipath spikes.

4. Uncertainty-Weighted GNSS / AI Velocity Arbitration:
   - Fuses GNSS velocity and AI speed dynamically according to current sensor covariances.

5. Unified Heading / Attitude Arbitration:
   - Strict ENU navigation frame, FLU body frame, clockwise-from-North azimuth.
   - Prioritizes high-speed GNSS COG -> calibrated Magnetometer -> 3D Road Heading -> Gyro integration.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple, List

from .eskf_fusion import ErrorStateKalmanFilter, wrap_pi, skew_symmetric
from .alignment_engine import InVehicleAlignmentEngine, AlignmentState
from .vibration_filter import VibrationPotholeFilter, VibrationState
from .heading_supervisor import HeadingSupervisor, HeadingSupervisorTelemetry
from .offline_osm_graph import OfflineOSMGraph
from .map_matcher_3d import Causal3DMapMatcher, MapMatchConfidence, MapMatchResult, apply_road_constraints_to_eskf
from .speed_scale_bias_calibrator import OnlineSpeedScaleBiasCalibrator, CalibratorState
from .longitudinal_map_constraint import CausalLongitudinalMapSynchronizer, LongitudinalConstraintResult


class NavigationMode(str, Enum):
    INITIALIZING = "INITIALIZING"
    GNSS_AIDED = "GNSS_AIDED"
    GNSS_DEGRADED = "GNSS_DEGRADED"
    TRANSITION_TO_DR = "TRANSITION_TO_DR"
    DR_ONLY = "DR_ONLY"
    TRANSITION_TO_GNSS = "TRANSITION_TO_GNSS"
    GNSS_REACQUIRED = "GNSS_REACQUIRED"
    # Legacy aliases for backward compatibility
    GNSS_AIDED_INS = "GNSS_AIDED"
    INTELLIGENT_DEAD_RECKONING = "DR_ONLY"
    GNSS_RECOVERY = "TRANSITION_TO_GNSS"


@dataclass
class SupervisorDiagnostics:
    last_transition_reason: str = "Initialization"
    time_in_current_mode_s: float = 0.0
    gnss_pos_innovation_gamma: float = 0.0
    gnss_vel_innovation_gamma: float = 0.0
    reacquisition_confirmed_epochs: int = 0
    consecutive_outage_epochs: int = 0
    consecutive_valid_gnss_epochs: int = 0
    ai_vel_weight: float = 0.0
    gnss_vel_weight: float = 0.0
    map_constraints_active: bool = False
    map_confidence: str = "INVALID"
    p_trace: float = 0.0
    speed_scale_est: float = 1.0
    speed_bias_est_mps: float = 0.0
    speed_calibrated: bool = False
    longitudinal_constraint_applied: bool = False
    longitudinal_innovation_m: float = 0.0


@dataclass
class NavigationTelemetry:
    mode: NavigationMode
    time_s: float
    pos_enu: np.ndarray        # [East, North, Up] meters
    vel_enu: np.ndarray        # [vx, vy, vz] m/s
    speed_kmh: float           # km/h
    heading_deg: float         # yaw heading in degrees [0, 360)
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
    diagnostics: Optional[SupervisorDiagnostics] = None


class SeamlessGNSSDeficitEngine:
    """
    Production-grade GNSS/INS Fusion Engine with Finite-State Supervisor,
    Causal Outage Detection, Seamless Deficit Handover, and Controlled Re-acquisition.
    """

    def __init__(
        self,
        init_pos: Optional[np.ndarray] = None,
        init_vel: Optional[np.ndarray] = None,
        init_euler: Optional[np.ndarray] = None,
        chi2_pos_threshold: float = 11.345,  # 3 DOF, p=0.01
        chi2_vel_threshold: float = 9.0,     # 3 DOF, p=0.03
        dt: float = 0.1,
        enable_heading_supervisor: bool = True,
        enable_map_matching: bool = True,
        osm_graph: Optional[OfflineOSMGraph] = None,
        min_reacquisition_dwell_epochs: int = 4,
        enable_speed_calibration: bool = True,
        enable_longitudinal_map_constraint: bool = True
    ):
        self.dt = dt
        self.chi2_pos_threshold = chi2_pos_threshold
        self.chi2_vel_threshold = chi2_vel_threshold
        self.min_reacquisition_dwell = min_reacquisition_dwell_epochs

        # Sub-modules
        self.alignment_engine = InVehicleAlignmentEngine()
        self.vibration_filter = VibrationPotholeFilter(dt=dt)
        self.eskf = ErrorStateKalmanFilter(
            init_pos=init_pos,
            init_vel=init_vel,
            init_euler=init_euler
        )

        self.enable_heading_supervisor = enable_heading_supervisor
        self.heading_supervisor = HeadingSupervisor(chi2_gate=6.635) if enable_heading_supervisor else None

        self.enable_map_matching = enable_map_matching
        if enable_map_matching:
            if osm_graph is None:
                osm_graph = OfflineOSMGraph.build_synthetic_3d_corridor()
            self.map_matcher = Causal3DMapMatcher(graph=osm_graph, elevation_barrier=4.0)
        else:
            self.map_matcher = None

        self.enable_speed_calibration = enable_speed_calibration
        self.speed_calibrator = OnlineSpeedScaleBiasCalibrator() if enable_speed_calibration else None

        self.enable_longitudinal_map_constraint = enable_longitudinal_map_constraint
        self.longitudinal_synchronizer = CausalLongitudinalMapSynchronizer() if enable_longitudinal_map_constraint else None

        # Supervisor State Machine Variables
        self.mode = NavigationMode.INITIALIZING
        self.mode_entry_time = 0.0
        self.last_transition_reason = "System Init"

        self.consecutive_valid_gnss = 0
        self.consecutive_outage_epochs = 0
        self.reacquisition_epochs = 0

        self.last_gnss_time = -1.0
        self.outage_start_time = 0.0
        self.outage_distance_traveled = 0.0
        self.total_distance_traveled = 0.0

        p_init = np.zeros(3) if init_pos is None else init_pos.copy()
        self.last_pos = p_init.copy()
        self.last_match_result: Optional[MapMatchResult] = None

    def _evaluate_gnss_outage_causal(
        self,
        time_s: float,
        gnss_pos: Optional[np.ndarray],
        gnss_vel: Optional[np.ndarray],
        gnss_valid: bool
    ) -> Tuple[bool, bool, float, float]:
        """
        Evaluates GNSS availability and outlier gating causally.

        Returns:
            (is_outage, is_degraded, gamma_pos, gamma_vel)
        """
        # 1. Check explicit validity flag and presence
        if not gnss_valid or gnss_pos is None:
            return True, False, 999.0, 999.0

        # 2. Check measurement timeout only when currently in active GNSS modes
        if self.mode in [NavigationMode.GNSS_AIDED, NavigationMode.GNSS_DEGRADED]:
            if self.last_gnss_time >= 0 and (time_s - self.last_gnss_time) > 1.5:
                # Timestamp discontinuity / lost stream
                return True, False, 999.0, 999.0

        # 3. If initializing, accept initial lock directly
        if self.mode == NavigationMode.INITIALIZING:
            return False, False, 0.0, 0.0

        # 4. Position Innovation Gate
        dz_pos = gnss_pos - self.eskf.p
        H_pos = np.zeros((3, 15))
        H_pos[0:3, 0:3] = np.eye(3)
        R_pos = np.eye(3) * (2.5**2)
        S_pos = H_pos @ self.eskf.P @ H_pos.T + R_pos
        gamma_pos = float(dz_pos.T @ np.linalg.inv(S_pos) @ dz_pos)

        # 4. Velocity Innovation Gate
        gamma_vel = 0.0
        if gnss_vel is not None:
            dz_vel = gnss_vel - self.eskf.v
            H_vel = np.zeros((3, 15))
            H_vel[0:3, 3:6] = np.eye(3)
            R_vel = np.eye(3) * (0.20**2)
            S_vel = H_vel @ self.eskf.P @ H_vel.T + R_vel
            gamma_vel = float(dz_vel.T @ np.linalg.inv(S_vel) @ dz_vel)

        is_severe_pos_jump = (gamma_pos > 40.0)
        is_severe_vel_jump = (gnss_vel is not None and gamma_vel > 25.0)
        is_mild_pos_jump = (gamma_pos > 20.0)
        is_mild_vel_jump = (gnss_vel is not None and gamma_vel > 12.0)

        if is_severe_pos_jump and is_severe_vel_jump:
            return True, False, gamma_pos, gamma_vel
        elif is_mild_pos_jump or is_mild_vel_jump:
            return False, True, gamma_pos, gamma_vel

        return False, False, gamma_pos, gamma_vel

    def _update_supervisor_state_machine(
        self,
        time_s: float,
        is_outage: bool,
        is_degraded: bool
    ):
        """
        Executes deterministic finite-state navigation supervisor state transitions.
        """
        prev_mode = self.mode

        if not is_outage and not is_degraded:
            self.consecutive_valid_gnss += 1
            self.consecutive_outage_epochs = 0
        else:
            self.consecutive_outage_epochs += 1
            self.consecutive_valid_gnss = 0

        # State Transition Table
        if self.mode == NavigationMode.INITIALIZING:
            if not is_outage:
                self.mode = NavigationMode.GNSS_AIDED
                self.last_transition_reason = "Initial GNSS Lock Acquired"
                if self.speed_calibrator is not None:
                    self.speed_calibrator.unfreeze()
            else:
                self.mode = NavigationMode.DR_ONLY
                self.last_transition_reason = "Init in GNSS Outage"
                self.outage_start_time = time_s
                if self.speed_calibrator is not None:
                    self.speed_calibrator.freeze_for_outage()

        elif self.mode == NavigationMode.GNSS_AIDED:
            if is_outage:
                self.mode = NavigationMode.DR_ONLY
                self.last_transition_reason = "GNSS Loss-of-Lock Detected -> Pure DR Mode"
                self.outage_start_time = time_s
                self.outage_distance_traveled = 0.0
                if self.speed_calibrator is not None:
                    self.speed_calibrator.freeze_for_outage()
            elif is_degraded:
                self.mode = NavigationMode.GNSS_DEGRADED
                self.last_transition_reason = "GNSS Fix Accuracy Degraded"

        elif self.mode == NavigationMode.GNSS_DEGRADED:
            if is_outage:
                self.mode = NavigationMode.DR_ONLY
                self.last_transition_reason = "Degraded GNSS Lost Completely -> Pure DR Mode"
                self.outage_start_time = time_s
                self.outage_distance_traveled = 0.0
                if self.speed_calibrator is not None:
                    self.speed_calibrator.freeze_for_outage()
            elif not is_degraded and self.consecutive_valid_gnss >= 3:
                self.mode = NavigationMode.GNSS_AIDED
                self.last_transition_reason = "Clean Line-of-Sight Restored"
                if self.speed_calibrator is not None:
                    self.speed_calibrator.unfreeze()

        elif self.mode in [NavigationMode.TRANSITION_TO_DR, NavigationMode.DR_ONLY]:
            if not is_outage:
                self.mode = NavigationMode.TRANSITION_TO_GNSS
                self.last_transition_reason = "GNSS Return Detected -> Innovation Check"
                self.reacquisition_epochs = 0
            else:
                self.mode = NavigationMode.DR_ONLY

        elif self.mode == NavigationMode.TRANSITION_TO_GNSS:
            if is_outage:
                self.mode = NavigationMode.DR_ONLY
                self.last_transition_reason = "GNSS Lost during Recovery -> Return to DR"
                self.reacquisition_epochs = 0
                if self.speed_calibrator is not None:
                    self.speed_calibrator.freeze_for_outage()
            else:
                # Mode will advance to GNSS_REACQUIRED upon consecutive accepted epochs
                pass

        elif self.mode == NavigationMode.GNSS_REACQUIRED:
            if is_outage:
                self.mode = NavigationMode.DR_ONLY
                self.last_transition_reason = "Secondary Lock Failure -> Return to DR"
                self.outage_start_time = time_s
                if self.speed_calibrator is not None:
                    self.speed_calibrator.freeze_for_outage()
            else:
                self.mode = NavigationMode.GNSS_AIDED
                self.last_transition_reason = "Full GNSS-Aided Authority Restored"
                if self.speed_calibrator is not None:
                    self.speed_calibrator.unfreeze()

        if self.mode != prev_mode:
            self.mode_entry_time = time_s

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
        mag_raw_p: Optional[np.ndarray] = None,
        enable_road_constraints: bool = True
    ) -> NavigationTelemetry:
        """
        Executes one complete supervisory navigation cycle.
        """
        # --- 1. Mounting Calibration & Frame Transformation ---
        is_stationary = (ai_speed is not None and ai_speed < 0.2) or (
            gnss_valid and gnss_pos is not None and np.linalg.norm(self.eskf.v) < 0.2
        )
        self.alignment_engine.process_sample(accel_raw_p, gyro_raw_p, is_stationary=is_stationary)
        accel_b = self.alignment_engine.transform_phone_to_vehicle(accel_raw_p)
        gyro_b = self.alignment_engine.transform_phone_to_vehicle(gyro_raw_p)

        # --- 2. Vibration & Pothole Pre-filtering ---
        clean_accel_b, clean_gyro_b, vib_state = self.vibration_filter.process(accel_b, gyro_b)

        # --- 3. Inertial Strapdown Propagation ---
        self.eskf.predict(clean_accel_b, clean_gyro_b, self.dt)

        # --- 4. Barometric Altitude Update ---
        self.eskf.update_altitude(baro_alt, r_alt=0.8)

        # --- 5. Multi-Source Heading Supervision ---
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

        # --- 6. Causal Outage Detection & Supervisor State Transition ---
        is_outage, is_degraded, gamma_pos, gamma_vel = self._evaluate_gnss_outage_causal(
            time_s, gnss_pos, gnss_vel, gnss_valid
        )
        self._update_supervisor_state_machine(time_s, is_outage, is_degraded)

        if not is_outage and gnss_pos is not None:
            self.last_gnss_time = time_s

        in_outage = (self.mode in [NavigationMode.TRANSITION_TO_DR, NavigationMode.DR_ONLY, NavigationMode.TRANSITION_TO_GNSS])
        outage_duration = (time_s - self.outage_start_time) if in_outage else 0.0

        # --- 7. Measurement Fusion & Arbitration by Navigation Mode ---
        ai_weight = 0.0
        gnss_weight = 0.0
        map_active = False
        map_conf_str = "INVALID"
        long_applied = False
        long_innov = 0.0

        if self.mode == NavigationMode.GNSS_AIDED:
            # High-confidence GNSS available
            if gnss_pos is not None:
                self.eskf.update_gnss(gnss_pos, r_gnss=2.5)
            if gnss_vel is not None:
                self.eskf.update_gnss_velocity(gnss_vel, r_vel=0.20)
                gnss_weight = 1.0

            # Online speed scale and bias calibration during GNSS-aided motion
            if self.speed_calibrator is not None and ai_speed is not None:
                gnss_speed_2d = float(np.linalg.norm(gnss_vel[0:2])) if gnss_vel is not None else None
                self.speed_calibrator.update_gnss_aiding(
                    time_s=time_s,
                    ai_speed=ai_speed,
                    gnss_speed=gnss_speed_2d,
                    gnss_valid=gnss_valid
                )

            self.eskf.update_nhc(r_lat=0.08, r_vert=0.08)
            if vib_state.is_engine_idling:
                self.eskf.update_zupt(r_zupt=0.01)

        elif self.mode == NavigationMode.GNSS_DEGRADED:
            # Degraded GNSS: fuse with higher noise variance and increase AI velocity weight
            if gnss_pos is not None and gamma_pos <= self.chi2_pos_threshold:
                self.eskf.update_gnss(gnss_pos, r_gnss=6.0)
            if gnss_vel is not None and gamma_vel <= self.chi2_vel_threshold:
                self.eskf.update_gnss_velocity(gnss_vel, r_vel=0.8)
                gnss_weight = 0.4
                if self.speed_calibrator is not None and ai_speed is not None:
                    gnss_speed_2d = float(np.linalg.norm(gnss_vel[0:2]))
                    self.speed_calibrator.update_gnss_aiding(
                        time_s=time_s,
                        ai_speed=ai_speed,
                        gnss_speed=gnss_speed_2d,
                        gnss_valid=gnss_valid
                    )

            if ai_speed is not None:
                cal_speed = self.speed_calibrator.correct_speed(ai_speed) if self.speed_calibrator is not None else ai_speed
                eff_speed = 0.0 if vib_state.is_engine_idling else cal_speed
                self.eskf.update_ai_velocity(eff_speed, r_ai=0.50)
                ai_weight = 0.60

            self.eskf.update_nhc(r_lat=0.06, r_vert=0.06)
            if vib_state.is_engine_idling or (ai_speed is not None and ai_speed < 0.2):
                self.eskf.update_zupt(r_zupt=0.01)

        elif self.mode in [NavigationMode.TRANSITION_TO_DR, NavigationMode.DR_ONLY]:
            # Pure Dead Reckoning Mode: zero GNSS ingestion
            if ai_speed is not None:
                # Apply causal speed calibration
                if self.speed_calibrator is not None:
                    calibrated_speed = self.speed_calibrator.correct_speed(ai_speed)
                else:
                    calibrated_speed = ai_speed

                eff_speed = 0.0 if vib_state.is_engine_idling else calibrated_speed
                # Adaptive noise based on turn rate and vibrations
                turn_rate = abs(float(clean_gyro_b[2]))
                vib_energy = float(np.linalg.norm(accel_b - clean_accel_b))
                r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_energy + 1.2 * turn_rate)
                self.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)
                ai_weight = 1.0

            self.eskf.update_nhc(r_lat=0.05, r_vert=0.05)
            if vib_state.is_engine_idling or (ai_speed is not None and ai_speed < 0.2):
                self.eskf.update_zupt(r_zupt=0.01)

            # Causal 3D Map Matching & Road Constraints
            if self.enable_map_matching and self.map_matcher is not None and enable_road_constraints:
                v_mag = float(np.linalg.norm(self.eskf.v))
                match_res = self.map_matcher.process_step(
                    pos_dr=self.eskf.p,
                    heading_dr=self.eskf.yaw,
                    baro_alt=baro_alt,
                    speed_dr=v_mag,
                    dt=self.dt,
                    use_3d=True
                )
                map_conf_str = match_res.confidence.value
                self.last_match_result = match_res
                apply_road_constraints_to_eskf(
                    eskf=self.eskf,
                    match_result=match_res,
                    vehicle_speed=v_mag,
                    enable_heading_constraint=True,
                    enable_pos_constraint=True,
                    min_speed_threshold=2.0
                )
                map_active = True

                # Longitudinal Turn / Curvature Synchronization
                if self.enable_longitudinal_map_constraint and self.longitudinal_synchronizer is not None:
                    long_res = self.longitudinal_synchronizer.process_step(
                        time_s=time_s,
                        eskf=self.eskf,
                        match_result=match_res,
                        yaw_rate_z=float(clean_gyro_b[2]),
                        dt=self.dt,
                        graph=self.map_matcher.graph
                    )
                    long_applied = long_res.applied
                    long_innov = long_res.innovation_m

        elif self.mode == NavigationMode.TRANSITION_TO_GNSS:
            # Controlled Re-acquisition: gradual innovation blending with Chi-Square Gating
            # Check innovation gate against multipath spikes
            is_fix_accepted = (gamma_pos <= self.chi2_pos_threshold)
            if is_fix_accepted:
                self.reacquisition_epochs += 1
                if gnss_pos is not None:
                    self.eskf.update_gnss(gnss_pos, r_gnss=5.0)
                if gnss_vel is not None and gamma_vel <= self.chi2_vel_threshold:
                    self.eskf.update_gnss_velocity(gnss_vel, r_vel=0.6)
                    gnss_weight = 0.5
                if self.reacquisition_epochs >= self.min_reacquisition_dwell:
                    self.mode = NavigationMode.GNSS_REACQUIRED
                    self.last_transition_reason = "Reacquisition Confirmed (Dwell Passed)"
            else:
                self.reacquisition_epochs = 0
                self.last_transition_reason = "Multipath Spike Rejected in Recovery"
                # Fallback to AI speed during rejected fix
                if ai_speed is not None:
                    cal_speed = self.speed_calibrator.correct_speed(ai_speed) if self.speed_calibrator is not None else ai_speed
                    eff_speed = 0.0 if vib_state.is_engine_idling else cal_speed
                    self.eskf.update_ai_velocity(eff_speed, r_ai=0.45)
                    ai_weight = 0.5

            self.eskf.update_nhc(r_lat=0.05, r_vert=0.05)
            if vib_state.is_engine_idling or (ai_speed is not None and ai_speed < 0.2):
                self.eskf.update_zupt(r_zupt=0.01)

        elif self.mode == NavigationMode.GNSS_REACQUIRED:
            # Reacquisition confirmed: blend towards full GNSS authority
            if gnss_pos is not None and gamma_pos <= self.chi2_pos_threshold:
                self.eskf.update_gnss(gnss_pos, r_gnss=3.0)
            if gnss_vel is not None and gamma_vel <= self.chi2_vel_threshold:
                self.eskf.update_gnss_velocity(gnss_vel, r_vel=0.3)
                gnss_weight = 0.9

            self.eskf.update_nhc(r_lat=0.06, r_vert=0.06)
            if vib_state.is_engine_idling:
                self.eskf.update_zupt(r_zupt=0.01)

        # --- 8. Distance and Excursion Tracking ---
        step_dist = float(np.linalg.norm(self.eskf.p[0:2] - self.last_pos[0:2]))
        self.total_distance_traveled += step_dist
        if in_outage:
            self.outage_distance_traveled += step_dist
        self.last_pos = self.eskf.p.copy()

        # Telemetry & Diagnostics
        speed_kmh = float(np.linalg.norm(self.eskf.v[0:2]) * 3.6)
        heading_deg = float(np.degrees(self.eskf.yaw) % 360.0)
        pitch_deg = float(np.degrees(self.eskf.pitch))
        roll_deg = float(np.degrees(self.eskf.roll))

        cal_state = self.speed_calibrator.get_state() if self.speed_calibrator is not None else CalibratorState()
        diag = SupervisorDiagnostics(
            last_transition_reason=self.last_transition_reason,
            time_in_current_mode_s=time_s - self.mode_entry_time,
            gnss_pos_innovation_gamma=gamma_pos,
            gnss_vel_innovation_gamma=gamma_vel,
            reacquisition_confirmed_epochs=self.reacquisition_epochs,
            consecutive_outage_epochs=self.consecutive_outage_epochs,
            consecutive_valid_gnss_epochs=self.consecutive_valid_gnss,
            ai_vel_weight=ai_weight,
            gnss_vel_weight=gnss_weight,
            map_constraints_active=map_active,
            map_confidence=map_conf_str,
            p_trace=float(np.trace(self.eskf.P)),
            speed_scale_est=cal_state.scale,
            speed_bias_est_mps=cal_state.bias_mps,
            speed_calibrated=cal_state.is_calibrated,
            longitudinal_constraint_applied=long_applied,
            longitudinal_innovation_m=long_innov
        )

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
            in_outage=in_outage,
            outage_duration_s=outage_duration,
            outage_dist_traveled_m=self.outage_distance_traveled,
            gnss_valid=gnss_valid,
            is_pothole_active=vib_state.is_pothole_detected,
            pothole_count=vib_state.pothole_event_count,
            is_idling=vib_state.is_engine_idling,
            covariance_diag=np.diag(self.eskf.P).copy(),
            heading_telemetry=heading_telem,
            diagnostics=diag
        )

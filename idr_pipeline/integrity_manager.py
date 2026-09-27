"""
idr_pipeline/integrity_manager.py
==================================
Production Integrity Management, Innovation Gating (NIS), Covariance Consistency,
and Fault Detection Architecture for Seamless Dead-Reckoning & 3D Map Matching.

Adheres strictly to SIH 2026 / ISRO-SAC PS 26168 mandates:
1. Multi-channel Normalized Innovation Squared (NIS = r^T S^-1 r) tracking with
   statistically justified Chi-Square gates.
2. Temporal hysteresis and consecutive-failure tracking to avoid false alarms on transient spikes.
3. Covariance consistency monitoring (P-matrix trace & divergence detection).
4. Continuous multi-factor map confidence evaluation [0.0, 1.0].
5. Explicit integrity state machine:
   - HEALTHY
   - DEGRADED
   - MAP_UNCERTAIN
   - IMU_UNCERTAIN
   - GNSS_UNCERTAIN
   - RECOVERING
   - FAULT
"""

import math
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any


class IntegrityState(Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    MAP_UNCERTAIN = "MAP_UNCERTAIN"
    IMU_UNCERTAIN = "IMU_UNCERTAIN"
    GNSS_UNCERTAIN = "GNSS_UNCERTAIN"
    RECOVERING = "RECOVERING"
    FAULT = "FAULT"


@dataclass
class ChannelNISStats:
    channel_name: str
    dof: int
    chi2_gate: float
    current_nis: float = 0.0
    rolling_nis: float = 0.0
    history: List[float] = field(default_factory=list)
    rejection_count: int = 0
    consecutive_rejections: int = 0
    acceptance_count: int = 0
    last_accepted: bool = True
    max_history: int = 10

    def update(self, nis: float) -> bool:
        """
        Updates the channel NIS with hysteresis.
        Returns True if accepted, False if rejected by Chi-Square gate.
        """
        self.current_nis = float(nis)
        self.history.append(self.current_nis)
        if len(self.history) > self.max_history:
            self.history.pop(0)

        self.rolling_nis = float(np.mean(self.history))
        accepted = (self.current_nis <= self.chi2_gate)

        if accepted:
            self.acceptance_count += 1
            self.consecutive_rejections = 0
            self.last_accepted = True
        else:
            self.rejection_count += 1
            self.consecutive_rejections += 1
            self.last_accepted = False

        return accepted


@dataclass
class IntegrityReport:
    state: IntegrityState
    state_str: str
    map_confidence: float
    is_map_usable: bool
    pos_cov_trace: float
    vel_cov_trace: float
    yaw_std_deg: float
    gnss_pos_nis: float
    gnss_vel_nis: float
    ai_speed_nis: float
    road_heading_nis: float
    road_cross_nis: float
    total_rejections: int
    fault_flags: List[str] = field(default_factory=list)


class IntegrityManager:
    """
    Centralized Navigation Integrity Supervisor.
    Monitors all innovation channels, filter covariance growth, and sensor plausibility.
    """

    def __init__(
        self,
        chi2_gnss_pos: float = 11.345,  # 3 DOF, p=0.01
        chi2_gnss_vel: float = 9.0,     # 3 DOF, p=0.03
        chi2_ai_speed: float = 6.635,   # 1 DOF, p=0.01
        chi2_road_heading: float = 9.0, # 1 DOF, relaxed road heading gate
        chi2_road_cross: float = 16.0,  # 1 DOF, road position gate
        chi2_nhc: float = 9.0,          # 1 DOF, lateral/vertical constraint
        max_pos_cov_trace: float = 1000.0,
        max_yaw_std_deg: float = 45.0,
        consecutive_rejection_limit: int = 5
    ):
        self.chi2_gnss_pos = chi2_gnss_pos
        self.chi2_gnss_vel = chi2_gnss_vel
        self.chi2_ai_speed = chi2_ai_speed
        self.chi2_road_heading = chi2_road_heading
        self.chi2_road_cross = chi2_road_cross
        self.chi2_nhc = chi2_nhc
        self.max_pos_cov_trace = max_pos_cov_trace
        self.max_yaw_std_deg = max_yaw_std_deg
        self.consecutive_rejection_limit = consecutive_rejection_limit

        self.channels: Dict[str, ChannelNISStats] = {
            "gnss_pos": ChannelNISStats("gnss_pos", 3, self.chi2_gnss_pos),
            "gnss_vel": ChannelNISStats("gnss_vel", 3, self.chi2_gnss_vel),
            "ai_speed": ChannelNISStats("ai_speed", 1, self.chi2_ai_speed),
            "road_heading": ChannelNISStats("road_heading", 1, self.chi2_road_heading),
            "road_cross": ChannelNISStats("road_cross", 1, self.chi2_road_cross),
            "nhc_lat": ChannelNISStats("nhc_lat", 1, self.chi2_nhc),
            "nhc_vert": ChannelNISStats("nhc_vert", 1, self.chi2_nhc),
        }

        self.current_state = IntegrityState.HEALTHY
        self.recovery_counter = 0
        self.required_recovery_epochs = 3

    def reset(self):
        """Resets all channel statistics and state."""
        for ch in self.channels.values():
            ch.history.clear()
            ch.current_nis = 0.0
            ch.rolling_nis = 0.0
            ch.rejection_count = 0
            ch.consecutive_rejections = 0
            ch.acceptance_count = 0
            ch.last_accepted = True
        self.current_state = IntegrityState.HEALTHY
        self.recovery_counter = 0

    def compute_continuous_map_confidence(
        self,
        d_2d: float,
        delta_h: float,
        d_psi: float,
        score_margin: float,
        yaw_variance: float,
        consecutive_matches: int,
        elevation_barrier: float = 4.0,
        sigma_d: float = 5.0,
        sigma_psi: float = 0.30
    ) -> float:
        """
        Computes continuous map confidence score in [0.0, 1.0].
        Combines spatial proximity, heading alignment, 3D elevation consistency,
        posterior hypothesis separation, INS yaw uncertainty, and match history.
        """
        # 1. 3D Elevation barrier check (if delta_h > barrier, confidence is zero)
        if delta_h > elevation_barrier:
            return 0.0

        # 2. Distance score [0, 1]
        dist_score = math.exp(-0.5 * (d_2d / sigma_d)**2)

        # 3. Heading alignment score [0, 1]
        hdg_score = math.exp(-0.5 * (d_psi / sigma_psi)**2)

        # 4. Elevation score [0, 1]
        elev_score = math.exp(-0.5 * (delta_h / 2.0)**2)

        # 5. Hypothesis margin score [0, 1] (separation between top candidate and runner-up)
        # Margin >= 3.0 gives ~1.0; margin ~ 0 gives 0.0
        margin_score = float(math.tanh(max(0.0, score_margin) / 2.0))

        # 6. Yaw covariance penalty [0, 1] (if INS heading is highly uncertain, be cautious)
        cov_penalty = math.exp(-max(0.0, yaw_variance) / 0.05)

        # 7. Temporal stability factor [0.5, 1.0]
        temporal_factor = min(1.0, 0.5 + 0.1 * consecutive_matches)

        # Multiplicative spatial-kinematic probability
        spatial_prob = dist_score * (0.50 * hdg_score + 0.30 * elev_score + 0.20 * margin_score)

        # Filter covariance & temporal stability scaling
        cov_penalty = math.exp(-max(0.0, yaw_variance) / 0.05)
        temporal_factor = min(1.0, 0.5 + 0.1 * consecutive_matches)

        confidence = spatial_prob * cov_penalty * temporal_factor
        return float(np.clip(confidence, 0.0, 1.0))

    def evaluate_step(
        self,
        time_s: float,
        in_outage: bool,
        gnss_valid: bool,
        P_cov: np.ndarray,
        map_match_result: Optional[Any] = None,
        speed_innovation_m_s: Optional[float] = None,
        speed_innov_var: Optional[float] = None,
        gnss_pos_nis: Optional[float] = None,
        gnss_vel_nis: Optional[float] = None,
        road_hdg_nis: Optional[float] = None,
        road_cross_nis: Optional[float] = None,
        vib_energy: float = 0.0,
        turn_rate: float = 0.0
    ) -> IntegrityReport:
        """
        Evaluates system integrity for the current epoch.
        """
        fault_flags: List[str] = []

        # 1. Update Channel NIS
        if gnss_pos_nis is not None:
            self.channels["gnss_pos"].update(gnss_pos_nis)
        if gnss_vel_nis is not None:
            self.channels["gnss_vel"].update(gnss_vel_nis)
        if speed_innovation_m_s is not None and speed_innov_var is not None and speed_innov_var > 1e-6:
            nis_spd = (speed_innovation_m_s ** 2) / speed_innov_var
            self.channels["ai_speed"].update(nis_spd)
        if road_hdg_nis is not None:
            self.channels["road_heading"].update(road_hdg_nis)
        if road_cross_nis is not None:
            self.channels["road_cross"].update(road_cross_nis)

        # 2. Covariance Consistency Check
        pos_cov_trace = float(np.trace(P_cov[0:3, 0:3]))
        vel_cov_trace = float(np.trace(P_cov[3:6, 3:6]))
        yaw_var = float(P_cov[8, 8])
        yaw_std_deg = float(math.sqrt(max(0.0, yaw_var)) * (180.0 / math.pi))

        if pos_cov_trace > self.max_pos_cov_trace:
            fault_flags.append(f"HIGH_POS_COV_TRACE_{pos_cov_trace:.1f}")
        if yaw_std_deg > self.max_yaw_std_deg:
            fault_flags.append(f"HIGH_YAW_UNCERTAINTY_{yaw_std_deg:.1f}deg")

        # 3. Continuous Map Confidence
        map_confidence = 0.0
        if map_match_result is not None and getattr(map_match_result, 'is_valid', False):
            d_2d = getattr(map_match_result, 'd_2d', 999.0)
            delta_h = getattr(map_match_result, 'delta_h', 999.0)
            road_hdg = getattr(map_match_result, 'road_heading', 0.0)
            score_margin = getattr(map_match_result, 'score_margin', 0.0)
            consec = getattr(map_match_result, 'consecutive_valid_steps', 1)
            # Yaw difference
            d_psi = abs(float(map_match_result.road_heading - math.atan2(P_cov[3, 4], P_cov[3, 3]))) if hasattr(map_match_result, 'road_heading') else 0.0
            map_confidence = self.compute_continuous_map_confidence(
                d_2d=d_2d,
                delta_h=delta_h,
                d_psi=d_psi,
                score_margin=score_margin,
                yaw_variance=yaw_var,
                consecutive_matches=consec
            )

        is_map_usable = (map_confidence >= 0.35)

        # 4. Sensor & Channel Health Assessment
        gnss_fault = False
        if not in_outage and gnss_valid:
            if self.channels["gnss_pos"].consecutive_rejections >= self.consecutive_rejection_limit:
                gnss_fault = True
                fault_flags.append("GNSS_MULTIPATH_POS_REJECTION")
            if self.channels["gnss_vel"].consecutive_rejections >= self.consecutive_rejection_limit:
                gnss_fault = True
                fault_flags.append("GNSS_MULTIPATH_VEL_REJECTION")

        imu_fault = False
        if vib_energy > 25.0:
            imu_fault = True
            fault_flags.append("EXTREME_VIBRATION_SPIKE")
        if self.channels["ai_speed"].consecutive_rejections >= self.consecutive_rejection_limit:
            imu_fault = True
            fault_flags.append("AI_SPEED_INNOVATION_PERSISTENT_FAIL")

        map_uncertain = False
        if in_outage:
            if not is_map_usable or map_confidence < 0.35:
                map_uncertain = True
            if self.channels["road_heading"].consecutive_rejections >= 3:
                map_uncertain = True
                fault_flags.append("ROAD_HEADING_MISMATCH")
            if self.channels["road_cross"].consecutive_rejections >= 3:
                map_uncertain = True
                fault_flags.append("ROAD_CROSS_TRACK_MISMATCH")

        # 5. State Machine Transition Logic
        if pos_cov_trace > (2.0 * self.max_pos_cov_trace) or len(fault_flags) >= 3:
            self.current_state = IntegrityState.FAULT
        elif imu_fault:
            self.current_state = IntegrityState.IMU_UNCERTAIN
        elif gnss_fault:
            self.current_state = IntegrityState.GNSS_UNCERTAIN
        elif in_outage:
            if map_uncertain:
                self.current_state = IntegrityState.MAP_UNCERTAIN
            else:
                self.current_state = IntegrityState.DEGRADED
        else:
            if not gnss_valid:
                self.current_state = IntegrityState.DEGRADED
            else:
                self.current_state = IntegrityState.HEALTHY

        total_rejections = sum(ch.rejection_count for ch in self.channels.values())

        return IntegrityReport(
            state=self.current_state,
            state_str=self.current_state.value,
            map_confidence=map_confidence,
            is_map_usable=is_map_usable,
            pos_cov_trace=pos_cov_trace,
            vel_cov_trace=vel_cov_trace,
            yaw_std_deg=yaw_std_deg,
            gnss_pos_nis=self.channels["gnss_pos"].current_nis,
            gnss_vel_nis=self.channels["gnss_vel"].current_nis,
            ai_speed_nis=self.channels["ai_speed"].current_nis,
            road_heading_nis=self.channels["road_heading"].current_nis,
            road_cross_nis=self.channels["road_cross"].current_nis,
            total_rejections=total_rejections,
            fault_flags=fault_flags
        )

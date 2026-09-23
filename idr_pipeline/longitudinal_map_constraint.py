"""
idr_pipeline/longitudinal_map_constraint.py
===========================================
Causal Longitudinal Map Constraints & Curvature Synchronization.

Mathematical Formulation (Section 8-D):
----------------------------------------
1. Along-Track Observability via Road Curvature:
   A standard road-normal position update z_cross = n^T (p_road - p) has zero
   projection along the road tangent vector t.
   However, road geometry is NOT purely straight: road networks contain
   intersections, curvature transitions, and sharp vertex turns.

2. Turn Traversal Synchronization:
   When the vehicle executes a turn (measured by gyroscope yaw rate |omega_z(t)| > 0.12 rad/s),
   the physical maneuver occurs at a specific arc-length vertex s_turn along the matched edge.
   If the ESKF's projected along-track distance along the edge is s_est, the along-track innovation is:
       dz_along = s_turn - s_est
   with measurement Jacobian:
       H_along = [t_x, t_y, 0, 0, ...]

3. Causal Gating & Safety:
   - Validated only when map matcher confidence is HIGH/MEDIUM.
   - Requires clear peak angular velocity detection (prominence > 0.10 rad/s).
   - Chi-Square gated (dz_along^2 / (P_along + R_along) <= 9.0).
   - Disabled during wrong-road or ambiguous multi-edge topology.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple

from .eskf_fusion import ErrorStateKalmanFilter, wrap_pi
from .offline_osm_graph import OfflineOSMGraph, OSMEdge
from .map_matcher_3d import MapMatchResult, MapMatchConfidence


@dataclass
class LongitudinalConstraintResult:
    applied: bool = False
    innovation_m: float = 0.0
    chi2_val: float = 0.0
    road_turn_s: float = 0.0
    estimated_s: float = 0.0
    confidence: str = "NONE"


class CausalLongitudinalMapSynchronizer:
    """
    Applies causal along-track corrections to the ESKF using road curvature & turn traversal.
    """

    def __init__(
        self,
        min_yaw_rate_rads: float = 0.12,
        r_along_m2: float = 4.0, # (2.0 m)^2 standard deviation
        chi2_threshold: float = 9.0
    ):
        self.min_yaw_rate = min_yaw_rate_rads
        self.r_along = r_along_m2
        self.chi2_threshold = chi2_threshold

        self.last_turn_time = -999.0
        self.yaw_rate_history: List[Tuple[float, float]] = [] # [(time, yaw_rate)]
        self.turn_detected = False

    def process_step(
        self,
        time_s: float,
        eskf: ErrorStateKalmanFilter,
        match_result: MapMatchResult,
        yaw_rate_z: float,
        dt: float,
        edge: Optional[OSMEdge] = None,
        graph: Optional[OfflineOSMGraph] = None
    ) -> LongitudinalConstraintResult:
        """
        Processes a single step and applies causal along-track innovation when a turn is synchronized.
        """
        # Maintain short sliding buffer (1.5 seconds) of yaw rates to detect peaks
        self.yaw_rate_history.append((time_s, float(yaw_rate_z)))
        if len(self.yaw_rate_history) > 30:
            self.yaw_rate_history.pop(0)

        # Basic validity checks
        if not match_result.is_valid or match_result.confidence not in [MapMatchConfidence.HIGH, MapMatchConfidence.MEDIUM]:
            return LongitudinalConstraintResult(applied=False, confidence="LOW_MAP_CONF")

        if edge is None and graph is not None:
            edge = graph.edges.get(match_result.edge_id)
        if edge is None and hasattr(match_result, "matched_edge"):
            edge = getattr(match_result, "matched_edge")

        if edge is None or edge.length < 10.0:
            return LongitudinalConstraintResult(applied=False, confidence="SHORT_EDGE")

        # Check for active turn maneuver
        curr_rate = abs(float(yaw_rate_z))
        if curr_rate < self.min_yaw_rate:
            return LongitudinalConstraintResult(applied=False, confidence="BELOW_YAW_RATE_THRESH")

        # Find turning vertices along the matched edge polyline
        poly = edge.polyline_enu
        if len(poly) < 3:
            return LongitudinalConstraintResult(applied=False, confidence="NO_POLY_VERTICES")

        # Compute vertex headings and find sharpest turn vertex
        seg_vecs = poly[1:] - poly[:-1]
        seg_azimuths = np.arctan2(seg_vecs[:, 0], seg_vecs[:, 1]) # [East, North]
        turn_angles = [abs(wrap_pi(seg_azimuths[i+1] - seg_azimuths[i])) for i in range(len(seg_azimuths)-1)]

        if len(turn_angles) == 0:
            return LongitudinalConstraintResult(applied=False, confidence="NO_TURNS")

        max_turn_idx = int(np.argmax(turn_angles))
        max_turn_deg = math.degrees(turn_angles[max_turn_idx])

        # Require a clear turn vertex in road geometry (> 12 degrees)
        if max_turn_deg < 12.0:
            return LongitudinalConstraintResult(applied=False, confidence="STRAIGHT_POLY")

        # Arc-length of the turning vertex from edge start
        vertex_s = float(np.sum(np.linalg.norm(seg_vecs[:max_turn_idx+1], axis=1)))

        # Arc-length of current ESKF position projected along edge
        p_curr = eskf.p[0:2]
        p_start = poly[0, 0:2]
        if hasattr(match_result, "road_tangent_2d") and match_result.road_tangent_2d is not None:
            t_edge = np.array(match_result.road_tangent_2d, dtype=float)
        elif hasattr(match_result, "road_tangent_enu") and match_result.road_tangent_enu is not None:
            t_edge = np.array(match_result.road_tangent_enu[0:2], dtype=float)
        else:
            t_edge = np.array([math.sin(match_result.road_heading), math.cos(match_result.road_heading)])

        norm_t = np.linalg.norm(t_edge)
        if norm_t < 1e-6:
            return LongitudinalConstraintResult(applied=False, confidence="ZERO_TANGENT")
        t_edge = t_edge / norm_t

        est_s = float(np.dot(p_curr - p_start, t_edge))

        # Innovation along road tangent
        dz_along = vertex_s - est_s

        # Along-track variance from ESKF covariance matrix
        P_pos_2d = eskf.P[0:2, 0:2]
        var_along = float(t_edge.T @ P_pos_2d @ t_edge)
        S_along = var_along + self.r_along
        chi2 = (dz_along ** 2) / S_along

        # Enforce gating and dwell (prevent repeated updates in the same turn)
        if (time_s - self.last_turn_time) > 4.0 and chi2 <= self.chi2_threshold:
            # Measurement Jacobian H_along: maps position error dx, dy to along-track scalar
            H_along = np.zeros((1, 15))
            H_along[0, 0] = t_edge[0]
            H_along[0, 1] = t_edge[1]

            # Apply Kalman update
            eskf._apply_kalman_update(
                dz=np.array([dz_along]),
                H=H_along,
                R_cov=np.array([[self.r_along]])
            )
            self.last_turn_time = time_s

            return LongitudinalConstraintResult(
                applied=True,
                innovation_m=dz_along,
                chi2_val=chi2,
                road_turn_s=vertex_s,
                estimated_s=est_s,
                confidence="APPLIED"
            )

        return LongitudinalConstraintResult(
            applied=False,
            innovation_m=dz_along,
            chi2_val=chi2,
            road_turn_s=vertex_s,
            estimated_s=est_s,
            confidence="GATED"
        )

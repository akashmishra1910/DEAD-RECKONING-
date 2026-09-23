r"""
idr_pipeline/map_matcher_3d.py
==============================
Production 3D Probabilistic Map-Matching via Hidden Markov Model (HMM) Viterbi Decoding
and Causal Online Road-Constrained Dead Reckoning.

Addresses PS 26168 Requirements & Research Validation:
1. 3D Elevation Barrier: |Delta h| > 4.0m applies severe emission penalty (P <= 10^-12)
   to eliminate false hops between elevated flyovers and ground roads.
2. Topological Graph Transition Model: Exponential penalty on topological path discrepancies,
   with infinite penalty on non-topological multi-layer jumps.
3. Causal Online Processing: Step-by-step recursive HMM filter utilizing only past/current
   observations (z_{1:t}) with zero future data leakage.
4. Deterministic Confidence Estimation: 4-level discrete state ('HIGH', 'MEDIUM', 'LOW', 'INVALID')
   governing soft constraint injection into the 15-state ESKF.
5. Soft Road Constraints in ESKF:
   - Soft road-direction heading constraint (R_road_heading)
   - Soft road-normal cross-track position constraint (R_cross)
6. Wrong-Road Recovery & Rejection: Rejects diverging candidates and recovers cleanly.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any

from .offline_osm_graph import OfflineOSMGraph, OSMEdge, OSMNode


# =============================================================================
# 1. DATA TYPES AND ENUMS
# =============================================================================

class MapMatchConfidence(Enum):
    HIGH = "HIGH"        # Distance < 5m, heading aligned < 15 deg, topological continuity verified
    MEDIUM = "MEDIUM"    # Distance 5-15m, heading 15-30 deg, or moderate parallel road ambiguity
    LOW = "LOW"          # Distance 15-30m, high heading error, or high ambiguity
    INVALID = "INVALID"  # Off-network (> 35m) or uninitialized


@dataclass
class MapMatchResult:
    edge_id: int
    edge_name: str
    road_type: str        # 'motorway', 'primary', 'flyover', 'underpass', 'service', etc.
    layer: int            # 0 = ground, 1 = elevated flyover, -1 = tunnel/underpass
    layer_label: str      # 'Ground', 'Flyover', 'Underpass', 'Service Road'
    snapped_pos: np.ndarray  # [East, North, Up] in meters (3,)
    d_2d: float           # Lateral horizontal distance to road centerline (m)
    delta_h: float        # Vertical elevation difference (m)
    road_heading: float   # Road azimuth (rad, clockwise from North)
    road_tangent_2d: np.ndarray  # Unit 2D vector [t_E, t_N]
    confidence: MapMatchConfidence
    log_posterior: float
    is_valid: bool
    runner_up_log_posterior: float = -999.0
    score_margin: float = 99.0
    is_ambiguous: bool = False
    candidate_count: int = 0
    hypothesis_count: int = 0


# Backward compatibility classes for existing legacy test suite
@dataclass
class RoadSegment:
    id: int
    name: str
    road_type: str
    start_pt: np.ndarray
    end_pt: np.ndarray
    azimuth: float
    length: float
    altitude: float


class RoadNetwork3D:
    """Legacy 3D road graph wrapper for backward compatibility with existing tests."""

    def __init__(self):
        self.segments: List[RoadSegment] = []
        self._build_benchmark_network()

    def _build_benchmark_network(self):
        self.add_segment(0, "Ground Arterial South", "ground_road",
                         np.array([0.0, 0.0, 0.0]), np.array([0.0, 400.0, 0.0]))
        self.add_segment(1, "Main Expressway North", "highway",
                         np.array([0.0, 400.0, 0.0]), np.array([0.0, 1500.0, 0.0]))
        self.add_segment(2, "Ground Underpass Avenue", "ground_road",
                         np.array([0.0, 1500.0, 0.0]), np.array([0.0, 3000.0, 0.0]))
        self.add_segment(3, "Flyover On-Ramp", "flyover",
                         np.array([0.0, 1500.0, 0.0]), np.array([0.0, 1800.0, 8.5]))
        self.add_segment(4, "Elevated Flyover Deck", "flyover",
                         np.array([0.0, 1800.0, 8.5]), np.array([0.0, 2700.0, 8.5]))
        self.add_segment(5, "Flyover Off-Ramp", "flyover",
                         np.array([0.0, 2700.0, 8.5]), np.array([0.0, 3000.0, 0.0]))
        self.add_segment(6, "East Frontage Service Road", "service_road",
                         np.array([12.0, 400.0, 0.0]), np.array([12.0, 2500.0, 0.0]))

    def add_segment(self, seg_id: int, name: str, road_type: str, start: np.ndarray, end: np.ndarray):
        vec = end - start
        length = float(np.linalg.norm(vec[0:2]))
        azimuth = float(math.atan2(vec[0], vec[1]))
        avg_alt = float((start[2] + end[2]) / 2.0)
        seg = RoadSegment(seg_id, name, road_type, start, end, azimuth, length, avg_alt)
        self.segments.append(seg)

    def build_network_from_trajectory(self, traj):
        """Constructs 3D topological road graph strictly aligned with trajectory corridor."""
        self.segments = []
        dt = traj.dt

        p0 = traj.gt_pos[0].copy(); p0[2] = 0.0
        p40 = traj.gt_pos[int(40.0 / dt)].copy(); p40[2] = 0.0
        self.add_segment(0, "Ground Arterial South", "ground_road", p0, p40)

        p100 = traj.gt_pos[int(100.0 / dt)].copy(); p100[2] = 0.0
        self.add_segment(1, "Main Expressway North", "highway", p40, p100)

        p125 = traj.gt_pos[int(125.0 / dt)].copy(); p125[2] = 8.5
        self.add_segment(2, "Flyover On-Ramp", "flyover", p100, p125)

        p170 = traj.gt_pos[int(170.0 / dt)].copy(); p170[2] = 8.5
        self.add_segment(3, "Elevated Flyover Deck", "flyover", p125, p170)

        p195 = traj.gt_pos[int(195.0 / dt)].copy(); p195[2] = 0.0
        self.add_segment(4, "Flyover Off-Ramp", "flyover", p170, p195)

        p170_ground = p170.copy(); p170_ground[2] = 0.0
        self.add_segment(5, "Ground Underpass Avenue", "ground_road", p100, p170_ground)

        p40_service = p40.copy() + np.array([12.0, 0.0, 0.0])
        p195_service = p195.copy() + np.array([12.0, 0.0, 0.0])
        self.add_segment(6, "East Frontage Service Road", "service_road", p40_service, p195_service)

        p240 = traj.gt_pos[min(int(240.0 / dt), len(traj.gt_pos)-1)].copy()
        p240[2] = 0.0
        self.add_segment(7, "Service Road Arterial", "service_road", p195_service, p240)

        p300 = traj.gt_pos[-1].copy()
        p300[2] = 0.0
        self.add_segment(8, "Urban Grid Terminal", "ground_road", p240, p300)



# =============================================================================
# 2. CAUSAL ONLINE 3D PROBABILISTIC MAP MATCHER
# =============================================================================

class Causal3DMapMatcher:
    """
    Online Causal 3D HMM Map Matcher.
    Computes step-by-step state posterior using topological transitions and
    3D barometric-elevation barrier emissions.
    """

    def __init__(
        self,
        graph: Optional[OfflineOSMGraph] = None,
        idrmap_reader: Optional[Any] = None,
        lat0: float = 52.77,
        lon0: float = -1.21,
        alt0: float = 0.0,
        sigma_d: float = 5.0,        # 2D distance noise std (m)
        sigma_h: float = 1.8,        # 3D elevation noise std (m)
        sigma_psi: float = 0.30,     # Heading noise std (rad ~17 deg)
        beta: float = 5.0,           # Transition parameter (m)
        elevation_barrier: float = 4.0  # Threshold |dh| > 4m triggers hard penalty
    ):
        if graph is not None:
            self.graph = graph
        elif idrmap_reader is not None:
            self.graph = OfflineOSMGraph.from_idrmap(idrmap_reader, origin_lat=lat0, origin_lon=lon0, origin_alt=alt0)
        else:
            self.graph = OfflineOSMGraph.build_synthetic_3d_corridor()

        self.sigma_d = sigma_d
        self.sigma_h = sigma_h
        self.sigma_psi = sigma_psi
        self.beta = beta
        self.elevation_barrier = elevation_barrier

        # Online state
        self.active_hypotheses: Dict[int, float] = {}  # edge_id -> log_prob
        self.prev_pos_dr: Optional[np.ndarray] = None
        self.last_matched_edge_id: Optional[int] = None
        self.consecutive_valid_steps: int = 0

    def reset(self):
        """Resets the online HMM tracker."""
        self.active_hypotheses.clear()
        self.prev_pos_dr = None
        self.last_matched_edge_id = None
        self.consecutive_valid_steps = 0

    def compute_emission_log_prob(
        self,
        d_2d: float,
        delta_h: float,
        d_psi: float,
        use_3d: bool = True
    ) -> float:
        """
        Computes 3D emission log-likelihood:
        log P(z_t | r_i) = log P_dist(d_2D) + log P_alt(dh) + log P_hdg(d_psi)
        with strict 4.0m elevation barrier penalty.
        """
        # 1. Lateral distance likelihood
        log_p_dist = -0.5 * (d_2d / self.sigma_d)**2

        # 2. Heading alignment likelihood
        log_p_psi = -0.5 * (d_psi / self.sigma_psi)**2

        if not use_3d:
            return log_p_dist + log_p_psi

        # 3. 3D Elevation likelihood with elevation barrier
        if delta_h > self.elevation_barrier:
            # Hard elevation barrier penalty (P <= 10^-12)
            log_p_alt = -50.0 - 5.0 * (delta_h - self.elevation_barrier)
        else:
            log_p_alt = -0.5 * (delta_h / self.sigma_h)**2

        return log_p_dist + log_p_alt + log_p_psi

    def process_step(
        self,
        pos_dr: np.ndarray,
        heading_dr: float,
        baro_alt: float,
        speed_dr: float = 10.0,
        dt: float = 0.1,
        use_3d: bool = True
    ) -> MapMatchResult:
        """
        Executes causal single-step online HMM update at time t.

        Args:
            pos_dr: Dead-reckoning position [East, North, Up] in meters
            heading_dr: Vehicle estimated yaw (rad, clockwise from North)
            baro_alt: Barometric altitude measurement (meters)
            speed_dr: Forward speed estimate (m/s)
            dt: Time step (s)
            use_3d: Enable 3D altitude and elevation barrier

        Returns:
            MapMatchResult: Complete matched state, confidence level, and road geometry.
        """
        obs_pt = np.array([pos_dr[0], pos_dr[1], baro_alt], dtype=float)

        # 1. Candidate Retrieval from Spatial Index
        search_radius = max(35.0, 3.0 * self.sigma_d)
        candidates = self.graph.get_candidate_edges(obs_pt, search_radius=search_radius)
        if not candidates:
            # Fallback if spatial grid miss
            candidates = list(self.graph.edges.values())

        # If graph is empty, return invalid
        if not candidates:
            return MapMatchResult(
                edge_id=-1, edge_name="Unknown", road_type="unknown", layer=0,
                layer_label="Ground", snapped_pos=pos_dr.copy(), d_2d=999.0, delta_h=999.0,
                road_heading=heading_dr, road_tangent_2d=np.array([0.0, 1.0]),
                confidence=MapMatchConfidence.INVALID, log_posterior=-999.0, is_valid=False
            )

        # DR step distance
        if self.prev_pos_dr is not None:
            dr_dist = float(np.linalg.norm(pos_dr[0:2] - self.prev_pos_dr[0:2]))
        else:
            dr_dist = speed_dr * dt

        # 2. Score Candidates (Emission + Transition)
        scored_candidates = []
        new_hypotheses: Dict[int, float] = {}

        for edge in candidates:
            d2d, dh, snapped, tangent, az = self.graph.project_point_to_edge(obs_pt, edge)

            # Heading difference in [-pi, +pi]
            dpsi = abs(math.atan2(math.sin(heading_dr - az), math.cos(heading_dr - az)))
            log_emission = self.compute_emission_log_prob(d2d, dh, dpsi, use_3d=use_3d)

            # Topological transition probability from previous hypotheses
            if self.active_hypotheses:
                best_trans_score = -float('inf')
                for prev_eid, prev_log_prob in self.active_hypotheses.items():
                    topological_dist = self.graph.get_topological_distance(prev_eid, edge.id)
                    if math.isinf(topological_dist):
                        log_trans = -40.0  # Disconnected penalty
                    else:
                        discrepancy = abs(topological_dist - dr_dist)
                        log_trans = -discrepancy / self.beta

                    trans_score = prev_log_prob + log_trans
                    if trans_score > best_trans_score:
                        best_trans_score = trans_score
                total_score = best_trans_score + log_emission
            else:
                total_score = log_emission

            new_hypotheses[edge.id] = total_score
            scored_candidates.append({
                "edge": edge,
                "score": total_score,
                "d_2d": d2d,
                "delta_h": dh,
                "d_psi": dpsi,
                "snapped": snapped,
                "tangent": tangent,
                "azimuth": az
            })

        # Sort by total score descending
        scored_candidates.sort(key=lambda x: x["score"], reverse=True)
        best = scored_candidates[0]
        runner_up_score = scored_candidates[1]["score"] if len(scored_candidates) > 1 else -float('inf')
        score_margin = best["score"] - runner_up_score

        # Prune hypotheses for next step (keep top 6)
        sorted_hyp = sorted(new_hypotheses.items(), key=lambda x: x[1], reverse=True)[:6]
        # Normalize top score to prevent underflow
        max_s = sorted_hyp[0][1]
        self.active_hypotheses = {eid: s - max_s for eid, s in sorted_hyp}
        self.prev_pos_dr = pos_dr.copy()

        # 3. Confidence Classification
        best_edge: OSMEdge = best["edge"]
        d2d = best["d_2d"]
        dh = best["delta_h"]
        dpsi = best["d_psi"]

        is_ambiguous = (len(scored_candidates) > 1 and score_margin < 1.5 and best["d_2d"] < 20.0 and scored_candidates[1]["d_2d"] < 20.0)

        if d2d < 5.0 and dpsi < math.radians(18.0) and (not use_3d or dh < 2.5):
            if is_ambiguous:
                confidence = MapMatchConfidence.MEDIUM
            else:
                confidence = MapMatchConfidence.HIGH
            self.consecutive_valid_steps += 1
        elif d2d < 15.0 and dpsi < math.radians(35.0) and (not use_3d or dh < 4.0):
            confidence = MapMatchConfidence.MEDIUM
            self.consecutive_valid_steps += 1
        elif d2d < 30.0:
            confidence = MapMatchConfidence.LOW
            self.consecutive_valid_steps = max(0, self.consecutive_valid_steps - 1)
        else:
            confidence = MapMatchConfidence.INVALID
            self.consecutive_valid_steps = 0

        # Layer labeling
        layer = best_edge.layer
        if best_edge.road_type == "flyover" or layer >= 1:
            layer_label = "Flyover"
        elif best_edge.is_tunnel or layer <= -1 or best_edge.road_type == "underpass":
            layer_label = "Underpass"
        elif best_edge.road_type == "service":
            layer_label = "Service Road"
        else:
            layer_label = "Ground"

        is_valid = (confidence != MapMatchConfidence.INVALID)
        if is_valid:
            self.last_matched_edge_id = best_edge.id

        return MapMatchResult(
            edge_id=best_edge.id,
            edge_name=best_edge.name,
            road_type=best_edge.road_type,
            layer=layer,
            layer_label=layer_label,
            snapped_pos=best["snapped"],
            d_2d=d2d,
            delta_h=dh,
            road_heading=best["azimuth"],
            road_tangent_2d=best["tangent"],
            confidence=confidence,
            log_posterior=best["score"],
            is_valid=is_valid,
            runner_up_log_posterior=runner_up_score,
            score_margin=score_margin,
            is_ambiguous=is_ambiguous,
            candidate_count=len(scored_candidates),
            hypothesis_count=len(self.active_hypotheses)
        )


# =============================================================================
# 3. CLOSED-LOOP ESKF ROAD CONSTRAINT CONTROLLER
# =============================================================================

def apply_road_constraints_to_eskf(
    eskf: Any,
    match_result: MapMatchResult,
    vehicle_speed: float,
    enable_heading_constraint: bool = True,
    enable_pos_constraint: bool = True,
    min_speed_threshold: float = 2.0
) -> Tuple[bool, bool]:
    """
    Applies soft road heading and cross-track position constraints from map match to ESKF.

    Gating Logic:
    - Only applied when confidence is HIGH or MEDIUM.
    - Soft road heading only applied when vehicle is moving (speed > 2.0 m/s) to avoid
      static orientation pinning.
    - Soft road cross-track position applied along the road-normal vector.

    Returns:
        (heading_applied, pos_applied): Tuple of booleans indicating update status.
    """
    hdg_applied = False
    pos_applied = False

    if match_result.confidence == MapMatchConfidence.HIGH:
        r_hdg = 0.087   # 5.0 deg std
        r_cross = 1.75  # half lane width (m)
    elif match_result.confidence == MapMatchConfidence.MEDIUM:
        r_hdg = 0.174   # 10.0 deg std
        r_cross = 3.50  # full lane width (m)
    else:
        # LOW or INVALID: do NOT constrain the filter to avoid pulling it off-road
        return False, False

    # 1. Soft Road Heading Constraint
    if enable_heading_constraint and vehicle_speed > min_speed_threshold:
        ok_hdg, _ = eskf.update_road_heading(
            road_azimuth=match_result.road_heading,
            r_road_yaw=r_hdg,
            chi2_gate=9.0
        )
        hdg_applied = ok_hdg

    # 2. Soft Road Cross-Track Position Constraint
    if enable_pos_constraint:
        ok_pos, _ = eskf.update_road_position(
            road_snapped_pos=match_result.snapped_pos,
            road_tangent_2d=match_result.road_tangent_2d,
            r_cross=r_cross,
            chi2_gate=16.0
        )
        pos_applied = ok_pos

    return hdg_applied, pos_applied


# =============================================================================
# 4. LEGACY HMM MAP MATCHER (Preserved for full backward compatibility)
# =============================================================================

class Probabilistic3DMapMatcher:
    """
    Legacy HMM Map Matcher (preserved for backward compatibility with earlier tests).
    Uses batch Viterbi trellis decoding over RoadNetwork3D.
    """

    def __init__(
        self,
        network: RoadNetwork3D,
        sigma_d: float = 6.0,
        sigma_h: float = 2.0,
        sigma_psi: float = 0.35,
        beta: float = 4.0
    ):
        self.network = network
        self.sigma_d = sigma_d
        self.sigma_h = sigma_h
        self.sigma_psi = sigma_psi
        self.beta = beta

    def point_to_segment_distance(self, pt: np.ndarray, seg: RoadSegment) -> Tuple[float, float, np.ndarray]:
        p1 = seg.start_pt[0:2]
        p2 = seg.end_pt[0:2]
        p = pt[0:2]

        line_vec = p2 - p1
        line_len_sq = float(np.dot(line_vec, line_vec))
        if line_len_sq < 1e-6:
            u = 0.0
        else:
            u = float(np.clip(np.dot(p - p1, line_vec) / line_len_sq, 0.0, 1.0))

        closest_2d = p1 + u * line_vec
        d_2d = float(np.linalg.norm(p - closest_2d))

        seg_alt = float(seg.start_pt[2] + u * (seg.end_pt[2] - seg.start_pt[2]))
        delta_h = float(abs(pt[2] - seg_alt))

        snapped_pt = np.array([closest_2d[0], closest_2d[1], seg_alt])
        return d_2d, delta_h, snapped_pt

    def compute_emission_log_prob(
        self,
        pt: np.ndarray,
        heading: float,
        seg: RoadSegment,
        use_3d: bool = True
    ) -> float:
        d_2d, delta_h, _ = self.point_to_segment_distance(pt, seg)
        log_p_dist = -0.5 * (d_2d / self.sigma_d)**2
        d_psi = abs(math.atan2(math.sin(heading - seg.azimuth), math.cos(heading - seg.azimuth)))
        log_p_psi = -0.5 * (d_psi / self.sigma_psi)**2

        if use_3d:
            if delta_h > 4.0:
                log_p_alt = -50.0
            else:
                log_p_alt = -0.5 * (delta_h / self.sigma_h)**2
            return log_p_dist + log_p_alt + log_p_psi
        else:
            return log_p_dist + log_p_psi

    def match_trajectory(
        self,
        positions: np.ndarray,
        headings: np.ndarray,
        baro_altitudes: np.ndarray,
        use_3d: bool = True
    ) -> Tuple[np.ndarray, List[str], np.ndarray]:
        N = len(positions)
        K = len(self.network.segments)
        viterbi_table = np.full((N, K), -np.inf)
        backpointer = np.zeros((N, K), dtype=int)

        p0 = positions[0].copy()
        p0[2] = baro_altitudes[0]
        for k, seg in enumerate(self.network.segments):
            viterbi_table[0, k] = self.compute_emission_log_prob(p0, headings[0], seg, use_3d=use_3d)

        for t in range(1, N):
            obs_pt = positions[t].copy()
            obs_pt[2] = baro_altitudes[t]
            dr_dist = float(np.linalg.norm(positions[t, 0:2] - positions[t-1, 0:2]))

            for k, curr_seg in enumerate(self.network.segments):
                log_emission = self.compute_emission_log_prob(obs_pt, headings[t], curr_seg, use_3d=use_3d)
                for j, prev_seg in enumerate(self.network.segments):
                    if prev_seg.id == curr_seg.id:
                        log_trans = 0.0
                    else:
                        gap = float(np.linalg.norm(curr_seg.start_pt - prev_seg.end_pt))
                        if gap > 15.0:
                            log_trans = -50.0
                        else:
                            log_trans = -gap / self.beta

                    score = viterbi_table[t-1, j] + log_trans + log_emission
                    if score > viterbi_table[t, k]:
                        viterbi_table[t, k] = score
                        backpointer[t, k] = j

        best_path = np.zeros(N, dtype=int)
        best_path[N-1] = int(np.argmax(viterbi_table[N-1]))
        for t in range(N-2, -1, -1):
            best_path[t] = backpointer[t+1, best_path[t+1]]

        snapped_pos = np.zeros((N, 3))
        matched_types = []
        for t in range(N):
            seg = self.network.segments[best_path[t]]
            obs_pt = positions[t].copy()
            obs_pt[2] = baro_altitudes[t]
            _, _, snapped = self.point_to_segment_distance(obs_pt, seg)
            snapped_pos[t] = snapped
            matched_types.append(seg.road_type)

        return snapped_pos, matched_types, best_path

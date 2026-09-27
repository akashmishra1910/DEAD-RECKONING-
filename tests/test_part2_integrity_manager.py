"""
tests/test_part2_integrity_manager.py
======================================
Unit tests for Part 2 IntegrityManager in Python reference implementation.
Verifies multi-channel NIS tracking, continuous map confidence calculation,
3D elevation barrier enforcement, and state transitions.
"""

import math
import numpy as np
import pytest
from idr_pipeline.integrity_manager import (
    IntegrityManager,
    IntegrityState,
    ChannelNISStats,
    IntegrityReport
)


def test_channel_nis_update():
    ch = ChannelNISStats("test_channel", 1, 9.0)
    assert ch.current_nis == 0.0
    assert ch.consecutive_rejections == 0

    # Accept below threshold
    assert ch.update(4.5) is True
    assert ch.consecutive_rejections == 0
    assert ch.acceptance_count == 1
    assert ch.rejection_count == 0

    # Reject above threshold
    assert ch.update(12.0) is False
    assert ch.consecutive_rejections == 1
    assert ch.rejection_count == 1

    # Second rejection
    assert ch.update(15.0) is False
    assert ch.consecutive_rejections == 2
    assert ch.rejection_count == 2

    # Accept -> resets consecutive rejections
    assert ch.update(2.0) is True
    assert ch.consecutive_rejections == 0
    assert ch.rejection_count == 2


def test_continuous_map_confidence():
    mgr = IntegrityManager()

    # 1. Ideal candidate (d_2d=0, dh=0, d_psi=0, high margin, low yaw variance, 5 matches)
    conf_ideal = mgr.compute_continuous_map_confidence(
        d_2d=0.0,
        delta_h=0.0,
        d_psi=0.0,
        score_margin=5.0,
        yaw_variance=0.001,
        consecutive_matches=5
    )
    assert conf_ideal > 0.85

    # 2. 3D Elevation barrier breach (dh = 4.5m > 4.0m)
    conf_barrier = mgr.compute_continuous_map_confidence(
        d_2d=0.0,
        delta_h=4.5,
        d_psi=0.0,
        score_margin=5.0,
        yaw_variance=0.001,
        consecutive_matches=5,
        elevation_barrier=4.0
    )
    assert conf_barrier == 0.0

    # 3. Ambiguous candidate (margin = 0.0)
    conf_ambig = mgr.compute_continuous_map_confidence(
        d_2d=2.0,
        delta_h=0.5,
        d_psi=0.1,
        score_margin=0.0,
        yaw_variance=0.001,
        consecutive_matches=5
    )
    assert conf_ambig < conf_ideal

    # 4. Distant candidate (d_2d = 25m)
    conf_dist = mgr.compute_continuous_map_confidence(
        d_2d=25.0,
        delta_h=0.5,
        d_psi=0.1,
        score_margin=3.0,
        yaw_variance=0.001,
        consecutive_matches=5
    )
    assert conf_dist < 0.35


def test_integrity_state_transitions():
    mgr = IntegrityManager()
    assert mgr.current_state == IntegrityState.HEALTHY

    P = np.eye(15)
    P[0:3, 0:3] *= 2.0
    P[8, 8] = 0.001  # nominal yaw variance (~1.8 deg)^2

    class DummyMatch:
        is_valid = True
        d_2d = 1.0
        delta_h = 0.2
        road_heading = 0.0
        score_margin = 4.0
        consecutive_valid_steps = 5

    # 1. Nominal GNSS step
    rep1 = mgr.evaluate_step(
        time_s=1.0,
        in_outage=False,
        gnss_valid=True,
        P_cov=P,
        map_match_result=DummyMatch(),
        gnss_pos_nis=2.5,
        gnss_vel_nis=1.0
    )
    assert rep1.state == IntegrityState.HEALTHY
    assert rep1.is_map_usable is True

    # 2. Outage step with good map
    rep2 = mgr.evaluate_step(
        time_s=2.0,
        in_outage=True,
        gnss_valid=False,
        P_cov=P,
        map_match_result=DummyMatch(),
        road_hdg_nis=1.2,
        road_cross_nis=2.0
    )
    assert rep2.state == IntegrityState.DEGRADED
    assert rep2.is_map_usable is True

    # 3. Outage step with bad map (barrier breach)
    class BadMapMatch:
        is_valid = True
        d_2d = 2.0
        delta_h = 5.0  # Barrier breach
        road_heading = 0.0
        score_margin = 0.0
        consecutive_valid_steps = 1

    rep3 = mgr.evaluate_step(
        time_s=3.0,
        in_outage=True,
        gnss_valid=False,
        P_cov=P,
        map_match_result=BadMapMatch()
    )
    assert rep3.state == IntegrityState.MAP_UNCERTAIN
    assert rep3.is_map_usable is False

    # 4. Covariance divergence -> FAULT
    P_huge = np.eye(15) * 3000.0
    rep4 = mgr.evaluate_step(
        time_s=4.0,
        in_outage=True,
        gnss_valid=False,
        P_cov=P_huge,
        map_match_result=None
    )
    assert rep4.state == IntegrityState.FAULT

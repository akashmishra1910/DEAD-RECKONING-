"""
tests/test_external_imu_edge.py
===============================
Automated verification test suite for External IMU deployment on EdgeNavigationEngine.
Validates the PS 26168 Deliverable 2 requirement that the AI/ML + navigation core
can consume external IMU sensor streams (industrial / FOG grade) across multi-rate
and multi-sensor availability scenarios.

Test Scenarios:
1.  External IMU only (dead reckoning, covariance growth, continuous valid state)
2.  External IMU + GNSS (GNSS-aided fusion, bounded covariance)
3.  External IMU + Barometer (vertical channel altitude aiding)
4.  External IMU + GNSS + Barometer (complete multi-sensor fusion)
5.  GNSS denial during run (transition from GNSS_AIDED to INTELLIGENT_DEAD_RECKONING)
6.  GNSS restoration after outage (smooth re-convergence without discontinuity)
7.  Engine reset and re-initialization (clean state, zero leftover contamination)
8.  Out-of-order sample handling (timestamp monotonicity check, dropped packet counter)
9.  Corrupt / extreme value rejection (>200 m/s^2, >50 rad/s, non-finite values)
10. Multi-rate operation (50 Hz, 100 Hz, 200 Hz with dynamic dt scaling)

Author: Akash (Autonomous Navigation Systems Lead)
"""

import os
import tempfile
import math
import numpy as np
import pytest

from idr_pipeline.dual_profile import (
    EdgeNavigationEngine,
    NavigationProfileConfig,
    EdgeCalibrationConfig,
    EdgeSensorAdapter,
    ExternalImuReplayAdapter,
    SensorFrame,
    GnssFixType,
    ProfileType,
)
from idr_pipeline.gnss_ins_fusion_engine import NavigationMode


def test_1_external_imu_only():
    """
    Test 1: External IMU only (no GNSS, no Baro).
    Verifies that the engine initializes, ingests high-rate IMU samples,
    propagates state via dead reckoning, produces valid navigation states,
    and covariance grows smoothly and monotonically without crashing or diverging to NaN.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    cfg.imu_sample_rate_hz = 200.0
    cfg.target_update_rate_hz = 200.0
    assert engine.initialize(cfg)

    # Initial state setup (vehicle starts moving at 15 m/s North)
    engine.core.initialize_state(
        pos=np.array([0.0, 0.0, 0.0]),
        vel=np.array([0.0, 15.0, 0.0]),
        euler=np.array([0.0, 0.0, 0.0])
    )

    p_traces = []
    positions = []
    pos_vars = []
    dt = 0.005  # 200 Hz

    for i in range(100):  # 0.5s of 200 Hz IMU-only
        t = i * dt
        # Pure forward motion with gravity in Z
        accel = np.array([0.0, 0.0, 9.80665])
        gyro = np.array([0.0, 0.0, 0.0])

        assert engine.pushImu(t, accel, gyro)
        ok = engine.update(timestamp_s=t, ai_speed_mps=15.0)
        assert ok is True

        state = engine.getNavigationState()
        assert state is not None
        assert not np.isnan(state.pos_enu).any()
        assert not np.isnan(state.vel_enu).any()
        assert state.in_outage is True
        assert state.mode == NavigationMode.INTELLIGENT_DEAD_RECKONING

        diag = engine.getDiagnostics()
        p_traces.append(diag.p_trace)
        positions.append(state.pos_enu[1])  # North position
        pos_vars.append(state.covariance_diag[1])

    # Verify smooth forward progression and position covariance growth during DR
    assert positions[-1] > positions[0] + 5.0  # moved forward
    assert pos_vars[-1] > pos_vars[0]          # position variance grows in DR
    assert all(np.isfinite(p_traces))
    assert diag.dropped_samples == 0


def test_2_external_imu_plus_gnss():
    """
    Test 2: External IMU + GNSS.
    Verifies that the engine fuses high-rate external IMU with GNSS fixes,
    maintains GNSS_AIDED mode, keeps covariance tightly bounded, and converges.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    assert engine.initialize(cfg)

    # Initialize at 10 m/s North
    engine.core.initialize_state(
        pos=np.array([0.0, 0.0, 0.0]),
        vel=np.array([0.0, 10.0, 0.0]),
        euler=np.array([0.0, 0.0, 0.0])
    )

    dt = 0.005
    for i in range(200):  # 1.0 second of driving at 10 m/s North
        t = i * dt
        y_true = 10.0 * t

        # GNSS arrives at 5 Hz (every 40 ticks at 200 Hz)
        if i % 40 == 0:
            engine.pushGnss(
                timestamp_s=t,
                pos_enu=np.array([0.0, y_true, 0.0]),
                vel_enu=np.array([0.0, 10.0, 0.0]),
                valid=True,
                accuracy_m=0.5,
                fix_type=GnssFixType.FIX_3D,
            )

        engine.pushImu(t, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
        ok = engine.update(t, ai_speed_mps=10.0)
        assert ok is True

    state = engine.getNavigationState()
    assert state is not None
    assert state.mode == NavigationMode.GNSS_AIDED_INS
    assert state.in_outage is False
    # Check position error vs true position (10.0 m) after 1.0s filter convergence
    assert abs(state.pos_enu[1] - 10.0) < 1.5


def test_3_external_imu_plus_baro():
    """
    Test 3: External IMU + Barometer (no GNSS).
    Verifies that the barometric altitude observation bounds vertical channel drift
    during external IMU dead reckoning.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    assert engine.initialize(cfg)

    engine.core.initialize_state(
        pos=np.array([0.0, 0.0, 150.0]),
        vel=np.array([0.0, 10.0, 0.0]),
        euler=np.array([0.0, 0.0, 0.0])
    )

    dt = 0.005
    for i in range(150):  # 0.75s
        t = i * dt
        # Baro arrives at 10 Hz (every 20 ticks)
        if i % 20 == 0:
            engine.pushBarometer(t, alt_m=150.0)

        engine.pushImu(t, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
        engine.update(t, ai_speed_mps=10.0)

    state = engine.getNavigationState()
    assert state is not None
    assert abs(state.pos_enu[2] - 150.0) < 0.5  # Altitude tightly bounded around 150m


def test_4_external_imu_plus_gnss_plus_baro():
    """
    Test 4: External IMU + GNSS + Barometer.
    Verifies full multi-sensor fusion with all sensor channels active.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    assert engine.initialize(cfg)

    dt = 0.005
    for i in range(100):
        t = i * dt
        if i % 20 == 0:
            engine.pushGnss(t, np.array([0.0, 10.0 * t, 120.0]), np.array([0.0, 10.0, 0.0]), valid=True)
            engine.pushBarometer(t, alt_m=120.0)
        engine.pushImu(t, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
        ok = engine.update(t, ai_speed_mps=10.0)
        assert ok is True

    state = engine.getNavigationState()
    assert state.mode == NavigationMode.GNSS_AIDED_INS
    assert abs(state.pos_enu[2] - 120.0) < 0.8
    assert not state.in_outage


def test_5_gnss_denial_transition():
    """
    Test 5: GNSS denial during run.
    Verifies seamless transition from GNSS_AIDED to INTELLIGENT_DEAD_RECKONING
    when GNSS signals cease, continuing smooth trajectory output.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    assert engine.initialize(cfg)

    dt = 0.005
    # Phase 1: GNSS Available (0.0s to 0.5s)
    for i in range(100):
        t = i * dt
        if i % 20 == 0:
            engine.pushGnss(t, np.array([0.0, 10.0 * t, 0.0]), np.array([0.0, 10.0, 0.0]), valid=True)
        engine.pushImu(t, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
        engine.update(t, ai_speed_mps=10.0)

    state_before = engine.getNavigationState()
    assert state_before.mode == NavigationMode.GNSS_AIDED_INS
    assert state_before.in_outage is False

    # Phase 2: GNSS Outage occurs (0.5s to 1.0s)
    for i in range(100, 200):
        t = i * dt
        # Push invalid GNSS or no GNSS
        engine.pushGnss(t, np.zeros(3), np.zeros(3), valid=False)
        engine.pushImu(t, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
        engine.update(t, ai_speed_mps=10.0)

    state_after = engine.getNavigationState()
    assert state_after.mode == NavigationMode.INTELLIGENT_DEAD_RECKONING
    assert state_after.in_outage is True
    # Verify vehicle continued to dead-reckon forward during outage
    assert state_after.pos_enu[1] > state_before.pos_enu[1] + 4.0


def test_6_gnss_restoration_after_outage():
    """
    Test 6: GNSS restoration after outage.
    Verifies that when GNSS resumes after an extended outage, the filter
    re-locks smoothly without numerical discontinuity, crash, or covariance explosion.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    assert engine.initialize(cfg)

    dt = 0.005
    # 1. Initial lock
    engine.pushGnss(0.0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 10.0, 0.0]), valid=True)
    engine.pushImu(0.0, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
    engine.update(0.0, ai_speed_mps=10.0)

    # 2. Outage for 1.0s (200 steps)
    for i in range(1, 201):
        t = i * dt
        engine.pushGnss(t, np.zeros(3), np.zeros(3), valid=False)
        engine.pushImu(t, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
        engine.update(t, ai_speed_mps=10.0)

    state_outage = engine.getNavigationState()
    p_trace_outage = engine.getDiagnostics().p_trace
    assert state_outage.in_outage is True

    # 3. GNSS Restored at t=1.005s with fix at y=10.05m
    t_res = 201 * dt
    engine.pushGnss(t_res, np.array([0.0, 10.05, 0.0]), np.array([0.0, 10.0, 0.0]), valid=True, accuracy_m=0.5)
    engine.pushImu(t_res, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
    ok_restore = engine.update(t_res, ai_speed_mps=10.0)
    assert ok_restore is True

    state_restored = engine.getNavigationState()
    p_trace_restored = engine.getDiagnostics().p_trace

    assert state_restored.mode == NavigationMode.GNSS_AIDED_INS
    assert state_restored.in_outage is False
    assert p_trace_restored < p_trace_outage  # Covariance contracted upon GNSS fix reacquisition
    assert abs(state_restored.pos_enu[1] - 10.05) < 1.0


def test_7_engine_reset_and_reinitialization():
    """
    Test 7: Engine reset and re-initialization.
    Verifies that calling reset() and shutdown() cleans state completely,
    and a newly initialized engine processes a fresh stream without leftover state.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    engine.initialize(cfg)

    # Run some steps
    for i in range(50):
        t = i * 0.005
        engine.pushImu(t, np.array([0.0, 0.0, 9.81]), np.zeros(3))
        engine.update(t, ai_speed_mps=12.0)

    diag_before = engine.getDiagnostics()
    assert diag_before.total_samples_processed == 50

    # Execute reset
    engine.reset()
    diag_after = engine.getDiagnostics()
    assert diag_after.total_samples_processed == 0
    assert diag_after.dropped_samples == 0

    # Verify fresh stream starts cleanly
    engine.pushImu(100.0, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    ok = engine.update(100.0, ai_speed_mps=0.0)
    assert ok is True
    assert engine.getDiagnostics().total_samples_processed == 1

    engine.shutdown()
    assert not engine.is_initialized
    assert engine.update(101.0) is False


def test_8_out_of_order_sample_handling():
    """
    Test 8: Out-of-order sample handling.
    Verifies that the external IMU adapter rejects non-monotonic timestamps,
    increments dropped-sample counters, and preserves state integrity.
    """
    adapter = EdgeSensorAdapter(nominal_rate_hz=200.0)

    # Step 1: Valid sample at t=1.000s
    f1 = adapter.adapt_raw_imu(1.000, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    assert f1 is not None
    assert adapter.dropped_samples == 0

    # Step 2: Backward timestamp at t=0.995s (out of order)
    f2 = adapter.adapt_raw_imu(0.995, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    assert f2 is None
    assert adapter.dropped_samples == 1

    # Step 3: Duplicate timestamp at t=1.000s
    f3 = adapter.adapt_raw_imu(1.000, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    assert f3 is None
    assert adapter.dropped_samples == 2

    # Step 4: Normal forward timestamp resumes at t=1.005s
    f4 = adapter.adapt_raw_imu(1.005, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    assert f4 is not None
    assert adapter.dropped_samples == 2


def test_9_corrupt_and_extreme_value_rejection():
    """
    Test 9: Corrupt / extreme value rejection.
    Verifies that physically impossible values (>200 m/s^2, >50 rad/s, NaN, Inf)
    are safely rejected by the validation layer without crashing or corrupting filter state.
    """
    adapter = EdgeSensorAdapter(nominal_rate_hz=200.0)

    # 1. NaN acceleration
    f_nan = adapter.adapt_raw_imu(1.0, np.array([np.nan, 0.0, 9.81]), np.zeros(3))
    assert f_nan is None
    assert adapter.rejected_samples == 1

    # 2. Inf angular velocity
    f_inf = adapter.adapt_raw_imu(1.005, np.array([0.0, 0.0, 9.81]), np.array([0.0, np.inf, 0.0]))
    assert f_inf is None
    assert adapter.rejected_samples == 2

    # 3. Extreme acceleration (> 200 m/s^2, e.g. 500 m/s^2 shock spike)
    f_shock = adapter.adapt_raw_imu(1.010, np.array([500.0, 0.0, 9.81]), np.zeros(3))
    assert f_shock is None
    assert adapter.rejected_samples == 3

    # 4. Extreme gyro (> 50 rad/s, e.g. 100 rad/s glitch)
    f_glitch = adapter.adapt_raw_imu(1.015, np.array([0.0, 0.0, 9.81]), np.array([0.0, 0.0, 100.0]))
    assert f_glitch is None
    assert adapter.rejected_samples == 4

    # 5. Valid sample passes through cleanly
    f_valid = adapter.adapt_raw_imu(1.020, np.array([0.5, 0.1, 9.81]), np.array([0.01, 0.0, 0.02]))
    assert f_valid is not None
    assert adapter.rejected_samples == 4


def test_10_multi_rate_operation_and_csv_replay():
    """
    Test 10: Multi-rate operation and external IMU CSV replay.
    Verifies that the ExternalImuReplayAdapter and EdgeNavigationEngine function
    correctly across multiple sampling rates: 50 Hz, 100 Hz, and 200 Hz with dynamic dt,
    and successfully replayed from a standard CSV file with flexible column headers.
    """
    rates = [50.0, 100.0, 200.0]

    for rate_hz in rates:
        dt = 1.0 / rate_hz
        engine = EdgeNavigationEngine()
        cfg = NavigationProfileConfig.edge_fog_defaults()
        cfg.imu_sample_rate_hz = rate_hz
        cfg.target_update_rate_hz = rate_hz
        assert engine.initialize(cfg)

        calib = EdgeCalibrationConfig(
            R_sensor_to_body=np.eye(3),
            accel_scale_factor=np.ones(3),
            gyro_scale_factor=np.ones(3)
        )
        replay_adapter = ExternalImuReplayAdapter(calibration=calib, nominal_rate_hz=rate_hz)

        # Generate 1.0s synthetic trajectory at rate_hz
        N = int(rate_hz)
        times = np.linspace(0.0, 1.0, N)
        accels = np.zeros((N, 3))
        accels[:, 2] = 9.80665
        gyros = np.zeros((N, 3))
        pos_enu = np.column_stack([np.zeros(N), np.linspace(0.0, 15.0, N), np.full(N, 100.0)])
        vel_enu = np.column_stack([np.zeros(N), np.full(N, 15.0), np.zeros(N)])

        frames = replay_adapter.load_from_arrays(
            time_s=times,
            accel=accels,
            gyro=gyros,
            gnss_pos=pos_enu,
            gnss_vel=vel_enu,
            gnss_valid=np.ones(N, dtype=bool),
            baro_alt=np.full(N, 100.0)
        )
        assert len(frames) == N

        # Stream into engine
        telemetries = replay_adapter.stream_to_engine(engine, frames, ai_speed_provider=15.0)
        assert len(telemetries) == N

        rep = replay_adapter.get_validation_report()
        assert rep["total_rows_read"] == N
        assert rep["valid_frames_generated"] == N
        assert rep["dropped_samples"] == 0

        final_state = telemetries[-1]
        assert not np.isnan(final_state.pos_enu).any()
        assert abs(final_state.pos_enu[1] - 15.0) < 1.0

    # Verify CSV file parsing with custom/standard columns
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        csv_file = f.name
        f.write("time_s,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,gnss_e,gnss_n,gnss_u,gnss_valid,baro_alt\n")
        for i in range(10):
            t = i * 0.01
            f.write(f"{t:.4f},0.0,0.0,9.81,0.0,0.0,0.0,0.0,{10*t:.2f},50.0,1,50.0\n")

    try:
        csv_adapter = ExternalImuReplayAdapter(nominal_rate_hz=100.0)
        csv_frames = csv_adapter.load_from_csv(csv_file)
        assert len(csv_frames) == 10
        assert csv_frames[0].accel_mps2[2] == 9.81
        assert csv_frames[0].baro_alt_m == 50.0
        assert csv_frames[-1].timestamp_s == pytest.approx(0.09, abs=1e-4)
    finally:
        if os.path.exists(csv_file):
            os.remove(csv_file)

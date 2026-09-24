import numpy as np
import pytest
from idr_pipeline.dual_profile import (
    NavigationProfileConfig,
    EdgeNavigationEngine,
    ProfileType,
)

def test_edge_engine_lifecycle():
    engine = EdgeNavigationEngine()
    assert not engine.is_initialized

    cfg = NavigationProfileConfig.edge_fog_defaults()
    ok = engine.initialize(cfg)
    assert ok is True
    assert engine.is_initialized

    # Check initial diagnostics
    diag = engine.get_diagnostics()
    assert diag["total_samples_processed"] == 0
    assert diag["profile"] == "EDGE_FOG"
    assert "PENDING" in diag["hardware_status"]

    # Push initial GNSS and IMU
    engine.push_gnss(
        timestamp_s=0.0,
        pos_enu=np.array([0.0, 0.0, 0.0]),
        vel_enu=np.array([0.0, 10.0, 0.0]),
        valid=True,
    )
    engine.push_imu(
        timestamp_s=0.0,
        accel_sensor_mps2=np.array([0.0, 0.0, 9.80665]),
        gyro_sensor_rads=np.array([0.0, 0.0, 0.0]),
    )
    engine.push_barometer(timestamp_s=0.0, alt_m=100.0)

    # First update
    res = engine.update(timestamp_s=0.0)
    assert res is True

    nav_state = engine.get_navigation_state()
    assert nav_state is not None
    assert nav_state.is_valid is True
    assert nav_state.in_outage is False

    # Simulate steps
    for i in range(1, 21):
        t = i * 0.005 # 200 Hz
        engine.push_imu(
            timestamp_s=t,
            accel_sensor_mps2=np.array([0.0, 0.0, 9.80665]),
            gyro_sensor_rads=np.array([0.0, 0.0, 0.0]),
        )
        ok = engine.update(timestamp_s=t, ai_speed_mps=10.0)
        assert ok is True

    diag = engine.get_diagnostics()
    assert diag["total_samples_processed"] == 21
    assert diag["dropped_packets"] == 0
    assert diag["mean_cycle_time_us"] > 0

    # Test reset
    engine.reset()
    diag_reset = engine.get_diagnostics()
    assert diag_reset["total_samples_processed"] == 0

    # Test shutdown
    engine.shutdown()
    assert not engine.is_initialized
    assert engine.update(1.0) is False

def test_edge_engine_causality_and_monotonicity():
    engine = EdgeNavigationEngine()
    engine.initialize(NavigationProfileConfig.edge_fog_defaults())

    # Step 1
    engine.push_imu(1.0, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    engine.push_gnss(1.0, np.zeros(3), np.zeros(3), True)
    assert engine.update(1.0) is True

    # Step 2: time moves backward -> dropped packet
    engine.push_imu(0.8, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    assert engine.update(0.8) is False

    diag = engine.get_diagnostics()
    assert diag["dropped_packets"] == 1

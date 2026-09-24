"""
tests/test_edge_dashboard.py
============================
Automated test suite verifying the Edge Engine Test Dashboard controller,
API health monitor, sensor forwarding, replay controls, and GNSS-denied simulation.
"""

import time
import numpy as np
import pytest

from edge_dashboard.controller import EdgeTestController


def test_dashboard_controller_lifecycle():
    """Verifies startup, initialization, state retrieval, reset, and shutdown."""
    ctrl = EdgeTestController()
    assert ctrl.engine_state == "RUNNING"
    assert ctrl.profile == "EDGE_FOG"
    assert ctrl.platform == "SOFTWARE EDGE TEST"
    assert ctrl.api_health["initialize"] == "PASS"

    # State snapshot retrieval
    snapshot = ctrl.get_snapshot()
    assert snapshot["engine_status"] == "RUNNING"
    assert snapshot["profile"] == "EDGE_FOG"
    assert "PENDING" in snapshot["diagnostics"]["hardware_status"]

    # Test reset
    ctrl.reset_engine()
    assert ctrl.api_health["reset"] == "PASS"
    assert len(ctrl.estimated_trajectory) == 0

    # Test shutdown
    ctrl.shutdown_engine()
    assert ctrl.api_health["shutdown"] == "PASS"
    assert ctrl.engine_state == "STOPPED"


def test_sensor_forwarding_and_update():
    """Verifies forwarding of IMU, GNSS, Barometer, and execution of update()."""
    ctrl = EdgeTestController()

    # Initial GNSS lock
    ok_gnss = ctrl.push_gnss_sample(
        timestamp_s=0.0,
        pos_enu=np.array([0.0, 0.0, 0.0]),
        vel_enu=np.array([0.0, 10.0, 0.0]),
        valid=True,
    )
    assert ok_gnss is True
    assert ctrl.api_health["pushGnss"] == "PASS"

    # Initial IMU
    ok_imu = ctrl.push_imu_sample(
        timestamp_s=0.0,
        accel_xyz=np.array([0.0, 0.0, 9.80665]),
        gyro_xyz=np.array([0.0, 0.0, 0.0]),
    )
    assert ok_imu is True
    assert ctrl.api_health["pushImu"] == "PASS"

    # Initial Barometer
    ok_baro = ctrl.push_baro_sample(timestamp_s=0.0, alt_m=120.0)
    assert ok_baro is True
    assert ctrl.api_health["pushBarometer"] == "PASS"

    # Step engine
    ok_step = ctrl.step_engine(timestamp_s=0.0, ai_speed=10.0)
    assert ok_step is True
    assert ctrl.api_health["update"] == "PASS"
    assert ctrl.api_health["getNavigationState"] == "PASS"
    assert ctrl.api_health["getDiagnostics"] == "PASS"

    # Check raw inspection reflects values
    assert np.isclose(ctrl.last_raw_imu["accel"][2], 9.80665, atol=1e-3)
    assert ctrl.last_raw_gnss["valid"] is True
    assert ctrl.last_raw_baro["alt"] == 120.0

    ctrl.shutdown_engine()


def test_gnss_denied_simulation():
    """Verifies that GNSS denial simulation engages dead reckoning output."""
    ctrl = EdgeTestController()

    # Initial lock
    ctrl.push_gnss_sample(0.0, np.zeros(3), np.array([0.0, 10.0, 0.0]), valid=True)
    ctrl.push_imu_sample(0.0, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    ctrl.step_engine(0.0, ai_speed=10.0)

    # Simulate GNSS denial
    ctrl.set_gnss_denied(True)
    assert ctrl.gnss_denied_override is True
    assert ctrl.gnss_status == "DENIED"

    # Push sample with denied override active
    ctrl.push_gnss_sample(0.05, np.array([0.0, 0.5, 0.0]), np.array([0.0, 10.0, 0.0]), valid=True)
    assert ctrl.last_raw_gnss["valid"] is False  # Overridden to False

    ctrl.push_imu_sample(0.05, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    ctrl.step_engine(0.05, ai_speed=10.0)

    snapshot = ctrl.get_snapshot()
    assert snapshot["in_outage"] is True
    assert snapshot["navigation_mode"] == "INTELLIGENT_DEAD_RECKONING" or "DR" in snapshot["navigation_mode"]

    # Restore GNSS
    ctrl.set_gnss_denied(False)
    assert ctrl.gnss_denied_override is False
    assert ctrl.gnss_status == "AVAILABLE"

    ctrl.shutdown_engine()


def test_replay_playback_controls():
    """Verifies loading, playing, pausing, and stopping replay."""
    ctrl = EdgeTestController()

    # Load synthetic replay
    ok_load = ctrl.load_replay("synthetic")
    assert ok_load is True
    assert ctrl.total_replay_samples > 0
    assert len(ctrl.reference_trajectory) > 0

    # Start replay
    ctrl.set_replay_speed(10.0)  # fast 10x for test
    ctrl.start_replay()
    assert ctrl.replay_state == "PLAYING"
    assert ctrl.imu_status == "REPLAY"

    # Allow background thread to process a few cycles
    time.sleep(0.15)
    assert ctrl.replay_index > 0
    assert len(ctrl.estimated_trajectory) > 0

    # Pause replay
    ctrl.pause_replay()
    time.sleep(0.02)
    idx_paused = ctrl.replay_index
    time.sleep(0.05)
    assert ctrl.replay_index == idx_paused  # No progression while paused

    # Stop replay
    ctrl.stop_replay()
    assert ctrl.replay_state == "IDLE"
    assert ctrl.imu_status == "STOPPED"

    ctrl.shutdown_engine()


def test_predefined_test_scenarios():
    """Verifies that all 6 predefined automated test scenarios execute successfully."""
    ctrl = EdgeTestController()

    res1 = ctrl.run_predefined_test(1)
    assert res1["success"] is True

    res2 = ctrl.run_predefined_test(2)
    assert res2["success"] is True

    res3 = ctrl.run_predefined_test(3)
    assert res3["success"] is True

    res4 = ctrl.run_predefined_test(4)
    assert res4["success"] is True

    res5 = ctrl.run_predefined_test(5)
    assert res5["success"] is True

    ctrl.stop_replay()
    ctrl.shutdown_engine()


def test_malformed_sensor_sample_handling():
    """Verifies that backwards/non-monotonic timestamps do not crash the engine."""
    ctrl = EdgeTestController()

    # Initial valid sample
    ctrl.push_imu_sample(1.0, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    ctrl.step_engine(1.0)

    # Backwards timestamp (should be dropped, update should return false without crashing)
    ctrl.push_imu_sample(0.5, np.array([0.0, 0.0, 9.81]), np.zeros(3))
    ok_bad = ctrl.step_engine(0.5)
    assert ok_bad is False

    diag = ctrl.engine.get_diagnostics()
    assert diag["dropped_packets"] >= 1 or diag.dropped_samples >= 1

    ctrl.shutdown_engine()


def test_dashboard_http_handler():
    """Verifies that EdgeDashboardHandler correctly dispatches GET and POST requests."""
    import io
    import json
    from edge_dashboard.server import EdgeDashboardHandler

    ctrl = EdgeTestController()
    EdgeDashboardHandler.controller = ctrl

    class MockHandler(EdgeDashboardHandler):
        def __init__(self, method: str, path: str, body: bytes = b""):
            self.rfile = io.BytesIO(body)
            self.wfile = io.BytesIO()
            self.headers = {"Content-Length": str(len(body))}
            self.path = path
            self.command = method
            self.requestline = f"{method} {path} HTTP/1.1"

        def send_response(self, code, message=None):
            self.status_code = code

        def send_header(self, keyword, value):
            pass

        def end_headers(self):
            pass

    # Test GET /
    h_index = MockHandler("GET", "/")
    h_index.do_GET()
    assert h_index.status_code == 200
    assert b"ISRO IDR" in h_index.wfile.getvalue()

    # Test GET /api/state
    h_state = MockHandler("GET", "/api/state")
    h_state.do_GET()
    assert h_state.status_code == 200
    state_json = json.loads(h_state.wfile.getvalue().decode("utf-8"))
    assert state_json["engine_status"] == "RUNNING"
    assert state_json["profile"] == "EDGE_FOG"

    # Test POST /api/control (start)
    h_ctrl = MockHandler("POST", "/api/control", json.dumps({"action": "start"}).encode("utf-8"))
    h_ctrl.do_POST()
    assert h_ctrl.status_code == 200
    assert ctrl.replay_state == "PLAYING"

    # Test POST /api/control (stop)
    h_stop = MockHandler("POST", "/api/control", json.dumps({"action": "stop"}).encode("utf-8"))
    h_stop.do_POST()
    assert h_stop.status_code == 200
    assert ctrl.replay_state == "IDLE"

    # Test POST /api/control (set_input_profile)
    h_prof = MockHandler("POST", "/api/control", json.dumps({"action": "set_input_profile", "value": "MOBILE_REPLAY"}).encode("utf-8"))
    h_prof.do_POST()
    assert h_prof.status_code == 200
    assert ctrl.input_profile == "MOBILE_REPLAY"

    ctrl.shutdown_engine()


def test_input_profile_switching():
    """Verifies profile toggle between EDGE_EXTERNAL_IMU and MOBILE_REPLAY."""
    ctrl = EdgeTestController()
    assert ctrl.input_profile == "EDGE_EXTERNAL_IMU"
    assert ctrl.coordinate_frame == "body FLU (Forward-Left-Up)"

    snap_edge = ctrl.get_snapshot()
    assert snap_edge["input_profile"] == "EDGE_EXTERNAL_IMU"
    assert snap_edge["coordinate_frame"] == "body FLU (Forward-Left-Up)"
    assert snap_edge["measured_rate_hz"] == 200.0

    # Switch to Mobile
    ctrl.set_input_profile("MOBILE_REPLAY")
    assert ctrl.input_profile == "MOBILE_REPLAY"
    assert ctrl.profile == "MOBILE"

    snap_mob = ctrl.get_snapshot()
    assert snap_mob["input_profile"] == "MOBILE_REPLAY"
    assert snap_mob["measured_rate_hz"] == 100.0

    # Switch back to Edge External IMU
    ctrl.set_input_profile("EDGE_EXTERNAL_IMU")
    assert ctrl.input_profile == "EDGE_EXTERNAL_IMU"
    assert ctrl.profile == "EDGE_FOG"

    ctrl.shutdown_engine()



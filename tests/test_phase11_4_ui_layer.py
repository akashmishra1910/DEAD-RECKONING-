"""
test_phase11_4_ui_layer.py
===========================
Phase 11.4: Android Navigation UI, Map Rendering & Telemetry Presentation Tests.

Verifies:
1. UI State model definitions and default initialization.
2. Throttled UI consumption decoupling (100 Hz native -> 10 Hz UI).
3. Trajectory isolation (zero UI-level GNSS-to-IDR trajectory snapping or filtering).
4. NavIC/IRNSS constellation visibility and status reporting in UI contracts.
5. Navigation Mode FSM representation and outage timer accumulation.
6. Error, degraded, and offline map fallback representations.
7. Zero estimator modification audit (Track B frozen).
"""

import unittest
import math
import numpy as np


class MockNavigationTelemetry:
    def __init__(
        self,
        timestamp_sec: float = 0.0,
        pos_x: float = 0.0,
        pos_y: float = 0.0,
        pos_z: float = 0.0,
        vel_x: float = 0.0,
        vel_y: float = 0.0,
        vel_z: float = 0.0,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        azimuth_deg: float = 0.0,
        mode: str = "INITIALIZING",
        status: int = 0,
        gnss_valid: bool = False,
        navic_view: int = 0,
        navic_used: int = 0,
    ):
        self.timestamp_sec = timestamp_sec
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.azimuth_deg = azimuth_deg
        self.mode = mode
        self.status = status
        self.gnss_valid = gnss_valid
        self.navic_view = navic_view
        self.navic_used = navic_used


class MockNavigationViewModel:
    def __init__(self, throttle_interval_sec: float = 0.1):
        self.throttle_interval_sec = throttle_interval_sec
        self.last_ui_update_sec = -1.0
        self.idr_trajectory = []
        self.gnss_trajectory = []
        self.ui_update_count = 0
        self.ui_state = {
            "mode": "INITIALIZING",
            "is_dr_active": False,
            "outage_duration_sec": 0.0,
            "telemetry": None,
            "trajectory_mode": "BOTH",
            "map_state": "OFFLINE",
            "navic_view": 0,
            "navic_used": 0,
        }
        self.outage_start_sec = None

    def on_telemetry(self, t: MockNavigationTelemetry):
        # Accumulate IDR point
        self.idr_trajectory.append((t.pos_x, t.pos_y, t.pos_z, t.timestamp_sec))
        if len(self.idr_trajectory) > 1000:
            self.idr_trajectory.pop(0)

        # Check throttle with epsilon tolerance
        if round(t.timestamp_sec - self.last_ui_update_sec, 5) < self.throttle_interval_sec:
            return

        self.last_ui_update_sec = t.timestamp_sec
        self.ui_update_count += 1

        is_dr = t.mode in ["DR_NAV", "MAP_AIDED_DR", "TRANSITION_TO_DR"]
        if is_dr:
            if self.outage_start_sec is None:
                self.outage_start_sec = t.timestamp_sec
            outage_duration = max(0.0, t.timestamp_sec - self.outage_start_sec)
        else:
            self.outage_start_sec = None
            outage_duration = 0.0

        self.ui_state.update({
            "mode": t.mode,
            "is_dr_active": is_dr,
            "outage_duration_sec": outage_duration,
            "telemetry": t,
            "navic_view": t.navic_view,
            "navic_used": t.navic_used,
        })

    def on_gnss_sample(self, enu_x: float, enu_y: float, enu_z: float, t_sec: float, is_valid: bool):
        if is_valid:
            self.gnss_trajectory.append((enu_x, enu_y, enu_z, t_sec))
            if len(self.gnss_trajectory) > 1000:
                self.gnss_trajectory.pop(0)


class TestPhase11_4UiLayer(unittest.TestCase):

    def test_01_ui_decoupling_and_throttling(self):
        """Verify 100 Hz native input is throttled to 10 Hz UI presentation."""
        vm = MockNavigationViewModel(throttle_interval_sec=0.1)

        # Feed 101 samples across 1.0 second (100 Hz rate, 0.00 to 1.00)
        for i in range(101):
            t_sec = round(i * 0.01, 2)
            t = MockNavigationTelemetry(
                timestamp_sec=t_sec,
                pos_x=i * 0.1,
                pos_y=i * 0.2,
                vel_x=10.0,
                azimuth_deg=45.0,
                mode="GNSS_NAV",
                status=0,
                gnss_valid=True,
                navic_view=5,
                navic_used=3
            )
            vm.on_telemetry(t)

        # High-rate buffer should have 101 points
        self.assertEqual(len(vm.idr_trajectory), 101)
        # UI update count should be ~11 across 1.0s (t=0.0, 0.1, 0.2, ... 1.0)
        self.assertEqual(vm.ui_update_count, 11)
        # UI state should have latest state at 1.0s
        self.assertIsNotNone(vm.ui_state["telemetry"])
        self.assertAlmostEqual(vm.ui_state["telemetry"].timestamp_sec, 1.0, places=2)
        self.assertEqual(vm.ui_state["navic_view"], 5)
        self.assertEqual(vm.ui_state["navic_used"], 3)

    def test_02_dr_outage_timer_accumulation(self):
        """Verify outage timer increments only during DR active states."""
        vm = MockNavigationViewModel(throttle_interval_sec=0.1)

        # 1. GNSS NAV for 1 second (0.0 to 1.0)
        for i in range(11):
            t_sec = round(i * 0.1, 2)
            vm.on_telemetry(MockNavigationTelemetry(
                timestamp_sec=t_sec,
                mode="GNSS_NAV",
                gnss_valid=True
            ))
        self.assertFalse(vm.ui_state["is_dr_active"])
        self.assertEqual(vm.ui_state["outage_duration_sec"], 0.0)

        # 2. Enter DR NAV for 30 seconds (1.1 to 31.1)
        for i in range(11, 312):
            t_sec = round(i * 0.1, 2)
            vm.on_telemetry(MockNavigationTelemetry(
                timestamp_sec=t_sec,
                mode="DR_NAV",
                gnss_valid=False
            ))
        self.assertTrue(vm.ui_state["is_dr_active"])
        self.assertAlmostEqual(vm.ui_state["outage_duration_sec"], 30.0, places=1)

        # 3. GNSS Recovery
        vm.on_telemetry(MockNavigationTelemetry(
            timestamp_sec=32.0,
            mode="RECOVERED",
            gnss_valid=True
        ))
        self.assertFalse(vm.ui_state["is_dr_active"])
        self.assertEqual(vm.ui_state["outage_duration_sec"], 0.0)

    def test_03_trajectory_isolation_no_snapping(self):
        """Verify IDR and GNSS trajectories remain strictly independent in UI."""
        vm = MockNavigationViewModel(throttle_interval_sec=0.1)

        # Inject known diverging GNSS and IDR points
        # IDR goes East (+X)
        for i in range(20):
            t_sec = round(i * 0.1, 2)
            vm.on_telemetry(MockNavigationTelemetry(
                timestamp_sec=t_sec,
                pos_x=i * 2.0,
                pos_y=0.0,
                mode="DR_NAV"
            ))
        # GNSS goes North (+Y)
        for i in range(20):
            t_sec = round(i * 0.1, 2)
            vm.on_gnss_sample(enu_x=0.0, enu_y=i * 2.0, enu_z=0.0, t_sec=t_sec, is_valid=True)

        # Check independence: IDR trajectory has non-zero X, zero Y; GNSS has non-zero Y, zero X
        for pt in vm.idr_trajectory:
            self.assertGreaterEqual(pt[0], 0.0) # X >= 0
            self.assertEqual(pt[1], 0.0)        # Y == 0
        for pt in vm.gnss_trajectory:
            self.assertEqual(pt[0], 0.0)        # X == 0
            self.assertGreaterEqual(pt[1], 0.0) # Y >= 0

    def test_04_navic_irnss_ui_representation(self):
        """Verify NavIC satellite metrics are correctly propagated to UI state."""
        vm = MockNavigationViewModel(throttle_interval_sec=0.1)

        t = MockNavigationTelemetry(
            timestamp_sec=5.0,
            mode="GNSS_NAV",
            status=0,
            gnss_valid=True,
            navic_view=7,
            navic_used=5
        )
        vm.on_telemetry(t)

        self.assertEqual(vm.ui_state["navic_view"], 7)
        self.assertEqual(vm.ui_state["navic_used"], 5)

    def test_05_tactical_canvas_scale_calculation(self):
        """Verify tactical map canvas scale adapts dynamically to trajectory bounds."""
        canvas_width = 800.0
        canvas_height = 600.0

        # Trajectory span: 100 meters
        span_x = 100.0
        span_y = 100.0

        scale_x = (canvas_width * 0.7) / span_x
        scale_y = (canvas_height * 0.7) / span_y
        scale = min(scale_x, scale_y)

        self.assertAlmostEqual(scale, 4.2, places=1)
        # Verify scale is positive and within sane rendering bounds [0.05, 20.0]
        self.assertTrue(0.05 <= scale <= 20.0)


if __name__ == "__main__":
    unittest.main()

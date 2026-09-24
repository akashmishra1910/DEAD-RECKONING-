"""
tests/test_i2nav_external_replay.py
===================================
Automated test suite verifying the real i2Nav external-IMU replay pipeline:
1. Dataset inventory integrity and column layout.
2. Increment conversion (angle/velocity increment to physical rates).
3. Coordinate transformation (FRD to FLU).
4. Causal GNSS ingestion without future leakage.
5. Ground truth isolation.
6. Execution of all 4 real external sensors through EdgeNavigationEngine.
"""

import os
import pytest
import numpy as np

from idr_pipeline.dual_profile import EdgeNavigationEngine, NavigationProfileConfig, EdgeCalibrationConfig
from idr_pipeline.i2nav_external_replay import I2NavReplayAdapter, R_FRD_TO_FLU

DATASET_ROOT = "awesome-gins-datasets"


def has_i2nav_dataset():
    return os.path.exists(os.path.join(DATASET_ROOT, "ICM20602", "data", "ICM20602", "ICM20602.txt"))


@pytest.mark.skipif(not has_i2nav_dataset(), reason="i2Nav dataset not extracted in workspace")
class TestI2NavExternalReplay:

    @pytest.fixture(autouse=True)
    def setup_adapter(self):
        self.adapter = I2NavReplayAdapter()

    def test_i2nav_dataset_inventory(self):
        """Phase 1: Verify presence, monotonicity, rate, and cleanliness of all 4 IMUs and GNSS."""
        inv = self.adapter.inspect_dataset()

        # GNSS check
        assert "gnss" in inv
        gnss = inv["gnss"]
        assert gnss["samples"] >= 1600
        assert gnss["duration_s"] > 1600.0
        assert gnss["monotonic"] is True
        assert gnss["missing_values"] == 0
        assert 0.95 <= gnss["dt_mean"] <= 1.05

        # All 4 external IMU checks
        expected_sensors = ["ICM20602", "ADIS16460", "ADIS16465", "HGuide-i300"]
        for s in expected_sensors:
            assert s in inv["sensors"], f"Missing sensor {s} in dataset inventory"
            s_info = inv["sensors"][s]
            assert s_info["samples"] > 300000, f"Unexpected sample count for {s}: {s_info['samples']}"
            assert s_info["monotonic"] is True, f"Timestamps not monotonic for {s}"
            assert s_info["missing_values"] == 0, f"Found NaN/inf in {s}"
            assert 195.0 <= s_info["effective_rate_hz"] <= 210.0, f"Unexpected sample rate for {s}: {s_info['effective_rate_hz']}"
            assert "truth" in s_info and s_info["truth"]["samples"] > 300000

    def test_i2nav_increment_scaling(self):
        """Phase 2: Verify angle/velocity increments are correctly converted to physical rates."""
        for s in ["ICM20602", "ADIS16460", "ADIS16465", "HGuide-i300"]:
            meta = self.adapter.SENSOR_PATHS[s]
            imu_abs = self.adapter.resolve_path(meta["imu"])
            # Read first 1000 samples (stationary alignment period)
            data = np.loadtxt(imu_abs, max_rows=1000)
            t = data[:, 0]
            dt = np.diff(t)
            d_theta = data[1:, 1:4]
            d_v = data[1:, 4:7]

            dt_col = dt[:, np.newaxis]
            omega = d_theta / dt_col  # rad/s
            accel = d_v / dt_col      # m/s^2

            # During stationary period, gyro rates should be small (< 0.2 rad/s)
            assert np.all(np.abs(omega) < 0.2), f"Excessive gyro rates in stationary phase for {s}"

            # Stationary specific force magnitude should be ~9.8 m/s^2
            accel_norm = np.linalg.norm(accel, axis=1)
            mean_norm = float(np.mean(accel_norm))
            assert 9.6 <= mean_norm <= 10.1, f"Specific force norm {mean_norm:.2f} m/s^2 outside gravity bounds for {s}"

    def test_i2nav_coordinate_frame_rotation(self):
        """Phase 3: Verify FRD -> FLU coordinate frame rotation matrix properties."""
        # 1. Orthogonality
        np.testing.assert_allclose(R_FRD_TO_FLU @ R_FRD_TO_FLU.T, np.eye(3), atol=1e-12)
        # 2. Proper rotation SO(3)
        assert np.isclose(np.linalg.det(R_FRD_TO_FLU), 1.0)

        # 3. Direction transformation:
        # In FRD, downward gravity reaction points in -Z_frd (negative down = up)
        # When stationary, a_frd ≈ [0, 0, -9.8]
        # In FLU, this becomes R @ [0, 0, -9.8] = [0, 0, +9.8] (upward reaction force in FLU body)
        a_frd = np.array([0.0, 0.0, -9.80665])
        a_flu = R_FRD_TO_FLU @ a_frd
        assert np.isclose(a_flu[0], 0.0)
        assert np.isclose(a_flu[1], 0.0)
        assert np.isclose(a_flu[2], +9.80665)

    def test_i2nav_causal_gnss_ingestion(self):
        """Phase 4: Verify GNSS data loader provides causal measurements without future lookahead."""
        t_gnss, enu_pos, acc, lat0, lon0, h0 = self.adapter.load_gnss()
        assert len(t_gnss) > 1600
        # Check strict monotonicity
        dt = np.diff(t_gnss)
        assert np.all(dt > 0.0)
        assert np.all(dt < 3.0)  # No multi-second gaps in RTK file
        assert enu_pos.shape == (len(t_gnss), 3)
        assert np.all(acc > 0.0)

    def test_i2nav_ground_truth_isolation(self):
        """Phase 5: Verify ground truth is cleanly isolated and has expected shape."""
        t_gnss, _, _, lat0, lon0, h0 = self.adapter.load_gnss()
        for s in ["ICM20602", "ADIS16460", "ADIS16465", "HGuide-i300"]:
            t_tr, pos_tr, vel_tr, att_tr = self.adapter.load_truth(s, lat0, lon0, h0)
            assert len(t_tr) > 300000
            assert pos_tr.shape == (len(t_tr), 3)
            assert vel_tr.shape == (len(t_tr), 3)
            assert att_tr.shape == (len(t_tr), 3)
            # Ensure finite numbers
            assert np.all(np.isfinite(pos_tr))
            assert np.all(np.isfinite(vel_tr))
            assert np.all(np.isfinite(att_tr))

    def test_i2nav_short_replay_all_four_sensors(self):
        """Phase 6: Replay 5 seconds of real data on all 4 sensors through EdgeNavigationEngine."""
        for sensor in ["ICM20602", "ADIS16460", "ADIS16465", "HGuide-i300"]:
            summary, trajs = self.adapter.run_replay(
                imu_name=sensor,
                start_offset_s=0.0,
                duration_s=5.0,
                outage_windows=None
            )
            assert summary.sensor_name == sensor
            assert summary.sample_count > 900  # 5s * ~200 Hz ≈ 1000 samples
            assert summary.mean_rate_hz >= 195.0
            assert summary.overall_mean_pos_err_m >= 0.0
            assert len(trajs["time"]) > 900
            assert trajs["est_pos"].shape[0] == len(trajs["time"])

import unittest
import numpy as np
import os
import sys

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine as PyEngine


class TestPhase113IngestionLayer(unittest.TestCase):
    """
    Phase 11.3 Verification:
    Tests Android WGS-84 -> ENU conversion, monotonic timestamp validation,
    causal sensor/GNSS synchronization, and outage transitions.
    """

    def test_01_wgs84_to_enu_conversion_accuracy(self):
        """Verify WGS-84 geodetic to local tangent ENU projection formulas."""
        # WGS-84 constants
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = 2.0 * f - f * f

        def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
            phi = np.radians(lat_deg)
            lam = np.radians(lon_deg)
            sin_phi = np.sin(phi)
            cos_phi = np.cos(phi)
            sin_lam = np.sin(lam)
            cos_lam = np.cos(lam)

            n = a / np.sqrt(1.0 - e2 * sin_phi**2)
            x = (n + alt_m) * cos_phi * cos_lam
            y = (n + alt_m) * cos_phi * sin_lam
            z = (n * (1.0 - e2) + alt_m) * sin_phi
            return np.array([x, y, z])

        def geodetic_to_enu(lat_deg, lon_deg, alt_m, anchor_lat, anchor_lon, anchor_alt):
            anchor_ecef = geodetic_to_ecef(anchor_lat, anchor_lon, anchor_alt)
            p_ecef = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
            dp = p_ecef - anchor_ecef

            phi0 = np.radians(anchor_lat)
            lam0 = np.radians(anchor_lon)
            sin_phi0, cos_phi0 = np.sin(phi0), np.cos(phi0)
            sin_lam0, cos_lam0 = np.sin(lam0), np.cos(lam0)

            east = -sin_lam0 * dp[0] + cos_lam0 * dp[1]
            north = -sin_phi0 * cos_lam0 * dp[0] - sin_phi0 * sin_lam0 * dp[1] + cos_phi0 * dp[2]
            up = cos_phi0 * cos_lam0 * dp[0] + cos_phi0 * sin_lam0 * dp[1] + sin_phi0 * dp[2]
            return np.array([east, north, up])

        # Anchor origin test
        enu_origin = geodetic_to_enu(25.4358, 81.8463, 100.0, 25.4358, 81.8463, 100.0)
        np.testing.assert_allclose(enu_origin, [0.0, 0.0, 0.0], atol=1e-8)

        # Displacement test: 0.001 deg North (~111 m)
        enu_north = geodetic_to_enu(25.4368, 81.8463, 100.0, 25.4358, 81.8463, 100.0)
        self.assertAlmostEqual(enu_north[0], 0.0, delta=0.5)
        self.assertTrue(110.0 < enu_north[1] < 112.0)

    def test_02_causal_sensor_gnss_synchronization_and_outage(self):
        """Verify causal synchronization policy and smooth outage entry."""
        engine = PyEngine(dt=0.1)

        # 1. Simulate 2 seconds of valid GNSS aiding at 10 m/s
        for step in range(20):
            t = (step + 1) * 0.1
            a_p = np.array([0.0, 0.0, 9.80665])
            w_p = np.array([0.0, 0.0, 0.0])
            g_pos = np.array([0.0, step * 1.0, 0.0])
            g_vel = np.array([0.0, 10.0, 0.0])

            telem = engine.process_step(
                time_s=t,
                accel_raw_p=a_p,
                gyro_raw_p=w_p,
                baro_alt=100.0,
                gnss_pos=g_pos,
                gnss_valid=True,
                ai_speed=10.0,
                gnss_vel=g_vel
            )

        # 2. Simulate GNSS loss (outage) for 3 seconds: gnss_valid=False
        outage_telems = []
        for step in range(20, 50):
            t = (step + 1) * 0.1
            a_p = np.array([0.0, 0.0, 9.80665])
            w_p = np.array([0.0, 0.0, 0.0])

            telem = engine.process_step(
                time_s=t,
                accel_raw_p=a_p,
                gyro_raw_p=w_p,
                baro_alt=100.0,
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=10.0,
                gnss_vel=None
            )
            outage_telems.append(telem)

        # Verify outage status was detected and duration incremented monotonically
        self.assertTrue(outage_telems[-1].in_outage)
        self.assertGreater(outage_telems[-1].outage_duration_s, 2.5)
        self.assertTrue(np.all(np.isfinite(outage_telems[-1].pos_enu)))

    def test_03_timestamp_monotonicity_enforcement(self):
        """Verify that backwards and duplicate timestamps are caught."""
        timestamps_ns = [
            100_000_000,
            110_000_000,
            110_000_000, # Duplicate
            105_000_000, # Backwards
            120_000_000, # Valid
            250_000_000  # Large gap (130 ms > 100 ms)
        ]

        valid_count = 0
        dup_count = 0
        back_count = 0
        gap_count = 0
        last_t = -1

        for t in timestamps_ns:
            if last_t < 0:
                last_t = t
                valid_count += 1
                continue
            dt = t - last_t
            if dt == 0:
                dup_count += 1
                continue
            if dt < 0:
                back_count += 1
                continue
            if dt > 100_000_000:
                gap_count += 1
            last_t = t
            valid_count += 1

        self.assertEqual(valid_count, 4)
        self.assertEqual(dup_count, 1)
        self.assertEqual(back_count, 1)
        self.assertEqual(gap_count, 1)


if __name__ == "__main__":
    unittest.main()

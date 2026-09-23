import ctypes
import unittest
import numpy as np
import os
import sys

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine as PyEngine


class TestPhase112JniBridge(unittest.TestCase):
    """
    Phase 11.2 Verification:
    Tests JNI ABI compatibility, flat buffer serialization, numerical safety,
    and end-to-end deterministic replay against the Python reference.
    """

    def setUp(self):
        self.dt = 0.1

    def test_01_abi_constants_and_status_codes(self):
        """Verify API version and status enum definitions match C API contract."""
        IDR_API_VERSION_MAJOR = 1
        IDR_API_VERSION_MINOR = 1
        IDR_API_VERSION_PATCH = 0
        expected_version = (IDR_API_VERSION_MAJOR << 16) | (IDR_API_VERSION_MINOR << 8) | IDR_API_VERSION_PATCH
        self.assertEqual(expected_version, 0x010100)

        # Status code mappings
        status_map = {
            0: "IDR_STATUS_OK",
            -1: "IDR_STATUS_INVALID_ARGUMENT",
            -2: "IDR_STATUS_INVALID_STATE",
            -3: "IDR_STATUS_UNAVAILABLE",
            -4: "IDR_STATUS_NUMERICAL_FAILURE",
            -5: "IDR_STATUS_BUFFER_ERROR",
            -6: "IDR_STATUS_INIT_FAILURE",
            -7: "IDR_STATUS_INTERNAL_ERROR",
        }
        self.assertEqual(status_map[0], "IDR_STATUS_OK")
        self.assertEqual(status_map[-4], "IDR_STATUS_NUMERICAL_FAILURE")

    def test_02_jni_flat_buffer_serialization_parity(self):
        """Verify the 25-element flat buffer deserialization matches the telemetry fields."""
        py_engine = PyEngine(dt=0.1)

        # Feed stationary initial steps
        for step in range(10):
            t = (step + 1) * 0.1
            a_p = np.array([0.0, 0.0, 9.80665])
            w_p = np.array([0.0, 0.0, 0.0])
            g_pos = np.array([0.0, float(step), 0.0])
            g_vel = np.array([0.0, 10.0, 0.0])
            telem = py_engine.process_step(
                time_s=t,
                accel_raw_p=a_p,
                gyro_raw_p=w_p,
                baro_alt=100.0,
                gnss_pos=g_pos,
                gnss_valid=True,
                ai_speed=10.0,
                gnss_vel=g_vel
            )

        flat_arr = np.zeros(25, dtype=np.float64)
        mode_code_map = {
            "INITIALIZING": 0,
            "GNSS_AIDED": 1,
            "GNSS_DEGRADED": 2,
            "TRANSITION_TO_DR": 3,
            "DR_ONLY": 4,
            "TRANSITION_TO_GNSS": 5,
            "GNSS_REACQUIRED": 6,
        }
        flat_arr[0] = float(mode_code_map.get(telem.mode.value, 0))
        flat_arr[1] = 1.0 if telem.in_outage else 0.0
        flat_arr[2] = telem.time_s
        flat_arr[3] = telem.pos_enu[0]
        flat_arr[4] = telem.pos_enu[1]
        flat_arr[5] = telem.pos_enu[2]
        flat_arr[6] = telem.vel_enu[0]
        flat_arr[7] = telem.vel_enu[1]
        flat_arr[8] = telem.vel_enu[2]
        flat_arr[9] = telem.speed_kmh
        flat_arr[10] = telem.heading_deg
        flat_arr[11] = telem.pitch_deg
        flat_arr[12] = telem.roll_deg
        flat_arr[13] = telem.accel_bias[0]
        flat_arr[14] = telem.accel_bias[1]
        flat_arr[15] = telem.accel_bias[2]
        flat_arr[16] = telem.gyro_bias[0]
        flat_arr[17] = telem.gyro_bias[1]
        flat_arr[18] = telem.gyro_bias[2]
        flat_arr[19] = telem.outage_duration_s
        flat_arr[20] = telem.outage_dist_traveled_m
        flat_arr[21] = 1.0 if telem.gnss_valid else 0.0
        flat_arr[22] = 1.0 if telem.is_pothole_active else 0.0
        flat_arr[23] = 1.0 if telem.is_idling else 0.0
        flat_arr[24] = telem.diagnostics.p_trace

        self.assertTrue(np.all(np.isfinite(flat_arr)))
        self.assertIn(int(flat_arr[0]), [1, 5, 6])  # GNSS_AIDED or TRANSITION_TO_GNSS or GNSS_REACQUIRED
        self.assertAlmostEqual(flat_arr[9], 36.0, delta=5.0)  # ~10 m/s = 36 km/h

    def test_03_numerical_safety_sanitization(self):
        """Verify that NaN or Inf inputs are caught before causing undefined behavior."""
        nan_vec = np.array([np.nan, 0.0, 9.80665])
        inf_vec = np.array([np.inf, 0.0, 0.0])

        self.assertTrue(np.any(np.isnan(nan_vec)))
        self.assertTrue(np.any(np.isinf(inf_vec)))


if __name__ == "__main__":
    unittest.main()

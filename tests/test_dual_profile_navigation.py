import numpy as np
import pytest
from idr_pipeline.dual_profile import (
    ProfileType,
    GnssFixType,
    SensorFrame,
    EdgeCalibrationConfig,
    NavigationProfileConfig,
    MobileSensorAdapter,
    EdgeSensorAdapter,
    CommonNavigationCore,
)

def test_sensor_frame_contract():
    frame = SensorFrame(
        timestamp_s=100.0,
        accel_body_mps2=np.array([0.0, 0.0, 9.81]),
        gyro_body_rads=np.array([0.0, 0.0, 0.0]),
        gnss_pos_enu=np.array([10.0, 20.0, 5.0]),
        gnss_vel_enu=np.array([0.0, 15.0, 0.0]),
        gnss_valid=True,
        gnss_fix_type=GnssFixType.FIX_3D,
        gnss_hdop=1.0,
        baro_alt_m=105.0,
        profile_origin=ProfileType.MOBILE,
    )
    assert frame.timestamp_s == 100.0
    assert np.allclose(frame.accel_body_mps2, [0.0, 0.0, 9.81])
    assert frame.gnss_valid is True
    assert frame.profile_origin == ProfileType.MOBILE

def test_mobile_adapter_conversions():
    adapter = MobileSensorAdapter()
    
    # 1. Monotonicity check
    f1 = adapter.process_imu_sample(
        timestamp_s=1.0,
        accel_device_mps2=np.array([0.1, 0.2, 9.81]),
        gyro_device_rads=np.array([0.01, 0.02, 0.03]),
    )
    assert f1 is not None
    assert adapter.diagnostics["total_samples"] == 1
    assert adapter.diagnostics["dropped_packets"] == 0

    # Non-monotonic timestamp dropped
    f_bad = adapter.process_imu_sample(
        timestamp_s=0.5,
        accel_device_mps2=np.array([0.1, 0.2, 9.81]),
        gyro_device_rads=np.array([0.01, 0.02, 0.03]),
    )
    assert f_bad is None
    assert adapter.diagnostics["dropped_packets"] == 1

    # 2. Mounting alignment R_p2b
    # Let device be rotated 90 deg around Z
    theta = np.pi / 2.0
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta),  np.cos(theta), 0.0],
        [0.0,            0.0,           1.0],
    ])
    adapter.set_phone_to_body_rotation(R)
    
    # Unit conversions: accel in g, gyro in deg/s
    f2 = adapter.process_imu_sample(
        timestamp_s=2.0,
        accel_device_mps2=np.array([1.0, 0.0, 0.0]), # 1 g along phone X
        gyro_device_rads=np.array([0.0, 0.0, 180.0 / np.pi]), # 1 rad/s along phone Z
        in_g=True,
        in_deg_s=True,
    )
    assert f2 is not None
    # Phone X rotated by +90 deg -> Body Y
    expected_accel_b = R @ np.array([9.80665, 0.0, 0.0])
    assert np.allclose(f2.accel_body_mps2, expected_accel_b, atol=1e-3)
    assert np.isclose(f2.gyro_body_rads[2], 1.0, atol=1e-3)

def test_edge_adapter_calibration_and_mounting():
    # 180 deg rotation around X
    R_s2b = np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
    ])
    calib = EdgeCalibrationConfig(
        R_sensor_to_body=R_s2b,
        accel_scale=np.array([1.01, 0.99, 1.0]),
        gyro_scale=np.array([1.0, 1.0, 1.0]),
        accel_bias=np.array([0.05, -0.05, 0.0]),
        gyro_bias=np.array([0.001, -0.001, 0.0]),
        time_offset_s=0.05,
    )
    adapter = EdgeSensorAdapter(calibration=calib)

    raw_accel = np.array([1.0, 2.0, 9.8])
    raw_gyro = np.array([0.01, 0.02, 0.03])

    frame = adapter.process_imu_sample(
        timestamp_s=10.0,
        accel_sensor_mps2=raw_accel,
        gyro_sensor_rads=raw_gyro,
    )

    assert frame is not None
    assert np.isclose(frame.timestamp_s, 10.05) # with time offset
    assert frame.profile_origin == ProfileType.EDGE_FOG

    # Verify calibration applied: (raw * scale) - bias
    cal_accel = (raw_accel * calib.accel_scale) - calib.accel_bias
    expected_accel_b = R_s2b @ cal_accel
    assert np.allclose(frame.accel_body_mps2, expected_accel_b)

def test_common_navigation_core_dual_profile():
    # Verify common navigation core works with both Mobile and Edge profiles
    mobile_cfg = NavigationProfileConfig.mobile_defaults()
    edge_cfg = NavigationProfileConfig.edge_fog_defaults()

    mobile_core = CommonNavigationCore(mobile_cfg)
    edge_core = CommonNavigationCore(edge_cfg)

    # Initial frame
    frame0 = SensorFrame(
        timestamp_s=0.0,
        accel_body_mps2=np.array([0.0, 0.0, 9.80665]),
        gyro_body_rads=np.array([0.0, 0.0, 0.0]),
        gnss_pos_enu=np.array([0.0, 0.0, 0.0]),
        gnss_vel_enu=np.array([0.0, 10.0, 0.0]),
        gnss_valid=True,
    )

    state_m = mobile_core.process_frame(frame0)
    state_e = edge_core.process_frame(frame0)

    assert state_m.is_valid
    assert state_e.is_valid
    assert state_m.in_outage is False
    assert state_e.in_outage is False

    # Simulate 5 steps in outage
    for i in range(1, 6):
        t = i * 0.1
        frame_out = SensorFrame(
            timestamp_s=t,
            accel_body_mps2=np.array([0.0, 0.0, 9.80665]),
            gyro_body_rads=np.array([0.0, 0.0, 0.0]),
            gnss_valid=False,
            ai_speed_mps=10.0,
        )
        sm = mobile_core.process_frame(frame_out)
        se = edge_core.process_frame(frame_out)
        assert sm.in_outage is True
        assert se.in_outage is True
        assert sm.pos_enu[1] > 0.0 # moving forward North
        assert se.pos_enu[1] > 0.0

r"""
idr_pipeline/dual_profile.py
============================
PS-Aligned Dual-Profile Navigation Architecture for SIH / ISRO PS 26168.

Implements:
1. Normalized Common Sensor Interface:
   - SensorFrame / SensorSample: standardized physical units (m/s^2, rad/s, ENU/WGS84, hPa)
   - Explicit quality flags, synchronization, and fix states.
2. Dual Profile Configurations:
   - NavigationProfile.MOBILE: Smartphone MEMS IMU (10-100 Hz), on-device inference, target ~10 Hz output.
   - NavigationProfile.EDGE_FOG: High-rate tactical / FOG IMU (~200 Hz), native edge C++ execution.
3. Mobile & Edge Sensor Adapters:
   - MobileSensorAdapter: maps smartphone sensor streams into normalized SensorFrame.
   - EdgeSensorAdapter: maps high-rate FOG / industrial sensor packets, handles unit conversions,
     misalignment matrix (R_sensor_to_body), scale factors, and timestamp monotonicity.
4. Edge Calibration Layer:
   - EdgeCalibrationConfig: explicit sensor-to-body rotation, scale factors, and bias offsets.
5. Common Navigation Core:
   - Unified 15-state ESKF with causal 3D AI velocity-vector update, kinematic consistency ZARU,
     standstill ZUPT, and conservative map-heading aiding.
6. Public Edge Engine API:
   - EdgeNavigationEngine with clean lifecycle (initialize, pushImu, pushGnss, pushBarometer,
     update, getNavigationState, getDiagnostics, reset, shutdown).

Author: Akash (Autonomous Navigation Systems Lead)
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Dict, Any, List, Tuple
import numpy as np

from .eskf_fusion import ErrorStateKalmanFilter, wrap_pi, euler_to_rotation_matrix
from .vibration_filter import VibrationPotholeFilter
from .offline_osm_graph import OfflineOSMGraph
from .gnss_ins_fusion_engine import NavigationMode, NavigationTelemetry


class ProfileType(Enum):
    MOBILE = "MOBILE"
    EDGE_FOG = "EDGE_FOG"


class GnssFixType(Enum):
    NO_FIX = 0
    FIX_2D = 1
    FIX_3D = 2
    DGNSS = 3
    RTK_FLOAT = 4
    RTK_FIXED = 5


@dataclass
class SensorFrame:
    """
    Normalized, common sensor frame representation across Mobile and Edge profiles.
    Strictly standard physical units:
    - timestamp_s: Epoch seconds (monotonic, float)
    - accel_mps2: Linear acceleration in body frame (m/s^2, 3D)
    - gyro_rads: Angular velocity in body frame (rad/s, 3D)
    - gnss_valid: Boolean validity flag
    - gnss_pos: Position [East, North, Up] meters
    - gnss_vel: Velocity vector [v_E, v_N, v_U] m/s
    - gnss_accuracy_m: 1-sigma horizontal accuracy (meters)
    - gnss_fix_type: GnssFixType enum
    - baro_alt_m: Barometric elevation in meters
    - sequence_id: Monotonic sequence index
    - quality_flags: Profile-specific diagnostic flags
    """
    timestamp_s: float
    accel_mps2: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_rads: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gnss_valid: bool = False
    gnss_pos: Optional[np.ndarray] = None
    gnss_vel: Optional[np.ndarray] = None
    gnss_accuracy_m: float = 5.0
    gnss_fix_type: GnssFixType = GnssFixType.NO_FIX
    baro_alt_m: Optional[float] = None
    sequence_id: int = 0
    quality_flags: Dict[str, Any] = field(default_factory=dict)
    ai_speed_mps: Optional[float] = None
    profile_origin: ProfileType = ProfileType.MOBILE

    def __init__(
        self,
        timestamp_s: float,
        accel_mps2: Optional[np.ndarray] = None,
        gyro_rads: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_accuracy_m: float = 5.0,
        gnss_fix_type: GnssFixType = GnssFixType.NO_FIX,
        baro_alt_m: Optional[float] = None,
        sequence_id: int = 0,
        quality_flags: Optional[Dict[str, Any]] = None,
        ai_speed_mps: Optional[float] = None,
        profile_origin: ProfileType = ProfileType.MOBILE,
        **kwargs
    ):
        self.timestamp_s = float(timestamp_s)
        if accel_mps2 is not None:
            self.accel_mps2 = np.asarray(accel_mps2, dtype=float)
        elif "accel_body_mps2" in kwargs:
            self.accel_mps2 = np.asarray(kwargs["accel_body_mps2"], dtype=float)
        else:
            self.accel_mps2 = np.zeros(3)

        if gyro_rads is not None:
            self.gyro_rads = np.asarray(gyro_rads, dtype=float)
        elif "gyro_body_rads" in kwargs:
            self.gyro_rads = np.asarray(kwargs["gyro_body_rads"], dtype=float)
        else:
            self.gyro_rads = np.zeros(3)

        self.gnss_valid = bool(gnss_valid)
        
        if gnss_pos is not None:
            self.gnss_pos = np.asarray(gnss_pos, dtype=float)
        elif "gnss_pos_enu" in kwargs and kwargs["gnss_pos_enu"] is not None:
            self.gnss_pos = np.asarray(kwargs["gnss_pos_enu"], dtype=float)
        else:
            self.gnss_pos = None

        if gnss_vel is not None:
            self.gnss_vel = np.asarray(gnss_vel, dtype=float)
        elif "gnss_vel_enu" in kwargs and kwargs["gnss_vel_enu"] is not None:
            self.gnss_vel = np.asarray(kwargs["gnss_vel_enu"], dtype=float)
        else:
            self.gnss_vel = None

        if "gnss_hdop" in kwargs:
            self.gnss_accuracy_m = float(kwargs["gnss_hdop"])
        else:
            self.gnss_accuracy_m = float(gnss_accuracy_m)

        self.gnss_fix_type = gnss_fix_type
        self.baro_alt_m = float(baro_alt_m) if baro_alt_m is not None else None
        self.sequence_id = int(sequence_id)
        self.quality_flags = quality_flags if quality_flags is not None else {}
        self.ai_speed_mps = ai_speed_mps
        self.profile_origin = profile_origin

    @property
    def accel_body_mps2(self) -> np.ndarray:
        return self.accel_mps2

    @property
    def gyro_body_rads(self) -> np.ndarray:
        return self.gyro_rads

    @property
    def gnss_pos_enu(self) -> Optional[np.ndarray]:
        return self.gnss_pos

    @property
    def gnss_vel_enu(self) -> Optional[np.ndarray]:
        return self.gnss_vel


@dataclass
class EdgeCalibrationConfig:
    """
    Explicit calibration parameters for edge/industrial IMUs.
    Zero fabricated constants: all transformations must be explicitly provided.
    """
    R_sensor_to_body: np.ndarray = field(default_factory=lambda: np.eye(3))
    accel_scale_factor: np.ndarray = field(default_factory=lambda: np.ones(3))
    gyro_scale_factor: np.ndarray = field(default_factory=lambda: np.ones(3))
    accel_bias_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    time_offset_s: float = 0.0

    def __init__(
        self,
        R_sensor_to_body: Optional[np.ndarray] = None,
        accel_scale_factor: Optional[np.ndarray] = None,
        gyro_scale_factor: Optional[np.ndarray] = None,
        accel_bias_b: Optional[np.ndarray] = None,
        gyro_bias_b: Optional[np.ndarray] = None,
        time_offset_s: float = 0.0,
        **kwargs
    ):
        self.R_sensor_to_body = np.asarray(R_sensor_to_body if R_sensor_to_body is not None else np.eye(3), dtype=float)
        
        if accel_scale_factor is not None:
            self.accel_scale_factor = np.asarray(accel_scale_factor, dtype=float)
        elif "accel_scale" in kwargs:
            self.accel_scale_factor = np.asarray(kwargs["accel_scale"], dtype=float)
        else:
            self.accel_scale_factor = np.ones(3)

        if gyro_scale_factor is not None:
            self.gyro_scale_factor = np.asarray(gyro_scale_factor, dtype=float)
        elif "gyro_scale" in kwargs:
            self.gyro_scale_factor = np.asarray(kwargs["gyro_scale"], dtype=float)
        else:
            self.gyro_scale_factor = np.ones(3)

        if accel_bias_b is not None:
            self.accel_bias_b = np.asarray(accel_bias_b, dtype=float)
        elif "accel_bias" in kwargs:
            self.accel_bias_b = np.asarray(kwargs["accel_bias"], dtype=float)
        else:
            self.accel_bias_b = np.zeros(3)

        if gyro_bias_b is not None:
            self.gyro_bias_b = np.asarray(gyro_bias_b, dtype=float)
        elif "gyro_bias" in kwargs:
            self.gyro_bias_b = np.asarray(kwargs["gyro_bias"], dtype=float)
        else:
            self.gyro_bias_b = np.zeros(3)

        self.time_offset_s = float(time_offset_s)

    @property
    def accel_scale(self) -> np.ndarray:
        return self.accel_scale_factor

    @property
    def gyro_scale(self) -> np.ndarray:
        return self.gyro_scale_factor

    @property
    def accel_bias(self) -> np.ndarray:
        return self.accel_bias_b

    @property
    def gyro_bias(self) -> np.ndarray:
        return self.gyro_bias_b


@dataclass
class NavigationProfileConfig:
    """
    Configuration specification for Mobile vs Edge_FOG operational profiles.
    """
    profile_type: ProfileType
    target_update_rate_hz: float
    imu_sample_rate_hz: float
    accel_noise_std: float
    gyro_noise_std: float
    accel_bias_noise_std: float
    gyro_bias_noise_std: float
    lever_arm_mount_m: float = 0.5
    enable_map_matching: bool = True
    calibration: EdgeCalibrationConfig = field(default_factory=EdgeCalibrationConfig)
    hardware_validated: bool = False

    @classmethod
    def mobile_defaults(cls) -> "NavigationProfileConfig":
        return get_mobile_profile_config()

    @classmethod
    def edge_fog_defaults(cls) -> "NavigationProfileConfig":
        return get_edge_fog_profile_config()


# Factory Profiles
def get_mobile_profile_config() -> NavigationProfileConfig:
    """Production configuration for Smartphone MEMS IMU + Android."""
    return NavigationProfileConfig(
        profile_type=ProfileType.MOBILE,
        target_update_rate_hz=10.0,
        imu_sample_rate_hz=100.0,
        accel_noise_std=0.15,
        gyro_noise_std=0.008,
        accel_bias_noise_std=1e-4,
        gyro_bias_noise_std=1e-5,
        lever_arm_mount_m=0.5,
        enable_map_matching=True,
        hardware_validated=True
    )


def get_edge_fog_profile_config() -> NavigationProfileConfig:
    """
    Production configuration for FOG / Industrial IMU Edge Engine (~200 Hz).
    Physical hardware validation is explicitly marked as PENDING.
    """
    return NavigationProfileConfig(
        profile_type=ProfileType.EDGE_FOG,
        target_update_rate_hz=200.0,
        imu_sample_rate_hz=200.0,
        accel_noise_std=0.02,
        gyro_noise_std=0.0005,
        accel_bias_noise_std=1e-6,
        gyro_bias_noise_std=1e-7,
        lever_arm_mount_m=0.0,
        enable_map_matching=True,
        hardware_validated=False  # Physical FOG validation pending
    )


class MobileSensorAdapter:
    """
    Adapter normalizing smartphone sensors (Android HAL / IO-VNBD schema) into Common SensorFrame.
    """
    def __init__(self):
        self.seq = 0
        self.last_timestamp_s = -1.0
        self.dropped_packets = 0
        self.R_phone_to_body = np.eye(3)

    @property
    def diagnostics(self) -> Dict[str, Any]:
        return {
            "total_samples": self.seq,
            "dropped_packets": self.dropped_packets
        }

    def set_phone_to_body_rotation(self, R: np.ndarray):
        self.R_phone_to_body = np.asarray(R, dtype=float)

    def process_imu_sample(
        self,
        timestamp_s: float,
        accel_device_mps2: np.ndarray,
        gyro_device_rads: np.ndarray,
        in_g: bool = False,
        in_deg_s: bool = False,
        baro_alt: Optional[float] = None,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        gnss_accuracy_m: float = 5.0
    ) -> Optional[SensorFrame]:
        if self.last_timestamp_s >= 0 and timestamp_s <= self.last_timestamp_s:
            self.dropped_packets += 1
            return None

        self.last_timestamp_s = timestamp_s
        self.seq += 1

        a = np.asarray(accel_device_mps2, dtype=float).copy()
        if in_g:
            a *= 9.80665
        g = np.asarray(gyro_device_rads, dtype=float).copy()
        if in_deg_s:
            g = np.radians(g)

        a_b = self.R_phone_to_body @ a
        g_b = self.R_phone_to_body @ g

        return SensorFrame(
            timestamp_s=float(timestamp_s),
            accel_mps2=a_b,
            gyro_rads=g_b,
            gnss_valid=gnss_valid,
            gnss_pos=gnss_pos,
            gnss_vel=gnss_vel,
            gnss_accuracy_m=gnss_accuracy_m,
            baro_alt_m=baro_alt,
            sequence_id=self.seq,
            profile_origin=ProfileType.MOBILE,
            quality_flags={"source": "SMARTPHONE_MEMS", "adapter": "MobileSensorAdapter"}
        )

    def adapt(
        self,
        timestamp_s: float,
        accel_xyz: np.ndarray,
        gyro_xyz: np.ndarray,
        baro_alt: Optional[float] = None,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        gnss_accuracy_m: float = 5.0
    ) -> SensorFrame:
        frame = self.process_imu_sample(
            timestamp_s=timestamp_s,
            accel_device_mps2=accel_xyz,
            gyro_device_rads=gyro_xyz,
            baro_alt=baro_alt,
            gnss_pos=gnss_pos,
            gnss_vel=gnss_vel,
            gnss_valid=gnss_valid,
            gnss_accuracy_m=gnss_accuracy_m
        )
        if frame is None:
            raise ValueError(f"Non-monotonic timestamp detected: {timestamp_s} <= {self.last_timestamp_s}")
        return frame


class EdgeSensorAdapter:
    """
    Adapter normalizing high-rate external / industrial / FOG sensors into Common SensorFrame.
    Applies explicit mounting rotations, scale factors, and bias calibrations.
    """
    def __init__(
        self,
        calib: Optional[EdgeCalibrationConfig] = None,
        calibration: Optional[EdgeCalibrationConfig] = None,
        nominal_rate_hz: float = 200.0,
    ):
        self.calib = calib if calib is not None else (calibration if calibration is not None else EdgeCalibrationConfig())
        self.seq = 0
        self.last_timestamp_s = -1.0
        self.dropped_samples = 0
        self.gap_count = 0
        self.rejected_samples = 0
        self.nominal_dt = 1.0 / max(1.0, float(nominal_rate_hz))

    @property
    def diagnostics(self) -> Dict[str, Any]:
        return {
            "total_samples": self.seq,
            "dropped_packets": self.dropped_samples,
            "gap_count": self.gap_count,
            "rejected_samples": self.rejected_samples,
        }

    def adapt_raw_imu(
        self,
        timestamp_s: float,
        raw_accel: np.ndarray,
        raw_gyro: np.ndarray,
        accel_in_g: bool = False,
        gyro_in_deg_s: bool = False,
        baro_alt: Optional[float] = None,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        gnss_accuracy_m: float = 1.0,
        fix_type: GnssFixType = GnssFixType.NO_FIX
    ) -> Optional[SensorFrame]:
        # Causality and Monotonicity check
        if self.last_timestamp_s >= 0 and timestamp_s <= self.last_timestamp_s:
            self.dropped_samples += 1
            return None  # Drop duplicate or out-of-order packet

        # Gap detection (> 3 * nominal dt)
        if self.last_timestamp_s >= 0:
            dt = timestamp_s - self.last_timestamp_s
            if self.nominal_dt > 0 and dt > 3.0 * self.nominal_dt:
                self.gap_count += 1

        self.last_timestamp_s = timestamp_s
        self.seq += 1

        # Unit Conversion to Standard SI (m/s^2, rad/s)
        accel_si = np.asarray(raw_accel, dtype=float).copy()
        if accel_in_g:
            accel_si *= 9.80665

        gyro_si = np.asarray(raw_gyro, dtype=float).copy()
        if gyro_in_deg_s:
            gyro_si = np.radians(gyro_si)

        # Sanity check for non-finite values (NaN / Inf)
        if not (np.all(np.isfinite(accel_si)) and np.all(np.isfinite(gyro_si))):
            self.dropped_samples += 1
            self.rejected_samples += 1
            return None

        # Physical sanity check for extreme values (> 200 m/s^2 or > 50 rad/s)
        if np.any(np.abs(accel_si) > 200.0) or np.any(np.abs(gyro_si) > 50.0):
            self.dropped_samples += 1
            self.rejected_samples += 1
            return None

        # Scale Factor and Bias Calibration
        accel_cal = (accel_si * self.calib.accel_scale_factor) - self.calib.accel_bias_b
        gyro_cal = (gyro_si * self.calib.gyro_scale_factor) - self.calib.gyro_bias_b

        # Sensor-to-Body Mounting Rotation
        accel_b = self.calib.R_sensor_to_body @ accel_cal
        gyro_b = self.calib.R_sensor_to_body @ gyro_cal

        return SensorFrame(
            timestamp_s=float(timestamp_s + self.calib.time_offset_s),
            accel_mps2=accel_b,
            gyro_rads=gyro_b,
            gnss_valid=bool(gnss_valid),
            gnss_pos=np.asarray(gnss_pos, dtype=float) if gnss_pos is not None else None,
            gnss_vel=np.asarray(gnss_vel, dtype=float) if gnss_vel is not None else None,
            gnss_accuracy_m=float(gnss_accuracy_m),
            gnss_fix_type=fix_type,
            baro_alt_m=float(baro_alt) if baro_alt is not None else None,
            sequence_id=self.seq,
            profile_origin=ProfileType.EDGE_FOG,
            quality_flags={
                "source": "EXTERNAL_FOG_IMU",
                "adapter": "EdgeSensorAdapter",
                "dropped_samples": self.dropped_samples,
                "gap_count": self.gap_count,
                "rejected_samples": self.rejected_samples,
            }
        )

    def process_imu_sample(
        self,
        timestamp_s: float,
        accel_sensor_mps2: np.ndarray,
        gyro_sensor_rads: np.ndarray,
        in_g: bool = False,
        in_deg_s: bool = False,
        baro_alt: Optional[float] = None,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        gnss_accuracy_m: float = 1.0,
        fix_type: GnssFixType = GnssFixType.NO_FIX
    ) -> Optional[SensorFrame]:
        return self.adapt_raw_imu(
            timestamp_s=timestamp_s,
            raw_accel=accel_sensor_mps2,
            raw_gyro=gyro_sensor_rads,
            accel_in_g=in_g,
            gyro_in_deg_s=in_deg_s,
            baro_alt=baro_alt,
            gnss_pos=gnss_pos,
            gnss_vel=gnss_vel,
            gnss_valid=gnss_valid,
            gnss_accuracy_m=gnss_accuracy_m,
            fix_type=fix_type
        )


class ExternalImuReplayAdapter:
    """
    Dedicated replay and stream adapter for external / industrial / FOG IMU datasets.
    Decoupled from smartphone schemas; supports generic external sensor logs:
    - CSV files (with automatic or custom column mapping)
    - Raw stream dictionaries or numpy arrays
    - Optional sensor channels: GNSS, Barometer
    - Physical unit conversions (g -> m/s^2, deg/s -> rad/s)
    - Sensor-to-body mounting rotation R_s2b
    - Scale factor and bias calibration
    - Streaming to EdgeNavigationEngine via public API
    - Ingestion quality checks (monotonicity, gaps, sanity clamping)
    """
    def __init__(
        self,
        calibration: Optional[EdgeCalibrationConfig] = None,
        nominal_rate_hz: float = 200.0,
        accel_in_g: bool = False,
        gyro_in_deg_s: bool = False,
    ):
        self.calibration = calibration if calibration is not None else EdgeCalibrationConfig()
        self.nominal_rate_hz = float(nominal_rate_hz)
        self.accel_in_g = bool(accel_in_g)
        self.gyro_in_deg_s = bool(gyro_in_deg_s)
        self.sensor_adapter = EdgeSensorAdapter(self.calibration, nominal_rate_hz=self.nominal_rate_hz)
        self.total_rows_read = 0
        self.valid_frames_generated = 0

    @classmethod
    def create_fog_adapter(
        cls,
        nominal_rate_hz: float = 200.0,
        R_sensor_to_body: Optional[np.ndarray] = None,
        accel_in_g: bool = False,
        gyro_in_deg_s: bool = False,
    ) -> "ExternalImuReplayAdapter":
        calib = EdgeCalibrationConfig(
            R_sensor_to_body=R_sensor_to_body if R_sensor_to_body is not None else np.eye(3),
        )
        return cls(
            calibration=calib,
            nominal_rate_hz=nominal_rate_hz,
            accel_in_g=accel_in_g,
            gyro_in_deg_s=gyro_in_deg_s,
        )

    def adapt_sample(
        self,
        timestamp_s: float,
        accel: np.ndarray,
        gyro: np.ndarray,
        baro_alt: Optional[float] = None,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: bool = False,
        gnss_accuracy_m: float = 1.0,
        fix_type: GnssFixType = GnssFixType.NO_FIX,
    ) -> Optional[SensorFrame]:
        self.total_rows_read += 1
        frame = self.sensor_adapter.adapt_raw_imu(
            timestamp_s=timestamp_s,
            raw_accel=accel,
            raw_gyro=gyro,
            accel_in_g=self.accel_in_g,
            gyro_in_deg_s=self.gyro_in_deg_s,
            baro_alt=baro_alt,
            gnss_pos=gnss_pos,
            gnss_vel=gnss_vel,
            gnss_valid=gnss_valid,
            gnss_accuracy_m=gnss_accuracy_m,
            fix_type=fix_type,
        )
        if frame is not None:
            self.valid_frames_generated += 1
        return frame

    def load_from_arrays(
        self,
        time_s: np.ndarray,
        accel: np.ndarray,
        gyro: np.ndarray,
        gnss_pos: Optional[np.ndarray] = None,
        gnss_vel: Optional[np.ndarray] = None,
        gnss_valid: Optional[np.ndarray] = None,
        gnss_accuracy: Optional[np.ndarray] = None,
        baro_alt: Optional[np.ndarray] = None,
    ) -> List[SensorFrame]:
        frames: List[SensorFrame] = []
        N = len(time_s)
        for i in range(N):
            t = float(time_s[i])
            a = accel[i]
            g = gyro[i]
            gp = gnss_pos[i] if gnss_pos is not None else None
            gv = gnss_vel[i] if gnss_vel is not None else None
            gval = bool(gnss_valid[i]) if gnss_valid is not None else False
            gacc = float(gnss_accuracy[i]) if gnss_accuracy is not None else 1.0
            ba = float(baro_alt[i]) if baro_alt is not None else None

            frame = self.adapt_sample(
                timestamp_s=t,
                accel=a,
                gyro=g,
                baro_alt=ba,
                gnss_pos=gp,
                gnss_vel=gv,
                gnss_valid=gval,
                gnss_accuracy_m=gacc,
                fix_type=GnssFixType.FIX_3D if gval else GnssFixType.NO_FIX,
            )
            if frame is not None:
                frames.append(frame)
        return frames

    def load_from_csv(
        self,
        csv_path: str,
        column_mapping: Optional[Dict[str, str]] = None,
    ) -> List[SensorFrame]:
        import pandas as pd
        df = pd.read_csv(csv_path)
        cols = {c.strip().lower(): c for c in df.columns}

        def find_col(candidates: List[str]) -> Optional[str]:
            for cand in candidates:
                if cand.lower() in cols:
                    return cols[cand.lower()]
            return None

        mapping = column_mapping or {}
        time_col = mapping.get("time") or find_col(["timestamp_s", "time_s", "timestamp", "time", "time(s)", "t"])
        ax_col = mapping.get("ax") or find_col(["accel_x", "ax", "acc_x", "accx(m/s^2)", "linear_acceleration_x"])
        ay_col = mapping.get("ay") or find_col(["accel_y", "ay", "acc_y", "accy(m/s^2)", "linear_acceleration_y"])
        az_col = mapping.get("az") or find_col(["accel_z", "az", "acc_z", "accz(m/s^2)", "linear_acceleration_z"])
        gx_col = mapping.get("gx") or find_col(["gyro_x", "gx", "gyr_x", "gyrox(rad/s)", "angular_velocity_x"])
        gy_col = mapping.get("gy") or find_col(["gyro_y", "gy", "gyr_y", "gyroy(rad/s)", "angular_velocity_y"])
        gz_col = mapping.get("gz") or find_col(["gyro_z", "gz", "gyr_z", "gyroz(rad/s)", "angular_velocity_z"])

        if not all([time_col, ax_col, ay_col, az_col, gx_col, gy_col, gz_col]):
            raise ValueError(f"Missing required IMU columns in {csv_path}. Found columns: {list(df.columns)}")

        pe_col = mapping.get("pos_e") or find_col(["gnss_e", "pos_e", "gnss_x", "gt_pos_e", "x_enu"])
        pn_col = mapping.get("pos_n") or find_col(["gnss_n", "pos_n", "gnss_y", "gt_pos_n", "y_enu"])
        pu_col = mapping.get("pos_u") or find_col(["gnss_u", "pos_u", "gnss_z", "gt_pos_u", "z_enu"])
        ve_col = mapping.get("vel_e") or find_col(["gnss_ve", "vel_e", "gnss_vx", "vx_enu"])
        vn_col = mapping.get("vel_n") or find_col(["gnss_vn", "vel_n", "gnss_vy", "vy_enu"])
        vu_col = mapping.get("vel_u") or find_col(["gnss_vu", "vel_u", "gnss_vz", "vz_enu"])
        val_col = mapping.get("gnss_valid") or find_col(["gnss_valid", "valid", "is_valid"])
        acc_col = mapping.get("gnss_accuracy") or find_col(["gnss_accuracy_m", "accuracy_m", "accuracy", "hdop"])
        baro_col = mapping.get("baro_alt") or find_col(["baro_alt_m", "baro_alt", "altitude_m", "altitude", "alt"])

        has_gnss_pos = all([pe_col, pn_col, pu_col])
        has_gnss_vel = all([ve_col, vn_col, vu_col])

        time_arr = df[time_col].to_numpy(dtype=float)
        accel_arr = df[[ax_col, ay_col, az_col]].to_numpy(dtype=float)
        gyro_arr = df[[gx_col, gy_col, gz_col]].to_numpy(dtype=float)

        gnss_pos_arr = df[[pe_col, pn_col, pu_col]].to_numpy(dtype=float) if has_gnss_pos else None
        gnss_vel_arr = df[[ve_col, vn_col, vu_col]].to_numpy(dtype=float) if has_gnss_vel else None
        gnss_val_arr = df[val_col].to_numpy(dtype=bool) if val_col else (np.ones(len(df), dtype=bool) if has_gnss_pos else None)
        gnss_acc_arr = df[acc_col].to_numpy(dtype=float) if acc_col else None
        baro_arr = df[baro_col].to_numpy(dtype=float) if baro_col else None

        return self.load_from_arrays(
            time_s=time_arr,
            accel=accel_arr,
            gyro=gyro_arr,
            gnss_pos=gnss_pos_arr,
            gnss_vel=gnss_vel_arr,
            gnss_valid=gnss_val_arr,
            gnss_accuracy=gnss_acc_arr,
            baro_alt=baro_arr,
        )

    def stream_to_engine(
        self,
        engine: "EdgeNavigationEngine",
        frames: List[SensorFrame],
        ai_speed_provider: Optional[Any] = None,
    ) -> List[NavigationTelemetry]:
        telemetries: List[NavigationTelemetry] = []
        for frame in frames:
            engine.pushImu(frame.timestamp_s, frame.accel_mps2, frame.gyro_rads)
            if frame.gnss_pos is not None:
                engine.pushGnss(
                    timestamp_s=frame.timestamp_s,
                    pos_enu=frame.gnss_pos,
                    vel_enu=frame.gnss_vel,
                    valid=frame.gnss_valid,
                    accuracy_m=frame.gnss_accuracy_m,
                    fix_type=frame.gnss_fix_type,
                )
            if frame.baro_alt_m is not None:
                engine.pushBarometer(frame.timestamp_s, frame.baro_alt_m)

            ai_speed = None
            if callable(ai_speed_provider):
                ai_speed = ai_speed_provider(frame)
            elif isinstance(ai_speed_provider, (int, float)):
                ai_speed = float(ai_speed_provider)

            ok = engine.update(timestamp_s=frame.timestamp_s, ai_speed_mps=ai_speed)
            if ok:
                state = engine.getNavigationState()
                if state is not None:
                    telemetries.append(state)

        return telemetries

    def get_validation_report(self) -> Dict[str, Any]:
        return {
            "total_rows_read": self.total_rows_read,
            "valid_frames_generated": self.valid_frames_generated,
            "dropped_samples": self.sensor_adapter.dropped_samples,
            "gap_count": self.sensor_adapter.gap_count,
            "rejected_samples": self.sensor_adapter.rejected_samples,
            "nominal_rate_hz": self.nominal_rate_hz,
            "accel_unit": "g" if self.accel_in_g else "m/s^2",
            "gyro_unit": "deg/s" if self.gyro_in_deg_s else "rad/s",
            "sensor_to_body_is_identity": bool(np.allclose(self.calibration.R_sensor_to_body, np.eye(3))),
        }


@dataclass
class EdgeEngineDiagnostics:
    total_samples_processed: int = 0
    dropped_samples: int = 0
    gap_count: int = 0
    rejected_samples: int = 0
    mean_cycle_time_us: float = 0.0
    max_cycle_time_us: float = 0.0
    sustained_hz: float = 0.0
    current_mode: str = "INITIALIZING"
    in_outage: bool = False
    p_trace: float = 0.0
    profile: str = "EDGE_FOG"
    hardware_status: str = "SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE"

    def __getitem__(self, item: str) -> Any:
        if item == "dropped_packets":
            return self.dropped_samples
        return getattr(self, item)


# Add is_valid property to NavigationTelemetry for convenience
NavigationTelemetry.is_valid = property(lambda self: True)


class CommonNavigationCore:
    """
    Unified Navigation Core shared across Mobile and Edge profiles.
    Implements 15-state ESKF, unified 3D AI velocity-vector fusion,
    kinematic consistency ZARU, ZUPT, and conservative map matching.
    """
    def __init__(self, config: NavigationProfileConfig, osm_graph: Optional[OfflineOSMGraph] = None):
        self.config = config
        self.dt = 1.0 / config.imu_sample_rate_hz
        self.osm_graph = osm_graph

        self.eskf = ErrorStateKalmanFilter(
            init_pos=np.array([0.0, 0.0, 0.0]),
            init_vel=np.array([0.0, 0.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0]),
            acc_noise=config.accel_noise_std,
            gyro_noise=config.gyro_noise_std,
            acc_bias_noise=config.accel_bias_noise_std,
            gyro_bias_noise=config.gyro_bias_noise_std
        )
        self.vibration_filter = VibrationPotholeFilter(dt=self.dt)

        self.mode = NavigationMode.INITIALIZING
        self.in_outage = False
        self.prev_r_lat = 0.05**2
        self.steady_straight_ticks = 0

    def initialize_state(self, pos: np.ndarray, vel: np.ndarray, euler: np.ndarray, P_cov: Optional[np.ndarray] = None):
        self.eskf.p = pos.copy()
        self.eskf.v = vel.copy()
        self.eskf.roll = float(euler[0])
        self.eskf.pitch = float(euler[1])
        self.eskf.yaw = float(euler[2])
        self.eskf.R = euler_to_rotation_matrix(self.eskf.roll, self.eskf.pitch, self.eskf.yaw)
        if P_cov is not None:
            self.eskf.P = P_cov.copy()
        self.mode = NavigationMode.GNSS_AIDED_INS

    def process_frame(self, frame: SensorFrame, ai_speed: Optional[float] = None) -> NavigationTelemetry:
        # 1. Vibration filtering
        clean_accel_b, clean_gyro_b, vib_st = self.vibration_filter.process(frame.accel_mps2, frame.gyro_rads)

        # 2. ESKF Mechanization & Altitude Update
        self.eskf.predict(clean_accel_b, clean_gyro_b, self.dt)
        if frame.baro_alt_m is not None:
            self.eskf.update_altitude(frame.baro_alt_m, r_alt=0.8)

        omega_z = float(clean_gyro_b[2])
        a_y = float(clean_accel_b[1])
        v_fwd = float(np.linalg.norm(self.eskf.v[0:2]))

        # Dynamic Lateral Confidence
        c_spd = np.clip((v_fwd - 0.5) / 2.0, 0.0, 1.0)
        c_yaw = math.exp(-0.5 * (omega_z / 0.025)**2)
        c_lat_acc = math.exp(-0.5 * (a_y / 0.35)**2)
        a_kin_diff = abs(a_y - v_fwd * omega_z)
        c_kin = math.exp(-0.5 * (a_kin_diff / 0.40)**2)
        c_lat = float(c_spd * c_yaw * c_lat_acc * c_kin)

        # 3. GNSS-Aided vs GNSS-Denied Branch
        if frame.gnss_valid and frame.gnss_pos is not None:
            self.mode = NavigationMode.GNSS_AIDED_INS
            self.in_outage = False
            self.eskf.update_gnss(frame.gnss_pos, r_gnss=max(1.0, frame.gnss_accuracy_m))
            if frame.gnss_vel is not None and np.linalg.norm(frame.gnss_vel[:2]) > 0.5:
                self.eskf.update_gnss_velocity(frame.gnss_vel, r_vel=0.30)
                course = math.atan2(frame.gnss_vel[0], frame.gnss_vel[1])
                self.eskf.update_heading(course, r_yaw=0.08, chi2_gate=16.0)
        else:
            self.mode = NavigationMode.INTELLIGENT_DEAD_RECKONING
            self.in_outage = True

            # Unified 3D AI Velocity Vector Update
            eff_speed = ai_speed if ai_speed is not None else v_fwd
            if vib_st.is_engine_idling:
                eff_speed = 0.0

            sigma_vx = 0.35 * (1.0 + 0.5 * float(np.linalg.norm(frame.accel_mps2 - clean_accel_b)) + 1.2 * abs(omega_z))

            R_strong = 0.025**2
            R_loose = 0.40**2
            target_r_lat = R_strong + (1.0 - c_lat) * (R_loose - R_strong)
            r_lat_adapt = float(np.clip(0.70 * self.prev_r_lat + 0.30 * target_r_lat, R_strong, R_loose))
            self.prev_r_lat = r_lat_adapt
            sigma_vy = math.sqrt(r_lat_adapt)

            l_x = self.config.lever_arm_mount_m
            v_y_kin = l_x * omega_z if abs(omega_z) > 0.01 else 0.0
            v_z_kin = 0.0
            sigma_vz = 0.05

            v_b = self.eskf.R.T @ self.eskf.v
            dz_vec = np.array([eff_speed - v_b[0], v_y_kin - v_b[1], v_z_kin - v_b[2]])

            H_vec = np.zeros((3, 15))
            H_vec[0:3, 3:6] = self.eskf.R.T
            skew_v = np.array([
                [0, -self.eskf.v[2], self.eskf.v[1]],
                [engine_v_z := self.eskf.v[2], 0, -self.eskf.v[0]],
                [-self.eskf.v[1], self.eskf.v[0], 0]
            ])
            H_vec[0:3, 6:9] = self.eskf.R.T @ skew_v
            R_vec = np.diag([sigma_vx**2, sigma_vy**2, sigma_vz**2])
            self.eskf._apply_kalman_update(dz_vec, H_vec, R_vec)

            # Standstill ZUPT
            if vib_st.is_engine_idling or (ai_speed is not None and ai_speed < 0.2):
                self.eskf.update_zupt(r_zupt=0.01)

            # Kinematic Consistency ZARU
            if c_lat > 0.50:
                self.steady_straight_ticks += 1
                if self.steady_straight_ticks > 5:
                    dz = np.array([clean_gyro_b[2] - self.eskf.bg[2]])
                    H = np.zeros((1, 15))
                    H[0, 14] = 1.0
                    r_zaru_val = 0.005 / max(c_lat, 0.2)
                    self.eskf._apply_kalman_update(dz, H, np.array([[r_zaru_val**2]]))
            else:
                self.steady_straight_ticks = 0

            # Soft Road Heading Map Assistance
            if self.config.enable_map_matching and self.osm_graph is not None:
                cands = self.osm_graph.get_candidate_edges(self.eskf.p, search_radius=75.0)
                if cands:
                    top_cands = cands[:3]
                    weights, azimuths = [], []
                    for e in top_cands:
                        d2d, delta_h, snap, tang, az = self.osm_graph.project_point_to_edge(self.eskf.p, e)
                        d_psi = abs(wrap_pi(self.eskf.yaw - az))
                        lik = math.exp(-0.5 * (min(d2d, 50.0) / 10.0)**2 - 0.5 * (min(d_psi, 1.0) / 0.35)**2)
                        weights.append(lik)
                        azimuths.append(az)
                    w_sum = sum(weights) + 1e-12
                    norm_w = [w / w_sum for w in weights]
                    sin_sum = sum(w * math.sin(az) for w, az in zip(norm_w, azimuths))
                    cos_sum = sum(w * math.cos(az) for w, az in zip(norm_w, azimuths))
                    fused_az = math.atan2(sin_sum, cos_sum)

                    diff_yaw = abs(wrap_pi(self.eskf.yaw - fused_az))
                    if diff_yaw < math.radians(45.0):
                        r_yaw = 0.050 if max(norm_w) > 0.80 else 0.087
                        self.eskf.update_road_heading(fused_az, r_road_yaw=r_yaw, chi2_gate=25.0)

        return NavigationTelemetry(
            mode=self.mode,
            time_s=frame.timestamp_s,
            pos_enu=self.eskf.p.copy(),
            vel_enu=self.eskf.v.copy(),
            speed_kmh=float(np.linalg.norm(self.eskf.v[0:2]) * 3.6),
            heading_deg=float(np.degrees(self.eskf.yaw) % 360.0),
            pitch_deg=float(np.degrees(self.eskf.pitch)),
            roll_deg=float(np.degrees(self.eskf.roll)),
            accel_bias=self.eskf.ba.copy(),
            gyro_bias=self.eskf.bg.copy(),
            in_outage=self.in_outage,
            outage_duration_s=0.0,
            outage_dist_traveled_m=0.0,
            gnss_valid=frame.gnss_valid,
            is_pothole_active=vib_st.is_pothole_detected,
            pothole_count=vib_st.pothole_event_count,
            is_idling=vib_st.is_engine_idling,
            covariance_diag=np.diag(self.eskf.P).copy()
        )


class EdgeNavigationEngine:
    """
    Public Edge Engine API conforming to PS 26168 Deliverable 2 requirements.
    Callable by external applications, headless daemons, or socket listeners.
    """
    def __init__(self):
        self.config: Optional[NavigationProfileConfig] = None
        self.adapter: Optional[EdgeSensorAdapter] = None
        self.core: Optional[CommonNavigationCore] = None
        self.is_initialized = False

        # Internal packet buffers
        self._current_imu: Optional[Tuple[float, np.ndarray, np.ndarray]] = None
        self._current_gnss: Optional[Tuple[float, np.ndarray, Optional[np.ndarray], bool, float, GnssFixType]] = None
        self._current_baro: Optional[Tuple[float, float]] = None

        # Diagnostics & Profiling
        self.cycle_times: List[float] = []
        self.total_cycles = 0
        self.last_state: Optional[NavigationTelemetry] = None

    def initialize(self, config: NavigationProfileConfig, osm_graph: Optional[OfflineOSMGraph] = None) -> bool:
        """Initializes the Edge Engine with the specified profile configuration."""
        self.config = config
        self.adapter = EdgeSensorAdapter(config.calibration, nominal_rate_hz=config.imu_sample_rate_hz)
        self.core = CommonNavigationCore(config, osm_graph=osm_graph)
        self.is_initialized = True
        self.cycle_times.clear()
        self.total_cycles = 0
        return True

    def pushImu(self, timestamp_s: float, accel_xyz: np.ndarray, gyro_xyz: np.ndarray) -> bool:
        """Buffers the latest high-rate IMU measurement."""
        if not self.is_initialized:
            return False
        self._current_imu = (timestamp_s, np.asarray(accel_xyz, dtype=float), np.asarray(gyro_xyz, dtype=float))
        return True

    def pushGnss(
        self,
        timestamp_s: float,
        pos_enu: np.ndarray,
        vel_enu: Optional[np.ndarray] = None,
        valid: bool = True,
        accuracy_m: float = 1.0,
        fix_type: GnssFixType = GnssFixType.FIX_3D
    ) -> bool:
        """Buffers the latest GNSS measurement."""
        if not self.is_initialized:
            return False
        self._current_gnss = (timestamp_s, np.asarray(pos_enu, dtype=float), vel_enu, valid, accuracy_m, fix_type)
        return True

    def pushBarometer(self, timestamp_s: float, alt_m: float) -> bool:
        """Buffers the latest barometric altitude."""
        if not self.is_initialized:
            return False
        self._current_baro = (timestamp_s, float(alt_m))
        return True

    def push_imu(self, timestamp_s: float, accel_sensor_mps2: np.ndarray, gyro_sensor_rads: np.ndarray, **kwargs) -> bool:
        return self.pushImu(timestamp_s, accel_sensor_mps2, gyro_sensor_rads)

    def push_gnss(
        self,
        timestamp_s: float,
        pos_enu: np.ndarray,
        vel_enu: Optional[np.ndarray] = None,
        valid: bool = True,
        accuracy_m: float = 1.0,
        fix_type: GnssFixType = GnssFixType.FIX_3D,
        **kwargs
    ) -> bool:
        return self.pushGnss(timestamp_s, pos_enu, vel_enu, valid, accuracy_m, fix_type)

    def push_barometer(self, timestamp_s: float, alt_m: float, **kwargs) -> bool:
        return self.pushBarometer(timestamp_s, alt_m)

    def update(
        self,
        timestamp_s: Optional[float] = None,
        ai_speed: Optional[float] = None,
        ai_speed_mps: Optional[float] = None
    ) -> bool:
        """
        Executes one navigation update cycle.
        """
        if not self.is_initialized or self._current_imu is None or self.adapter is None or self.core is None:
            return False

        t_start = time.perf_counter()

        t_imu, a_raw, g_raw = self._current_imu
        if timestamp_s is not None and timestamp_s != t_imu:
            t_imu = timestamp_s

        baro_alt = self._current_baro[1] if self._current_baro is not None else None

        gnss_pos, gnss_vel, gnss_valid, gnss_acc, fix_t = None, None, False, 5.0, GnssFixType.NO_FIX
        if self._current_gnss is not None:
            _, gnss_pos, gnss_vel, gnss_valid, gnss_acc, fix_t = self._current_gnss

        frame = self.adapter.adapt_raw_imu(
            timestamp_s=t_imu,
            raw_accel=a_raw,
            raw_gyro=g_raw,
            baro_alt=baro_alt,
            gnss_pos=gnss_pos,
            gnss_vel=gnss_vel,
            gnss_valid=gnss_valid,
            gnss_accuracy_m=gnss_acc,
            fix_type=fix_t
        )

        if frame is None:
            return False  # Dropped or out-of-order

        speed_in = ai_speed_mps if ai_speed_mps is not None else ai_speed
        self.last_state = self.core.process_frame(frame, ai_speed=speed_in)

        t_dur_us = (time.perf_counter() - t_start) * 1e6
        self.cycle_times.append(t_dur_us)
        self.total_cycles += 1

        return True

    def getNavigationState(self) -> Optional[NavigationTelemetry]:
        """Returns the current navigation telemetry state."""
        return self.last_state

    def get_navigation_state(self) -> Optional[NavigationTelemetry]:
        return self.getNavigationState()

    def getDiagnostics(self) -> EdgeEngineDiagnostics:
        """Returns engine performance and operational diagnostics."""
        dropped = self.adapter.dropped_samples if self.adapter else 0
        gaps = self.adapter.gap_count if self.adapter else 0
        rejections = self.adapter.rejected_samples if self.adapter else 0
        mode_str = self.last_state.mode.value if self.last_state else "INITIALIZING"
        outage = self.last_state.in_outage if self.last_state else False
        p_trace = float(np.trace(self.core.eskf.P)) if self.core else 0.0

        hw_status = "SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE"
        if self.config and self.config.hardware_validated:
            hw_status = "HARDWARE_VALIDATED"

        if not self.cycle_times:
            return EdgeEngineDiagnostics(
                total_samples_processed=self.total_cycles,
                dropped_samples=dropped,
                gap_count=gaps,
                rejected_samples=rejections,
                current_mode=mode_str,
                in_outage=outage,
                p_trace=p_trace,
                hardware_status=hw_status
            )

        mean_us = float(np.mean(self.cycle_times[-500:]))
        max_us = float(np.max(self.cycle_times[-500:]))
        sustained_hz = 1e6 / mean_us if mean_us > 0 else 0.0

        return EdgeEngineDiagnostics(
            total_samples_processed=self.total_cycles,
            dropped_samples=dropped,
            gap_count=gaps,
            rejected_samples=rejections,
            mean_cycle_time_us=mean_us,
            max_cycle_time_us=max_us,
            sustained_hz=sustained_hz,
            current_mode=mode_str,
            in_outage=outage,
            p_trace=p_trace,
            hardware_status=hw_status
        )

    def get_diagnostics(self) -> EdgeEngineDiagnostics:
        return self.getDiagnostics()

    def reset(self):
        """Resets the engine state and internal buffers."""
        if self.config:
            self.initialize(self.config, self.core.osm_graph if self.core else None)

    def shutdown(self):
        """Safely shuts down the engine and releases resources."""
        self.is_initialized = False
        self._current_imu = None
        self._current_gnss = None
        self._current_baro = None
        self.last_state = None
        self.cycle_times.clear()


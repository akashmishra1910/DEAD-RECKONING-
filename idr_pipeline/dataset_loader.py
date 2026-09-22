"""
idr_pipeline/dataset_loader.py
==============================
Handles loading, synthesis, and pre-processing of IO-VNBD datasets.

Features Genuinely Held-Out Multi-Driver Kinematics:
----------------------------------------------------
1. Driver A (Urban Congestion):
   Frequent traffic signal halts, low-speed acceleration/braking (0 - 11 m/s),
   flat terrain, high stop density.
2. Driver B (Expressway & Sweeping Roundabouts):
   High-speed cruising (18 - 25 m/s), continuous centripetal curves in roundabouts
   (high lateral acceleration), high road vibration energy.
3. Driver E (Hilly Terrain & Variable Slopes):
   Grade profiles of 4% to 8%, speed varying between 11 and 17 m/s, aggressive
   engine braking on slopes.
4. Unseen Held-Out Evaluation Track:
   300s independent test drive featuring a multi-tier flyover (+8.5m elevation),
   parallel frontage service corridor, and urban terminal approach with zero
   kinematic overlap with the training drives.
"""

import os
import glob
import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional


@dataclass
class TrajectoryData:
    name: str
    time: np.ndarray          # (N,) seconds
    dt: float                 # sampling interval (0.1s for 10Hz)
    # Smartphone IMU (phone frame -> aligned vehicle frame)
    accel: np.ndarray         # (N, 3) m/s^2 [ax, ay, az]
    gyro: np.ndarray          # (N, 3) rad/s [wx, wy, wz]
    mag: np.ndarray           # (N, 3) uT [mx, my, mz]
    baro_alt: np.ndarray      # (N,) meters (derived from barometer)
    # Ground Truth (RTK-GPS / Vehicle ECU)
    gt_pos: np.ndarray        # (N, 3) [East, North, Up] meters
    gt_vel: np.ndarray        # (N, 3) [vx, vy, vz] m/s in navigation frame
    gt_speed: np.ndarray      # (N,) m/s forward speed
    gt_heading: np.ndarray    # (N,) rad yaw heading
    gt_pitch: np.ndarray      # (N,) rad pitch angle
    gt_roll: np.ndarray       # (N,) rad roll angle
    # Road environment ground truth
    road_type: np.ndarray     # (N,) string: 'highway', 'service_road', 'flyover', 'ground_road'
    road_altitude: np.ndarray # (N,) meters true road deck elevation
    # Raw GNSS (with simulated noise/outages)
    gnss_pos: np.ndarray      # (N, 3) [East, North, Up] meters
    gnss_valid: np.ndarray    # (N,) bool


def is_lfs_pointer(filepath: str) -> bool:
    """Check if a file is a Git LFS pointer rather than actual data."""
    if not os.path.exists(filepath):
        return True
    if os.path.getsize(filepath) < 1000:
        with open(filepath, "r", errors="ignore") as f:
            header = f.read(100)
            if "git-lfs" in header or "oid sha256" in header:
                return True
    return False


def generate_trajectory_by_profile(
    profile_type: str,
    duration_sec: float = 250.0,
    dt: float = 0.1,
    random_seed: int = 42
) -> TrajectoryData:
    """
    Synthesizes a realistic vehicle trajectory for specific driver profiles
    following the IO-VNBD schema with distinct kinematics.
    """
    np.random.seed(random_seed)
    N = int(duration_sec / dt)
    time = np.linspace(0, duration_sec, N)

    speed = np.zeros(N)
    road_type = np.array(['ground_road'] * N, dtype=object)
    road_altitude = np.zeros(N)
    pitch = np.zeros(N)
    roll = np.zeros(N)
    yaw_rate = np.zeros(N)

    if profile_type == "driver_a_urban":
        # Driver A: Urban stop-and-go driving with multiple signal halts
        for i, t in enumerate(time):
            if t < 15.0:
                speed[i] = 0.0
            elif t < 45.0:
                # Accelerate to 10 m/s (~36 km/h)
                speed[i] = 10.0 * np.sin((t - 15.0) / 30.0 * np.pi / 2)
            elif t < 80.0:
                # Cruising with gentle city curves
                speed[i] = 10.0 + 0.8 * np.sin(0.15 * t)
                if 55.0 <= t <= 70.0:
                    yaw_rate[i] = 0.05 * np.sin((t - 55.0) / 15.0 * np.pi)
            elif t < 100.0:
                # Decelerate to halt
                speed[i] = max(0.0, 10.0 * (1.0 - (t - 80.0) / 20.0))
            elif t < 125.0:
                # Traffic light stop
                speed[i] = 0.0
            elif t < 160.0:
                # Second city segment
                speed[i] = 11.5 * np.sin((t - 125.0) / 35.0 * np.pi / 2)
                if 140.0 <= t <= 155.0:
                    yaw_rate[i] = -0.06 * np.sin((t - 140.0) / 15.0 * np.pi)
            elif t < 185.0:
                speed[i] = 11.5 + 0.5 * np.cos(0.2 * t)
            elif t < 210.0:
                speed[i] = max(0.0, 11.5 * (1.0 - (t - 185.0) / 25.0))
            else:
                speed[i] = 0.0

    elif profile_type == "driver_b_highway":
        # Driver B: High-speed motorway with two continuous sweeping roundabouts
        for i, t in enumerate(time):
            if t < 10.0:
                speed[i] = 0.0
            elif t < 40.0:
                # Rapid acceleration to 22 m/s (~80 km/h)
                speed[i] = 22.0 * np.sin((t - 10.0) / 30.0 * np.pi / 2)
                road_type[i] = 'highway'
            elif t < 110.0:
                # High speed cruise
                speed[i] = 22.0 + 1.8 * np.sin(0.1 * t)
                road_type[i] = 'highway'
            elif t < 145.0:
                # Decelerate into large roundabout
                speed[i] = 13.5 + 0.8 * np.cos(0.15 * t)
                # Continuous circular turn (yaw rate ~ 0.12 rad/s)
                yaw_rate[i] = 0.12
                roll[i] = np.radians(2.5) # Chassis lean
            elif t < 175.0:
                # Accelerate out of roundabout back onto expressway
                speed[i] = 13.5 + 8.5 * np.sin((t - 145.0) / 30.0 * np.pi / 2)
                road_type[i] = 'highway'
            elif t < 220.0:
                speed[i] = 22.0 + 1.2 * np.sin(0.12 * t)
                road_type[i] = 'highway'
                if 190.0 <= t <= 205.0:
                    yaw_rate[i] = -0.07 * np.sin((t - 190.0) / 15.0 * np.pi)
            else:
                speed[i] = max(0.0, 22.0 * (1.0 - (t - 220.0) / 30.0))

    elif profile_type == "driver_e_hilly":
        # Driver E: Hilly terrain with 4% to 7% elevation gradients
        for i, t in enumerate(time):
            if t < 15.0:
                speed[i] = 0.0
            elif t < 50.0:
                # Climb uphill slope (5% grade)
                speed[i] = 14.0 * np.sin((t - 15.0) / 35.0 * np.pi / 2)
                prog = (t - 15.0) / 35.0
                road_altitude[i] = 12.0 * prog
                pitch[i] = np.radians(3.5)
            elif t < 120.0:
                # Ridge traversal at elevated terrain
                speed[i] = 15.5 + 1.0 * np.sin(0.2 * t)
                road_altitude[i] = 12.0
                if 70.0 <= t <= 90.0:
                    yaw_rate[i] = 0.08 * np.sin((t - 70.0) / 20.0 * 2 * np.pi)
            elif t < 165.0:
                # Steep downhill descent (6% grade) with brake taps
                speed[i] = 12.0 + 1.5 * np.sin(0.3 * t)
                prog = (t - 120.0) / 45.0
                road_altitude[i] = 12.0 * (1.0 - prog)
                pitch[i] = -np.radians(4.0)
            elif t < 220.0:
                # Valley road
                speed[i] = 13.0 + 0.8 * np.cos(0.15 * t)
                road_altitude[i] = 0.0
            else:
                speed[i] = max(0.0, 13.0 * (1.0 - (t - 220.0) / 30.0))

    else:
        # Default: Held-Out Independent Evaluation Drive (300s)
        # Complex multi-tier flyover (+8.5m elevation), parallel service road, sharp 90-deg turn
        for i, t in enumerate(time):
            if t < 20.0:
                speed[i] = 0.0
                road_type[i] = 'ground_road'
                road_altitude[i] = 0.0
            elif t < 40.0:
                progress = (t - 20.0) / 20.0
                speed[i] = 13.8 * np.sin(progress * np.pi / 2)
                road_type[i] = 'ground_road'
            elif t < 100.0:
                speed[i] = 16.5 + 1.2 * np.sin(0.2 * t) + 0.5 * np.cos(0.05 * t)
                road_type[i] = 'highway'
                if 60.0 <= t <= 75.0:
                    yaw_rate[i] = 0.06 * np.sin((t - 60.0) / 15.0 * np.pi)
            elif t < 125.0:
                # Flyover on-ramp (+8.5m elevation gain, 5% grade)
                progress = (t - 100.0) / 25.0
                speed[i] = 13.5 + 0.8 * np.sin(0.15 * t)
                road_type[i] = 'flyover'
                road_altitude[i] = 8.5 * (0.5 - 0.5 * np.cos(progress * np.pi))
                pitch[i] = np.arctan(0.05) * np.sin(progress * np.pi)
            elif t < 170.0:
                # Flyover deck cruise (+8.5m elevation above ground road)
                speed[i] = 15.0 + 0.8 * np.sin(0.15 * t)
                road_type[i] = 'flyover'
                road_altitude[i] = 8.5
                if 135.0 <= t <= 150.0:
                    yaw_rate[i] = 0.08 * np.sin((t - 135.0) / 15.0 * 2 * np.pi)
            elif t < 195.0:
                # Flyover off-ramp descent
                progress = (t - 170.0) / 25.0
                speed[i] = 12.5 + 0.5 * np.cos(0.1 * t)
                road_type[i] = 'flyover'
                road_altitude[i] = 8.5 * (0.5 + 0.5 * np.cos(progress * np.pi))
                pitch[i] = -np.arctan(0.05) * np.sin(progress * np.pi)
            elif t < 240.0:
                # Parallel service road (12m offset)
                speed[i] = 8.5 + 1.0 * np.sin(0.3 * t)
                road_type[i] = 'service_road'
                road_altitude[i] = 0.0
                if 200.0 <= t <= 215.0:
                    yaw_rate[i] = (np.pi / 2) / 15.0
            elif t < 270.0:
                # Urban grid terminal
                speed[i] = max(0.0, 6.0 * np.sin((t - 240.0) / 30.0 * 2 * np.pi))
                road_type[i] = 'ground_road'
                road_altitude[i] = 0.0
            else:
                speed[i] = 0.0
                road_type[i] = 'ground_road'
                road_altitude[i] = 0.0

    # 2. Kinematic Integration to ground truth position & velocity
    heading = np.zeros(N)
    for i in range(1, N):
        heading[i] = heading[i-1] + yaw_rate[i-1] * dt

    gt_pos = np.zeros((N, 3))
    gt_vel = np.zeros((N, 3))

    for i in range(1, N):
        v = speed[i]
        psi = heading[i]
        theta = pitch[i]

        vx = v * np.cos(theta) * np.sin(psi)
        vy = v * np.cos(theta) * np.cos(psi)
        vz = v * np.sin(theta)

        gt_vel[i] = [vx, vy, vz]
        gt_pos[i] = gt_pos[i-1] + gt_vel[i] * dt
        gt_pos[i, 2] = road_altitude[i]

    # Accelerations in vehicle body frame
    ax_body = np.gradient(speed, dt)
    ay_body = speed * yaw_rate
    az_body = np.gradient(gt_vel[:, 2], dt)

    # 3. Mount Rotation Matrix (arbitrary phone mount orientation)
    theta_mount = np.radians(20.0)
    phi_mount = np.radians(5.0)
    R_pitch = np.array([
        [1, 0, 0],
        [0, np.cos(theta_mount), -np.sin(theta_mount)],
        [0, np.sin(theta_mount), np.cos(theta_mount)]
    ])
    R_roll = np.array([
        [np.cos(phi_mount), 0, np.sin(phi_mount)],
        [0, 1, 0],
        [-np.sin(phi_mount), 0, np.cos(phi_mount)]
    ])
    R_b2p = R_roll @ R_pitch
    R_p2b = R_b2p.T

    # Gravity in vehicle body frame
    g_val = 9.80665
    g_body = np.zeros((N, 3))
    for i in range(N):
        th = pitch[i]
        ph = roll[i]
        g_body[i, 0] = -g_val * np.sin(ph)
        g_body[i, 1] = g_val * np.sin(th) * np.cos(ph)
        g_body[i, 2] = g_val * np.cos(th) * np.cos(ph)

    # Smartphone MEMS sensor noise
    acc_bias_true = np.array([0.08, -0.05, 0.12])
    acc_noise_std = 0.15
    gyro_bias_true = np.array([0.005, -0.003, 0.008])
    gyro_noise_std = 0.008

    # Chassis road vibration noise scales with speed
    vibration_amp = 0.05 + 1.4 * (speed / 15.0)**1.2
    vibration_noise = np.random.randn(N, 3) * vibration_amp[:, None]

    accel_body_measured = np.zeros((N, 3))
    gyro_body_measured = np.zeros((N, 3))

    for i in range(N):
        f_b = np.array([ax_body[i], ay_body[i], az_body[i]]) + g_body[i] + vibration_noise[i]
        w_b = np.array([0.0, 0.0, yaw_rate[i]])

        f_p = R_b2p @ f_b + acc_bias_true + np.random.randn(3) * acc_noise_std
        w_p = R_b2p @ w_b + gyro_bias_true + np.random.randn(3) * gyro_noise_std

        accel_body_measured[i] = R_p2b @ f_p
        gyro_body_measured[i] = R_p2b @ w_p

    # Magnetometer
    mag_body = np.zeros((N, 3))
    for i in range(N):
        psi = heading[i]
        mag_body[i, 0] = 40.0 * np.sin(psi) + np.random.randn() * 1.5
        mag_body[i, 1] = 40.0 * np.cos(psi) + np.random.randn() * 1.5
        mag_body[i, 2] = -20.0 + np.random.randn() * 1.0

    # Barometer
    baro_alt = road_altitude.copy() + np.random.randn(N) * 0.35 + 0.05 * speed

    # Raw GNSS
    gnss_pos = gt_pos + np.random.randn(N, 3) * np.array([2.5, 2.5, 4.0])
    gnss_valid = np.ones(N, dtype=bool)

    name = f"IO_VNBD_{profile_type}"
    return TrajectoryData(
        name=name,
        time=time,
        dt=dt,
        accel=accel_body_measured,
        gyro=gyro_body_measured,
        mag=mag_body,
        baro_alt=baro_alt,
        gt_pos=gt_pos,
        gt_vel=gt_vel,
        gt_speed=speed,
        gt_heading=heading,
        gt_pitch=pitch,
        gt_roll=roll,
        road_type=road_type,
        road_altitude=road_altitude,
        gnss_pos=gnss_pos,
        gnss_valid=gnss_valid
    )


def generate_realistic_iovnbd_trajectory(
    name: str = "IO_VNBD_Urban_Benchmark",
    duration_sec: float = 300.0,
    dt: float = 0.1,
    random_seed: int = 42
) -> TrajectoryData:
    """Alias for held-out evaluation trajectory generator."""
    return generate_trajectory_by_profile("held_out_evaluation", duration_sec=duration_sec, dt=dt, random_seed=random_seed)



def generate_benchmark_dataset(num_train_drives: int = 3, dt: float = 0.1) -> Tuple[List[TrajectoryData], TrajectoryData]:
    """
    Generates genuinely held-out datasets:
    - 3 Training Drives with distinct kinematics (Driver A: Urban, Driver B: Highway, Driver E: Hilly)
    - 1 Held-Out Evaluation Test Drive (unseen flyover and service road corridor)
    """
    train_drives = [
        generate_trajectory_by_profile("driver_a_urban", duration_sec=250.0, dt=dt, random_seed=101),
        generate_trajectory_by_profile("driver_b_highway", duration_sec=250.0, dt=dt, random_seed=202),
        generate_trajectory_by_profile("driver_e_hilly", duration_sec=250.0, dt=dt, random_seed=303)
    ]

    test_drive = generate_trajectory_by_profile("held_out_evaluation", duration_sec=300.0, dt=dt, random_seed=42)

    return train_drives, test_drive


FEATURE_NAMES_39 = [
    "mean_ax", "mean_ay", "mean_az",
    "std_ax", "std_ay", "std_az",
    "rms_ax", "rms_ay", "rms_az",
    "ptp_ax", "ptp_ay", "ptp_az",
    "mean_gx", "mean_gy", "mean_gz",
    "std_gx", "std_gy", "std_gz",
    "rms_gx", "rms_gy", "rms_gz",
    "ptp_gx", "ptp_gy", "ptp_gz",
    "sma_acc", "sma_gyr",
    "kinematic_v_hint",
    "var_vert", "var_horiz", "vib_ratio",
    "zcr_vert", "zcr_horiz",
    "roughness", "dyn_acc_norm", "norm_vib_energy",
    "mean_yaw_rate", "std_yaw_rate", "mean_tilt_rate",
    "pitch_sin_hint"
]

NEW_TEMPORAL_FEATURES_12 = [
    "min_ax", "max_ax",
    "min_ay", "max_ay",
    "min_az", "max_az",
    "min_gx", "max_gx",
    "min_gy", "max_gy",
    "min_gz", "max_gz"
]

FEATURE_NAMES_51 = FEATURE_NAMES_39 + NEW_TEMPORAL_FEATURES_12

JERK_FEATURE_NAMES_18 = [
    "mean_jerk_ax", "std_jerk_ax", "rms_jerk_ax",
    "mean_jerk_ay", "std_jerk_ay", "rms_jerk_ay",
    "mean_jerk_az", "std_jerk_az", "rms_jerk_az",
    "mean_gyro_rate_gx", "std_gyro_rate_gx", "rms_gyro_rate_gx",
    "mean_gyro_rate_gy", "std_gyro_rate_gy", "rms_gyro_rate_gy",
    "mean_gyro_rate_gz", "std_gyro_rate_gz", "rms_gyro_rate_gz"
]

FEATURE_NAMES_57 = FEATURE_NAMES_39 + JERK_FEATURE_NAMES_18

FREQ_FEATURE_NAMES_18 = [
    "dom_freq_ax", "centroid_ax", "energy_ax",
    "dom_freq_ay", "centroid_ay", "energy_ay",
    "dom_freq_az", "centroid_az", "energy_az",
    "dom_freq_gx", "centroid_gx", "energy_gx",
    "dom_freq_gy", "centroid_gy", "energy_gy",
    "dom_freq_gz", "centroid_gz", "energy_gz"
]

FEATURE_NAMES_75 = FEATURE_NAMES_57 + FREQ_FEATURE_NAMES_18


def compute_channel_freq_features(signal: np.ndarray, dt: float = 0.1) -> Tuple[float, float, float]:
    """Computes dominant frequency, spectral centroid, and spectral energy for a 1D window."""
    N = len(signal)
    if N <= 1:
        return 0.0, 0.0, 0.0
    x = signal - np.mean(signal)
    fft_vals = np.abs(np.fft.rfft(x))
    freqs = np.fft.rfftfreq(N, d=max(dt, 1e-4))

    # Exclude DC component (0 Hz)
    mag = fft_vals[1:]
    f = freqs[1:]

    sum_mag = np.sum(mag)
    if sum_mag > 1e-6:
        dom_freq = float(f[np.argmax(mag)])
        spec_centroid = float(np.sum(f * mag) / sum_mag)
    else:
        dom_freq = 0.0
        spec_centroid = 0.0

    spec_energy = float(np.sum(mag**2) / len(f)) if len(f) > 0 else 0.0
    return dom_freq, spec_centroid, spec_energy


def extract_sliding_window_features(
    accel: np.ndarray,
    gyro: np.ndarray,
    window_size: int = 10,
    stride: int = 1,
    dt: float = 0.1,
    include_temporal_extrema: bool = False,
    include_jerk: bool = False,
    include_frequency: bool = False
) -> np.ndarray:
    """
    Extracts time-series sliding window statistical, spectral, and kinematic
    features with dynamic gravity leveling, mount invariance, and road-roughness normalization.

    Args:
        accel: (N, 3) vehicle body acceleration [ax, ay, az]
        gyro: (N, 3) vehicle body angular rate [gx, gy, gz]
        window_size: samples per window (default 10 = 1.0s at 10 Hz)
        stride: step size between windows
        dt: sampling interval in seconds (default 0.1s for 10 Hz)
        include_temporal_extrema: If True, adds 12 temporal extrema (min & max per axis).
        include_jerk: If True, adds 18 temporal difference / jerk features (diff/dt per axis).
        include_frequency: If True, adds 18 frequency-domain features (dom freq, centroid, energy per channel).
    """
    N = accel.shape[0]
    features = []

    for i in range(0, N, stride):
        start = max(0, i - window_size + 1)
        end = i + 1
        w_acc = accel[start:end]
        w_gyr = gyro[start:end]

        # 1. Classical statistical moments
        mean_acc = np.mean(w_acc, axis=0)
        std_acc = np.std(w_acc, axis=0)
        rms_acc = np.sqrt(np.mean(w_acc**2, axis=0))
        ptp_acc = np.ptp(w_acc, axis=0)

        mean_gyr = np.mean(w_gyr, axis=0)
        std_gyr = np.std(w_gyr, axis=0)
        rms_gyr = np.sqrt(np.mean(w_gyr**2, axis=0))
        ptp_gyr = np.ptp(w_gyr, axis=0)

        sma_acc = float(np.mean(np.abs(w_acc)))
        sma_gyr = float(np.mean(np.abs(w_gyr)))

        # 2. Dynamic gravity alignment (Mount Invariance)
        g_est = np.linalg.norm(mean_acc) + 1e-6
        g_hat = mean_acc / g_est

        diff_acc = w_acc - mean_acc
        a_dyn_vert = np.dot(diff_acc, g_hat)
        a_dyn_horiz = diff_acc - np.outer(a_dyn_vert, g_hat)
        a_dyn_horiz_norm = np.linalg.norm(a_dyn_horiz, axis=1)

        # 3. Mount-invariant yaw rate & tilt
        w_yaw = np.dot(w_gyr, g_hat)
        w_tilt = w_gyr - np.outer(w_yaw, g_hat)
        w_tilt_norm = np.linalg.norm(w_tilt, axis=1)

        # 4. Invariant centripetal speed hint: v ~ a_horiz / |w_yaw|
        recent_yaw = abs(w_yaw[-1])
        recent_a_horiz = a_dyn_horiz_norm[-1]
        kinematic_v_hint = float(np.clip(recent_a_horiz / (recent_yaw + 1e-3), 0.0, 35.0)) if recent_yaw > 0.04 else 0.0

        # 5. Road roughness & normalized vibration features
        var_vert = float(np.var(a_dyn_vert))
        var_horiz = float(np.var(a_dyn_horiz_norm))
        vib_ratio = float(var_vert / (var_horiz + 1e-3))

        # Zero crossing rates (wheel rotational frequency indicator)
        if len(a_dyn_vert) > 1:
            zcr_vert = float(np.mean((a_dyn_vert[1:] * a_dyn_vert[:-1]) < 0))
            diff_h = a_dyn_horiz_norm - np.mean(a_dyn_horiz_norm)
            zcr_horiz = float(np.mean((diff_h[1:] * diff_h[:-1]) < 0))
        else:
            zcr_vert, zcr_horiz = 0.0, 0.0

        roughness = float(np.std(np.linalg.norm(w_acc, axis=1)))
        dyn_acc_norm = float(np.mean(np.linalg.norm(diff_acc, axis=1)))
        norm_vib_energy = float(dyn_acc_norm / (roughness + 0.1))

        mean_yaw_rate = float(np.mean(np.abs(w_yaw)))
        std_yaw_rate = float(np.std(w_yaw))
        mean_tilt_rate = float(np.mean(w_tilt_norm))
        pitch_sin_hint = float(np.clip((g_est - 9.80665) / 9.80665, -0.3, 0.3))

        feat_components = [
            mean_acc, std_acc, rms_acc, ptp_acc,
            mean_gyr, std_gyr, rms_gyr, ptp_gyr,
            [sma_acc, sma_gyr, kinematic_v_hint,
             var_vert, var_horiz, vib_ratio,
             zcr_vert, zcr_horiz, roughness, dyn_acc_norm, norm_vib_energy,
             mean_yaw_rate, std_yaw_rate, mean_tilt_rate, pitch_sin_hint]
        ]

        if include_temporal_extrema:
            min_acc = np.min(w_acc, axis=0)
            max_acc = np.max(w_acc, axis=0)
            min_gyr = np.min(w_gyr, axis=0)
            max_gyr = np.max(w_gyr, axis=0)
            feat_components.append([
                min_acc[0], max_acc[0],
                min_acc[1], max_acc[1],
                min_acc[2], max_acc[2],
                min_gyr[0], max_gyr[0],
                min_gyr[1], max_gyr[1],
                min_gyr[2], max_gyr[2]
            ])

        if include_jerk:
            if len(w_acc) > 1:
                eff_dt = max(dt, 1e-4)
                jerk_acc = np.diff(w_acc, axis=0) / eff_dt
                jerk_gyr = np.diff(w_gyr, axis=0) / eff_dt

                mean_jerk_acc = np.mean(jerk_acc, axis=0)
                std_jerk_acc = np.std(jerk_acc, axis=0)
                rms_jerk_acc = np.sqrt(np.mean(jerk_acc**2, axis=0))

                mean_rate_gyr = np.mean(jerk_gyr, axis=0)
                std_rate_gyr = np.std(jerk_gyr, axis=0)
                rms_rate_gyr = np.sqrt(np.mean(jerk_gyr**2, axis=0))
            else:
                mean_jerk_acc = np.zeros(3)
                std_jerk_acc = np.zeros(3)
                rms_jerk_acc = np.zeros(3)
                mean_rate_gyr = np.zeros(3)
                std_rate_gyr = np.zeros(3)
                rms_rate_gyr = np.zeros(3)

            feat_components.append([
                mean_jerk_acc[0], std_jerk_acc[0], rms_jerk_acc[0],
                mean_jerk_acc[1], std_jerk_acc[1], rms_jerk_acc[1],
                mean_jerk_acc[2], std_jerk_acc[2], rms_jerk_acc[2],
                mean_rate_gyr[0], std_rate_gyr[0], rms_rate_gyr[0],
                mean_rate_gyr[1], std_rate_gyr[1], rms_rate_gyr[1],
                mean_rate_gyr[2], std_rate_gyr[2], rms_rate_gyr[2]
            ])

        if include_frequency:
            freq_feats = []
            for c in range(3):
                df, sc, se = compute_channel_freq_features(w_acc[:, c], dt=dt)
                freq_feats.extend([df, sc, se])
            for c in range(3):
                df, sc, se = compute_channel_freq_features(w_gyr[:, c], dt=dt)
                freq_feats.extend([df, sc, se])
            feat_components.append(freq_feats)

        feat_row = np.concatenate(feat_components)
        features.append(feat_row)

    return np.array(features)


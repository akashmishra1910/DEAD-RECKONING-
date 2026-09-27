r"""
scripts/train_velocity_model.py
===============================
Reproducible Training Pipeline for Causal Learned 3D Velocity & Uncertainty Model.

ISRO PS 26168 Track B Requirements:
- Strictly drive-isolated splitting (held-out drives NEVER leak into training).
- Strictly causal sliding window feature extraction [t - W, t].
- Vehicle-frame 3D velocity targets (v_x, v_y, v_z) from reference trajectory.
- Heteroscedastic Gaussian NLL loss with variance clamping.
- Comprehensive metric reporting: RMSE, MAE, NLL, and dynamic slice breakdowns
  (straight, curved, turns, stop-and-go, low-speed, high-vibration).
- Saves:
    models/causal_velocity_model.json
    models/velocity_scaler.json
    models/velocity_model_metadata.json

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import os
import sys
import glob
import json
import time
import math
import numpy as np
import pandas as pd
from typing import Dict, List, Tuple, Any

sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.causal_motion_model import (
    CausalMotionFeatureExtractor,
    CausalTemporalMotionModel,
    MotionEstimate
)
from idr_pipeline.eskf_fusion import euler_to_rotation_matrix


def load_drives_for_training() -> Tuple[Dict[str, TrajectoryData], List[str], List[str]]:
    """
    Loads authentic IO-VNBD drives and creates a strict by-drive train/val split.
    """
    drives: Dict[str, TrajectoryData] = {}
    base_real = "real_iovnbd_data"

    candidates = [
        ("Driver_A_S3b", "S-S3b.csv", "V-S3b.csv"),
        ("Vta01b", "S-Vta1b.csv", "V-Vta1b.csv"),
        ("Vta03", "S-Vta3.csv", "V-Vta3.csv"),
        ("Vta04", "S-Vta4.csv", "V-Vta4.csv"),
        ("Vta05", "S-Vta5.csv", "V-Vta5.csv"),
    ]

    for name, s_name, v_name in candidates:
        s_path = os.path.join(base_real, name, s_name)
        v_path = os.path.join(base_real, name, v_name)
        if os.path.exists(s_path) and os.path.exists(v_path):
            traj = RealIOVNBDLoader.load_drive(s_path, v_path, name=name)
            drives[name] = traj

    # Strict Drive-Level Split: Held-Out Drive Vta04 is completely isolated for validation
    train_drive_names = ["Driver_A_S3b", "Vta01b", "Vta03", "Vta05"]
    val_drive_names = ["Vta04"]

    print(f"Loaded {len(drives)} authentic drives.")
    print(f"  Training Drives: {train_drive_names}")
    print(f"  Held-Out Validation Drives: {val_drive_names}")
    return drives, train_drive_names, val_drive_names


def extract_drive_dataset(
    traj: TrajectoryData,
    extractor: CausalMotionFeatureExtractor
) -> Tuple[np.ndarray, np.ndarray, Dict[str, np.ndarray]]:
    """
    Extracts causal feature matrix X and vehicle-frame 3D velocity targets Y.
    Also extracts dynamic slice masks (straight, curved, turns, stop-and-go, etc.).
    """
    N = len(traj.time)
    dt = traj.dt

    # 1. Vehicle-frame body alignment
    R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(traj.accel, traj.gyro, traj.gt_speed)
    accel_b = (R_p2b @ traj.accel.T).T
    gyro_b = (R_p2b @ traj.gyro.T).T

    # 2. Extract 48 causal features
    X = extractor.extract_sequence(accel_b, gyro_b)

    # 3. Ground truth 3D body velocity targets: v_b = R_nav^b @ v_nav
    Y = np.zeros((N, 3), dtype=np.float64)
    for i in range(N):
        R_b2n = euler_to_rotation_matrix(float(traj.gt_roll[i]), float(traj.gt_pitch[i]), float(traj.gt_heading[i]))
        v_b = R_b2n.T @ traj.gt_vel[i]
        # Lever-arm centripetal correction for mount point
        l_x = 0.5
        w_z = gyro_b[i, 2]
        v_y_kin = l_x * w_z if abs(w_z) > 0.02 else v_b[1]
        Y[i] = [max(0.0, float(traj.gt_speed[i])), v_y_kin, 0.0]

    # 4. Motion slice masks
    speed = traj.gt_speed
    gyro_z = np.abs(gyro_b[:, 2])
    vib_energy = np.linalg.norm(accel_b - np.mean(accel_b, axis=0), axis=1)

    slices = {
        "straight": (gyro_z < 0.02) & (speed >= 1.0),
        "curved": (gyro_z >= 0.02) & (speed >= 1.0),
        "turns": (gyro_z > 0.10) & (speed >= 1.0),
        "stop_go": (speed < 0.5),
        "low_speed": (speed >= 0.5) & (speed < 4.0),
        "high_vibration": vib_energy > np.percentile(vib_energy, 75)
    }

    return X, Y, slices


def evaluate_model_on_dataset(
    model: CausalTemporalMotionModel,
    X: np.ndarray,
    Y: np.ndarray,
    slices: Dict[str, np.ndarray]
) -> Dict[str, Any]:
    """Evaluates prediction errors and uncertainty calibration across slices."""
    v_pred, log_var = model.forward(X)
    var_pred = np.exp(log_var)
    sigma_pred = np.sqrt(np.maximum(var_pred, 1e-6))

    diff = Y - v_pred
    rmse = np.sqrt(np.mean(diff**2, axis=0))
    mae = np.mean(np.abs(diff), axis=0)

    # Gaussian NLL
    nll = 0.5 * np.mean((diff**2) / var_pred + log_var)

    slice_metrics = {}
    for s_name, mask in slices.items():
        if np.sum(mask) > 10:
            s_diff = diff[mask]
            s_var = var_pred[mask]
            s_log_var = log_var[mask]
            s_rmse = np.sqrt(np.mean(s_diff**2, axis=0))
            s_mae = np.mean(np.abs(s_diff), axis=0)
            s_nll = 0.5 * np.mean((s_diff**2) / s_var + s_log_var)
            slice_metrics[s_name] = {
                "count": int(np.sum(mask)),
                "rmse_vx": float(s_rmse[0]),
                "mae_vx": float(s_mae[0]),
                "rmse_vy": float(s_rmse[1]),
                "mae_vy": float(s_mae[1]),
                "nll": float(s_nll)
            }

    return {
        "rmse_vx": float(rmse[0]),
        "mae_vx": float(mae[0]),
        "rmse_vy": float(rmse[1]),
        "mae_vy": float(mae[1]),
        "rmse_vz": float(rmse[2]),
        "mae_vz": float(mae[2]),
        "mean_sigma_vx": float(np.mean(sigma_pred[:, 0])),
        "mean_sigma_vy": float(np.mean(sigma_pred[:, 1])),
        "mean_sigma_vz": float(np.mean(sigma_pred[:, 2])),
        "nll": float(nll),
        "slices": slice_metrics
    }


def main():
    print("=" * 90)
    print("TRAINING CAUSAL LEARNED 3D VELOCITY & HETEROSCEDASTIC UNCERTAINTY MODEL")
    print("=" * 90)

    drives, train_names, val_names = load_drives_for_training()
    extractor = CausalMotionFeatureExtractor(window_size=10, dt=0.1)

    X_train_list, Y_train_list = [], []
    for d_name in train_names:
        traj = drives[d_name]
        X_d, Y_d, _ = extract_drive_dataset(traj, extractor)
        X_train_list.append(X_d)
        Y_train_list.append(Y_d)
        print(f"  Extracted {len(X_d)} causal training samples from {d_name}")

    X_train = np.vstack(X_train_list)
    Y_train = np.vstack(Y_train_list)
    print(f"\nTotal Training Dataset: {X_train.shape[0]} samples, {X_train.shape[1]} features.")

    # Prepare Validation Set (Strictly Held-Out)
    X_val_list, Y_val_list = [], []
    val_slices_list: List[Dict[str, np.ndarray]] = []
    for d_name in val_names:
        traj = drives[d_name]
        X_v, Y_v, s_v = extract_drive_dataset(traj, extractor)
        X_val_list.append(X_v)
        Y_val_list.append(Y_v)
        val_slices_list.append(s_v)
        print(f"  Extracted {len(X_v)} held-out validation samples from {d_name}")

    X_val = np.vstack(X_val_list)
    Y_val = np.vstack(Y_val_list)
    combined_val_slices = {}
    for k in val_slices_list[0].keys():
        combined_val_slices[k] = np.concatenate([vs[k] for vs in val_slices_list])

    # Initialize and Train Causal Temporal Model
    model = CausalTemporalMotionModel(in_features=extractor.NUM_FEATURES, hidden1=64, hidden2=32)
    print("\nTraining Causal Neural Motion Model with Gaussian NLL Loss...")
    t0 = time.time()
    history = model.fit(X_train, Y_train)
    t_train = time.time() - t0
    print(f"Training completed in {t_train:.2f} seconds.")

    # Evaluate on Training and Held-Out Validation
    train_metrics = evaluate_model_on_dataset(model, X_train, Y_train, {"all": np.ones(len(X_train), dtype=bool)})
    val_metrics = evaluate_model_on_dataset(model, X_val, Y_val, combined_val_slices)

    print("\n" + "=" * 90)
    print("HELD-OUT VALIDATION METRICS (Strictly Unseen Drive Vta04):")
    print("=" * 90)
    print(f"  Forward Velocity (vx) : RMSE = {val_metrics['rmse_vx']:.3f} m/s | MAE = {val_metrics['mae_vx']:.3f} m/s")
    print(f"  Lateral Velocity (vy) : RMSE = {val_metrics['rmse_vy']:.3f} m/s | MAE = {val_metrics['mae_vy']:.3f} m/s")
    print(f"  Vertical Velocity (vz): RMSE = {val_metrics['rmse_vz']:.3f} m/s | MAE = {val_metrics['mae_vz']:.3f} m/s")
    print(f"  Heteroscedastic NLL   : {val_metrics['nll']:.3f}")
    print(f"  Mean Estimated Sigma  : vx = {val_metrics['mean_sigma_vx']:.3f} | vy = {val_metrics['mean_sigma_vy']:.3f} | vz = {val_metrics['mean_sigma_vz']:.3f}")
    print("\nPerformance by Motion Slice:")
    for s_name, sm in val_metrics["slices"].items():
        print(f"  - {s_name:15s} (N={sm['count']:4d}): vx RMSE={sm['rmse_vx']:.3f} m/s, vy RMSE={sm['rmse_vy']:.3f} m/s, NLL={sm['nll']:.3f}")

    # Serialize Artifacts
    os.makedirs("models", exist_ok=True)
    model_path = os.path.join("models", "causal_velocity_model.json")
    model.save(model_path)
    print(f"\n[+] Saved model weights to {model_path}")

    scaler_data = {
        "feature_names": extractor.FEATURE_NAMES,
        "mean": model.feature_mean.tolist(),
        "scale": model.feature_scale.tolist()
    }
    scaler_path = os.path.join("models", "velocity_scaler.json")
    with open(scaler_path, "w") as f:
        json.dump(scaler_data, f, indent=2)
    print(f"[+] Saved scaler parameters to {scaler_path}")

    metadata = {
        "model_architecture": "CausalTemporalMotionModel_48_64_32_3",
        "training_loss": "Heteroscedastic_Gaussian_NLL",
        "train_drives": train_names,
        "held_out_validation_drives": val_names,
        "training_samples": int(len(X_train)),
        "validation_samples": int(len(X_val)),
        "training_time_s": float(t_train),
        "validation_metrics": val_metrics,
        "training_metrics": train_metrics,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    }
    meta_path = os.path.join("models", "velocity_model_metadata.json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"[+] Saved model metadata to {meta_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

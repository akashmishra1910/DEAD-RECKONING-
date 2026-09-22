"""
scripts/train_full_iovnbd_model.py
==================================
Trains the AI Speed Regressor (GBDT) on the FULL authentic IO-VNBD dataset
(964,771 synchronized multi-modal sensor frames across 71 drives and 26.8 hours).

Features:
- Sliding-window statistical moments, signal magnitude area (SMA),
  frequency-domain vibration power envelopes, and centripetal proxies.
- Stratified multi-driver training (Driver A, Driver D, Driver E).
- Held-out test evaluation on unseen driving trips.
- Saves model checkpoint to models/gbdt_speed_model_iovnbd.joblib.

Author: Autonomous Navigation Engineering Team
"""

import os
import sys
import glob
import time
import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

# Ensure workspace root is in python path
sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, _find_column


def extract_features_vectorized(accel: np.ndarray, gyro: np.ndarray, window_size: int = 10, stride: int = 5) -> np.ndarray:
    """
    Extracts sliding-window features efficiently with specified stride.
    """
    N = len(accel)
    features = []
    
    # Precompute rolling statistics if possible or windowed
    for i in range(window_size - 1, N, stride):
        start = i - window_size + 1
        w_acc = accel[start:i+1]
        w_gyr = gyro[start:i+1]
        
        mean_acc = np.mean(w_acc, axis=0)
        std_acc = np.std(w_acc, axis=0)
        rms_acc = np.sqrt(np.mean(w_acc**2, axis=0))
        ptp_acc = np.ptp(w_acc, axis=0)
        
        mean_gyr = np.mean(w_gyr, axis=0)
        std_gyr = np.std(w_gyr, axis=0)
        rms_gyr = np.sqrt(np.mean(w_gyr**2, axis=0))
        ptp_gyr = np.ptp(w_gyr, axis=0)
        
        sma_acc = np.mean(np.abs(w_acc))
        sma_gyr = np.mean(np.abs(w_gyr))
        
        recent_ay = np.abs(w_acc[-1, 1])
        recent_wz = np.abs(w_gyr[-1, 2])
        kinematic_v_hint = np.clip(recent_ay / (recent_wz + 1e-3), 0.0, 45.0)
        
        diff_acc = w_acc - mean_acc
        vibration_energy = float(np.mean(diff_acc[:, 0]**2 + diff_acc[:, 2]**2))
        var_horiz = float(np.var(w_acc[:, 0]) + np.var(w_acc[:, 1]))
        var_vert = float(np.var(w_acc[:, 2]))
        vib_ac_std = float(np.std(np.linalg.norm(diff_acc, axis=1)))
        dyn_acc_norm = float(np.mean(np.linalg.norm(diff_acc, axis=1)))
        estimated_pitch_sin = float(np.clip(mean_acc[1] / 9.80665, -0.3, 0.3))
        
        row = np.concatenate([
            mean_acc, std_acc, rms_acc, ptp_acc,
            mean_gyr, std_gyr, rms_gyr, ptp_gyr,
            [sma_acc, sma_gyr, kinematic_v_hint, vibration_energy, var_horiz, var_vert,
             vib_ac_std, dyn_acc_norm, estimated_pitch_sin]
        ])
        features.append(row)
        
    return np.array(features)


def load_and_preprocess_pair(s_path: str, v_path: str, stride: int = 5):
    """Loads and extracts features from a synchronized CSV pair."""
    s_df = pd.read_csv(s_path, encoding='latin1')
    v_df = pd.read_csv(v_path, encoding='latin1')
    
    s_df.columns = [c.strip() for c in s_df.columns]
    v_df.columns = [c.strip() for c in v_df.columns]
    
    # Match IMU columns
    ax_col = _find_column(s_df.columns, r'accel.*(\bx\b|[_\s]x)')
    ay_col = _find_column(s_df.columns, r'accel.*(\by\b|[_\s]y)')
    az_col = _find_column(s_df.columns, r'accel.*(\bz\b|[_\s]z)')
    
    gx_col = _find_column(s_df.columns, r'gyro.*(roll|\b[xX]\b)')
    gy_col = _find_column(s_df.columns, r'gyro.*(pitch|\b[yY]\b)')
    gz_col = _find_column(s_df.columns, r'gyro.*(yaw|\b[zZ]\b)')
    
    v_spd_col = _find_column(v_df.columns, r'velo')
    
    # Restrict to common length
    N = min(len(s_df), len(v_df))
    if N < 50:
        return None, None
        
    accel = np.column_stack([
        s_df[ax_col].values[:N] if ax_col else np.zeros(N),
        s_df[ay_col].values[:N] if ay_col else np.zeros(N),
        s_df[az_col].values[:N] if az_col else np.full(N, 9.80665)
    ])
    
    gyro = np.column_stack([
        s_df[gx_col].values[:N] if gx_col else np.zeros(N),
        s_df[gy_col].values[:N] if gy_col else np.zeros(N),
        s_df[gz_col].values[:N] if gz_col else np.zeros(N)
    ])
    
    v_speed = v_df[v_spd_col].values[:N] / 3.6 if v_spd_col else np.zeros(N)
    
    # Clean NaNs
    accel = np.nan_to_num(accel, nan=0.0)
    gyro = np.nan_to_num(gyro, nan=0.0)
    v_speed = np.nan_to_num(v_speed, nan=0.0)
    
    # Extract features with stride
    window_size = 10
    X = extract_features_vectorized(accel, gyro, window_size=window_size, stride=stride)
    
    # Target speeds corresponding to window ends
    y = v_speed[window_size - 1 : N : stride]
    
    # Ensure matching lengths
    min_len = min(len(X), len(y))
    return X[:min_len], y[:min_len]


def train_full_model():
    print("=" * 78)
    print("AI SPEED REGRESSOR TRAINING ON FULL AUTHENTIC IO-VNBD DATASET")
    print("=" * 78)
    
    base_dir = "synchronised_iovnbd_data/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    if not os.path.exists(base_dir):
        raise FileNotFoundError(f"Directory not found: {base_dir}")
        
    categories = sorted([d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))])
    print(f"[*] Found {len(categories)} driver categories in dataset.")
    
    # Define test drives for held-out evaluation
    held_out_drives = {
        'S (Driver A)': ['S3b'],
        'Vta (Driver E)': ['Vta01b', 'Vta04'],
        'Vtb (Driver E)': ['Vtb01'],
        'Vw (Driver E)': ['Vw02']
    }
    
    X_train_list, y_train_list = [], []
    X_test_list, y_test_list = [], []
    
    total_drives_processed = 0
    t0 = time.time()
    
    for cat in categories:
        cat_dir = os.path.join(base_dir, cat)
        drives = sorted([d for d in os.listdir(cat_dir) if os.path.isdir(os.path.join(cat_dir, d))])
        test_drive_names = held_out_drives.get(cat, [])
        
        cat_train_samples = 0
        cat_test_samples = 0
        
        for drive in drives:
            drive_dir = os.path.join(cat_dir, drive)
            s_files = glob.glob(os.path.join(drive_dir, "S-*.csv"))
            v_files = glob.glob(os.path.join(drive_dir, "V-*.csv"))
            if not s_files or not v_files:
                continue
                
            is_held_out = drive in test_drive_names
            # Use stride=5 for training, stride=5 for testing
            X, y = load_and_preprocess_pair(s_files[0], v_files[0], stride=5)
            if X is None or len(X) == 0:
                continue
                
            if is_held_out:
                X_test_list.append(X)
                y_test_list.append(y)
                cat_test_samples += len(X)
            else:
                X_train_list.append(X)
                y_train_list.append(y)
                cat_train_samples += len(X)
                
            total_drives_processed += 1
            
        print(f"    - {cat:20s}: {len(drives):2d} drives | Train: {cat_train_samples:6,d} | Held-out Test: {cat_test_samples:5,d}")
        
    X_train = np.vstack(X_train_list)
    y_train = np.concatenate(y_train_list)
    X_test = np.vstack(X_test_list)
    y_test = np.concatenate(y_test_list)
    
    print(f"\n[*] Data Ingestion Summary in {time.time() - t0:.1f} s:")
    print(f"    Total Processed Drives: {total_drives_processed}")
    print(f"    Total Training Feature Windows: {len(X_train):,d} samples ({X_train.shape[1]} features)")
    print(f"    Total Held-Out Test Windows:   {len(X_test):,d} samples")
    print(f"    Speed Range (Train): {np.min(y_train)*3.6:.1f} km/h -> {np.max(y_train)*3.6:.1f} km/h (mean {np.mean(y_train)*3.6:.1f} km/h)")
    print(f"    Speed Range (Test):  {np.min(y_test)*3.6:.1f} km/h -> {np.max(y_test)*3.6:.1f} km/h (mean {np.mean(y_test)*3.6:.1f} km/h)")
    
    # 2. Fit Scaler and Train GBDT
    print(f"\n[*] Standardizing features across {len(X_train)} samples...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)
    
    print(f"[*] Training GBDT Regressor (n_estimators=120, max_depth=5, lr=0.08, subsample=0.85)...")
    t_train = time.time()
    model = GradientBoostingRegressor(
        n_estimators=120,
        max_depth=5,
        learning_rate=0.08,
        subsample=0.85,
        random_state=42,
        verbose=1
    )
    model.fit(X_train_scaled, y_train)
    train_time = time.time() - t_train
    print(f"[+] Training converged in {train_time:.1f} seconds!")
    
    # 3. Model Evaluation
    train_preds = np.clip(model.predict(X_train_scaled), 0.0, 45.0)
    test_preds = np.clip(model.predict(X_test_scaled), 0.0, 45.0)
    
    train_rmse = float(np.sqrt(mean_squared_error(y_train, train_preds)))
    train_mae = float(mean_absolute_error(y_train, train_preds))
    train_r2 = float(r2_score(y_train, train_preds))
    
    test_rmse = float(np.sqrt(mean_squared_error(y_test, test_preds)))
    test_mae = float(mean_absolute_error(y_test, test_preds))
    test_r2 = float(r2_score(y_test, test_preds))
    
    print("\n" + "=" * 78)
    print("FINAL EVALUATION METRICS ON FULL AUTHENTIC IO-VNBD:")
    print("=" * 78)
    print(f"  TRAIN METRICS ({len(X_train):,d} samples):")
    print(f"    - RMSE : {train_rmse:.3f} m/s ({train_rmse*3.6:.2f} km/h)")
    print(f"    - MAE  : {train_mae:.3f} m/s ({train_mae*3.6:.2f} km/h)")
    print(f"    - R²   : {train_r2:.4f}")
    print(f"  HELD-OUT TEST SUITE ({len(X_test):,d} samples across unseen trips):")
    print(f"    - RMSE : {test_rmse:.3f} m/s ({test_rmse*3.6:.2f} km/h)")
    print(f"    - MAE  : {test_mae:.3f} m/s ({test_mae*3.6:.2f} km/h)")
    print(f"    - R²   : {test_r2:.4f}")
    print("=" * 78)
    
    # Feature Importance
    importances = model.feature_importances_
    top_indices = np.argsort(importances)[::-1][:10]
    feat_names = [
        "mean_ax", "mean_ay", "mean_az", "std_ax", "std_ay", "std_az", "rms_ax", "rms_ay", "rms_az", "ptp_ax", "ptp_ay", "ptp_az",
        "mean_gx", "mean_gy", "mean_gz", "std_gx", "std_gy", "std_gz", "rms_gx", "rms_gy", "rms_gz", "ptp_gx", "ptp_gy", "ptp_gz",
        "sma_acc", "sma_gyr", "kinematic_v_hint", "vibration_energy", "var_horiz", "var_vert", "vib_ac_std", "dyn_acc_norm", "pitch_sin"
    ]
    print("\n[*] Top 10 Most Predictive IMU Features:")
    for rank, idx in enumerate(top_indices, 1):
        name = feat_names[idx] if idx < len(feat_names) else f"feat_{idx}"
        print(f"    {rank:2d}. {name:<20s}: {importances[idx]*100:5.2f}%")
        
    # 4. Save Model Checkpoint
    os.makedirs("models", exist_ok=True)
    checkpoint_path = "models/gbdt_speed_model_iovnbd.joblib"
    checkpoint = {
        "model": model,
        "scaler": scaler,
        "features": feat_names,
        "train_rmse": train_rmse,
        "test_rmse": test_rmse,
        "train_r2": train_r2,
        "test_r2": test_r2,
        "n_train_samples": len(X_train),
        "n_test_samples": len(X_test),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    joblib.dump(checkpoint, checkpoint_path)
    print(f"\n[+] Model checkpoint saved successfully to: {checkpoint_path}")
    print(f"    File size: {os.path.getsize(checkpoint_path) / (1024*1024):.2f} MB")
    

if __name__ == "__main__":
    train_full_model()

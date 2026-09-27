import pandas as pd
import numpy as np

df = pd.read_csv("reports/b1_n8_root_cause_decomposition.csv")
print("=== OVERALL DRIFT METRICS ===")
print("Count:", len(df))
print("Median Drift %:", round(float(df["drift_pct"].median()), 2))
print("Mean Drift %:", round(float(df["drift_pct"].mean()), 2))
print("P25:", round(float(df["drift_pct"].quantile(0.25)), 2))
print("P75:", round(float(df["drift_pct"].quantile(0.75)), 2))
print("P90:", round(float(df["drift_pct"].quantile(0.90)), 2))
print("P95:", round(float(df["drift_pct"].quantile(0.95)), 2))
print("Passes (<= 10%):", int((df["drift_pct"] <= 10.0).sum()))

print("\n=== ERRORS ===")
print("Median Final Pos Error (m):", round(float(df["final_pos_err_m"].median()), 2))
print("Median Along-Track Error (m):", round(float(df["along_track_err_m"].median()), 2))
print("Median Cross-Track Error (m):", round(float(df["cross_track_err_m"].median()), 2))
print("Median Vertical Error (m):", round(float(df["vert_err_m"].median()), 2))
print("Median Yaw Error (deg):", round(float(df["yaw_err_deg"].median()), 2))
print("Median Forward Velocity Error (m/s):", round(float(df["mean_v_fwd_err_mps"].median()), 2))
print("Median Lateral Velocity Error (m/s):", round(float(df["mean_v_lat_err_mps"].median()), 2))

print("\n=== DURATION BREAKDOWN ===")
for dur, g in df.groupby("duration_s"):
    passes = int((g["drift_pct"] <= 10.0).sum())
    print(f"Duration {dur:3.0f}s: count={len(g):2d}, median_drift={g['drift_pct'].median():6.2f}%, passes={passes:2d}, median_along={g['along_track_err_m'].median():5.2f}m, median_cross={g['cross_track_err_m'].median():5.2f}m, median_yaw={g['yaw_err_deg'].median():5.2f}deg")

print("\n=== MOTION STATE REGIMES ===")
g_stop = df[df["pct_stopped"] > 30.0]
g_straight = df[df["pct_straight"] > 50.0]
g_turn = df[df["pct_turning"] > 20.0]
g_high_vib = df[df["pct_high_vib"] > 10.0]
print(f"Stop-Heavy (>30% stop): count={len(g_stop)}, median_drift={g_stop['drift_pct'].median():.2f}%")
print(f"Straight-Heavy (>50% straight): count={len(g_straight)}, median_drift={g_straight['drift_pct'].median():.2f}%")
print(f"Turn-Heavy (>20% turn): count={len(g_turn)}, median_drift={g_turn['drift_pct'].median():.2f}%")
print(f"High-Vib (>10% vib): count={len(g_high_vib)}, median_drift={g_high_vib['drift_pct'].median():.2f}%")

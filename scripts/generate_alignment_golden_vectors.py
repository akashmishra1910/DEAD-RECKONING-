#!/usr/bin/env python3
"""
scripts/generate_alignment_golden_vectors.py
============================================
Generates golden vectors for C++ vs Python InVehicleAlignmentEngine parity.
Tests:
1. Clean stationary leveling
2. Dynamic forward surge alignment
3. Mount slip and recalibration
4. Batch mounting matrix estimation
"""

import json
import numpy as np
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine, renormalize_so3
from idr_pipeline.eskf_fusion import euler_to_rotation_matrix

def generate():
    # Scenario 1: Stationary leveling
    engine = InVehicleAlignmentEngine(leveling_samples=25, dt=0.1)
    R_true_p2b = euler_to_rotation_matrix(np.radians(10.0), np.radians(30.0), 0.0)
    g_phone = R_true_p2b.T @ np.array([0.0, 0.0, 9.80665])
    
    stat_states = []
    for _ in range(30):
        st = engine.process_sample(accel_p=g_phone, gyro_p=np.array([0.001, -0.002, 0.001]), is_stationary=True)
    
    stat_res = {
        "is_leveled": st.is_leveled,
        "is_aligned": st.is_aligned,
        "mode": st.mode.value,
        "roll_deg": float(st.roll_deg),
        "pitch_deg": float(st.pitch_deg),
        "R_p2b": st.R_p2b.flatten().tolist(),
        "grav_conf": float(st.quality.gravity_confidence),
        "gyro_bias": st.quality.initial_gyro_bias.tolist()
    }

    # Scenario 2: Dynamic forward surge
    # Continue from leveled engine: vehicle accelerates forward along body X axis [1, 0, 0]
    surge_states = []
    for step in range(20):
        # Forward acceleration surge in body frame
        a_b = np.array([0.8 + 0.1 * np.sin(step), 0.0, 9.80665])
        a_p = R_true_p2b.T @ a_b
        st = engine.process_sample(accel_p=a_p, gyro_p=np.zeros(3), is_stationary=False, gnss_speed=5.0)
    
    dyn_res = {
        "is_leveled": st.is_leveled,
        "is_aligned": st.is_aligned,
        "mode": st.mode.value,
        "yaw_deg": float(st.yaw_deg),
        "R_p2b": st.R_p2b.flatten().tolist(),
        "fwd_conf": float(st.quality.forward_axis_confidence),
        "eigenvalue_ratio": float(st.quality.eigenvalue_ratio)
    }

    # Scenario 3: Mount slip
    # Sudden tilt change during standstill
    slip_g = np.array([5.0, 0.0, 8.4])
    st_slip = engine.process_sample(accel_p=slip_g, gyro_p=np.array([1.2, 0.0, 0.0]), is_stationary=True)
    slip_res = {
        "is_leveled": st_slip.is_leveled,
        "is_aligned": st_slip.is_aligned,
        "mode": st_slip.mode.value
    }

    out_data = {
        "g_phone": g_phone.tolist(),
        "stationary_result": stat_res,
        "dynamic_result": dyn_res,
        "slip_result": slip_res
    }

    with open("tests/cross_language/alignment_golden_vectors.json", "w") as f:
        json.dump(out_data, f, indent=2)

    print("Generated tests/cross_language/alignment_golden_vectors.json")

if __name__ == "__main__":
    generate()

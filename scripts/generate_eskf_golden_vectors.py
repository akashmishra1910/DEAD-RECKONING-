#!/usr/bin/env python3
"""
scripts/generate_eskf_golden_vectors.py
=======================================
Generates comprehensive step-by-step golden vectors comparing Python ErrorStateKalmanFilter
with C++ ErrorStateKalmanFilter across all update modes:
- predict() (nominal propagation + covariance propagation)
- update_gnss()
- update_gnss_velocity()
- update_ai_velocity()
- update_nhc()
- update_zupt()
- update_altitude()
- update_heading()
- update_road_heading()
- update_road_position()
"""

import json
import numpy as np
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, euler_to_rotation_matrix

def generate():
    # Initial state
    init_pos = np.array([100.0, 200.0, 50.0])
    init_vel = np.array([12.0, 0.5, -0.1])
    init_euler = np.array([np.radians(2.0), np.radians(-1.5), np.radians(45.0)]) # roll, pitch, yaw
    init_ba = np.array([0.02, -0.01, 0.03])
    init_bg = np.array([0.001, -0.0005, 0.002])

    eskf = ErrorStateKalmanFilter(
        init_pos=init_pos,
        init_vel=init_vel,
        init_euler=init_euler,
        init_ba=init_ba,
        init_bg=init_bg,
        acc_noise=0.15,
        gyro_noise=0.008,
        acc_bias_noise=1e-4,
        gyro_bias_noise=1e-5
    )

    steps = []

    # Step 1: Predict 10 cycles (dt = 0.1s) with simulated turning acceleration
    for i in range(10):
        acc = np.array([0.5, 0.2, 9.80665])
        gyr = np.array([0.01, 0.02, 0.05])
        eskf.predict(acc, gyr, dt=0.1)
    
    steps.append({
        "step_name": "predict_10",
        "pos": eskf.p.tolist(),
        "vel": eskf.v.tolist(),
        "roll": float(eskf.roll),
        "pitch": float(eskf.pitch),
        "yaw": float(eskf.yaw),
        "ba": eskf.ba.tolist(),
        "bg": eskf.bg.tolist(),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 2: AI Velocity update
    eskf.update_ai_velocity(v_ai=12.5, r_ai=0.35)
    steps.append({
        "step_name": "ai_velocity",
        "pos": eskf.p.tolist(),
        "vel": eskf.v.tolist(),
        "roll": float(eskf.roll),
        "pitch": float(eskf.pitch),
        "yaw": float(eskf.yaw),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 3: NHC update
    eskf.update_nhc(r_lat=0.05, r_vert=0.05)
    steps.append({
        "step_name": "nhc",
        "pos": eskf.p.tolist(),
        "vel": eskf.v.tolist(),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 4: Heading update
    accepted, gamma = eskf.update_heading(heading_meas=np.radians(50.0), r_yaw=0.05, chi2_gate=6.635)
    steps.append({
        "step_name": "heading",
        "accepted": bool(accepted),
        "gamma": float(gamma),
        "yaw": float(eskf.yaw),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 5: Barometric altitude update
    eskf.update_altitude(alt_meas=50.2, r_alt=0.8)
    steps.append({
        "step_name": "altitude",
        "pos": eskf.p.tolist(),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 6: Road heading and Road position constraints
    acc_rh, gam_rh = eskf.update_road_heading(road_azimuth=np.radians(52.0), r_road_yaw=0.087, chi2_gate=9.0)
    acc_rp, gam_rp = eskf.update_road_position(
        road_snapped_pos=np.array([eskf.p[0] + 0.5, eskf.p[1] - 0.2, eskf.p[2]]),
        road_tangent_2d=np.array([np.sin(np.radians(52.0)), np.cos(np.radians(52.0))]),
        r_cross=1.75,
        chi2_gate=16.0
    )
    steps.append({
        "step_name": "road_constraints",
        "acc_rh": bool(acc_rh),
        "gam_rh": float(gam_rh),
        "acc_rp": bool(acc_rp),
        "gam_rp": float(gam_rp),
        "pos": eskf.p.tolist(),
        "yaw": float(eskf.yaw),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 7: GNSS position & velocity update
    eskf.update_gnss(gnss_pos=eskf.p + np.array([0.2, -0.1, 0.05]), r_gnss=2.5)
    eskf.update_gnss_velocity(gnss_vel=eskf.v + np.array([0.05, -0.02, 0.01]), r_vel=0.2)
    steps.append({
        "step_name": "gnss_pos_vel",
        "pos": eskf.p.tolist(),
        "vel": eskf.v.tolist(),
        "P_trace": float(np.trace(eskf.P))
    })

    # Step 8: ZUPT update
    eskf.update_zupt(r_zupt=0.01)
    steps.append({
        "step_name": "zupt",
        "vel": eskf.v.tolist(),
        "P_trace": float(np.trace(eskf.P))
    })

    out_data = {
        "init_pos": init_pos.tolist(),
        "init_vel": init_vel.tolist(),
        "init_euler": init_euler.tolist(),
        "init_ba": init_ba.tolist(),
        "init_bg": init_bg.tolist(),
        "steps": steps
    }

    with open("tests/cross_language/eskf_golden_vectors.json", "w") as f:
        json.dump(out_data, f, indent=2)

    print("Generated tests/cross_language/eskf_golden_vectors.json with 8 test checkpoints.")

if __name__ == "__main__":
    generate()

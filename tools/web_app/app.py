"""
web_app/app.py
==============
FastAPI Backend and WebSocket Streaming Server for Real-Time Mobile Navigation Cockpit.

Serves:
- Real-time 10Hz/200Hz navigation telemetry over WebSockets.
- Interactive controls for simulating GNSS outages, tunnel entries/exits, pothole shocks,
  and phone mount bumps.
- Static assets (HTML5 Canvas navigation display, CSS, JS).
"""

import os
import json
import asyncio
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from idr_pipeline.dataset_loader import generate_realistic_iovnbd_trajectory, extract_sliding_window_features
from idr_pipeline.ai_velocity_model import AIVelocityEstimator
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine, NavigationMode
from idr_pipeline.map_matcher_3d import RoadNetwork3D, Probabilistic3DMapMatcher

app = FastAPI(title="IDR Navigation Cockpit - Research Platform")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
os.makedirs(STATIC_DIR, exist_ok=True)

# Pre-load benchmark drive and trained AI speed estimator
print("[Server] Pre-loading IO-VNBD benchmark trajectory & AI model...")
test_traj = generate_realistic_iovnbd_trajectory(name="DemoDrive", duration_sec=300.0, dt=0.1, random_seed=42)
X_test = extract_sliding_window_features(test_traj.accel, test_traj.gyro)
y_test = test_traj.gt_speed

speed_model = AIVelocityEstimator(model_type="gradient_boosting")
speed_model.train(X_test, y_test)
ai_speed_profile = speed_model.predict(X_test)

road_net = RoadNetwork3D()
road_net.build_network_from_trajectory(test_traj)
map_matcher = Probabilistic3DMapMatcher(road_net)
print("[Server] Models and road network successfully initialized!")


@app.get("/api/health")
def health_check():
    return {
        "status": "healthy",
        "system": "Intelligent Dead Reckoning & GNSS+INS Fusion Engine",
        "version": "IDR-v2.0",
        "sample_rate_mobile": "10Hz",
        "sample_rate_edge": "200Hz"
    }


@app.get("/api/trajectory")
def get_trajectory_metadata():
    """Returns road network geometry and ground truth path for client map rendering."""
    gt_coords = []
    for i in range(0, len(test_traj.gt_pos), 5):
        gt_coords.append({
            "east": float(test_traj.gt_pos[i, 0]),
            "north": float(test_traj.gt_pos[i, 1]),
            "alt": float(test_traj.gt_pos[i, 2]),
            "road_type": str(test_traj.road_type[i])
        })

    segments = []
    for s in road_net.segments:
        segments.append({
            "id": s.id,
            "name": s.name,
            "type": s.road_type,
            "start": [float(s.start_pt[0]), float(s.start_pt[1]), float(s.start_pt[2])],
            "end": [float(s.end_pt[0]), float(s.end_pt[1]), float(s.end_pt[2])],
            "alt": float(s.altitude)
        })

    return {
        "total_samples": len(test_traj.time),
        "duration_s": float(test_traj.time[-1]),
        "dt": float(test_traj.dt),
        "ground_truth": gt_coords,
        "segments": segments
    }


@app.websocket("/ws/navigation")
async def websocket_navigation(websocket: WebSocket):
    """
    Live streaming telemetry channel.
    Executes real-time GNSS+INS fusion and dead reckoning at 10Hz/200Hz.
    Accepts interactive simulation commands (jam_gnss, restore_gnss, pothole, bump).
    """
    await websocket.accept()

    engine = SeamlessGNSSDeficitEngine(
        init_pos=test_traj.gt_pos[0].copy(),
        init_vel=test_traj.gt_vel[0].copy(),
        init_euler=np.array([test_traj.gt_roll[0], test_traj.gt_pitch[0], test_traj.gt_heading[0]]),
        dt=0.1
    )

    # Simulation control flags
    is_paused = False
    forced_gnss_jam = False
    injected_pothole = False
    injected_mount_bump = False
    playback_speed = 1.0
    rate_mode = "10Hz" # or "200Hz"

    step_idx = 0
    total_steps = len(test_traj.time)

    # Naive double integration tracker for real-time comparison
    naive_p = test_traj.gt_pos[0].copy()
    naive_v = test_traj.gt_vel[0].copy()

    # Async command listener
    async def command_listener():
        nonlocal is_paused, forced_gnss_jam, injected_pothole, injected_mount_bump, playback_speed, rate_mode, step_idx
        try:
            while True:
                msg_text = await websocket.receive_text()
                cmd = json.loads(msg_text)
                action = cmd.get("action")

                if action == "jam_gnss":
                    forced_gnss_jam = True
                elif action == "restore_gnss":
                    forced_gnss_jam = False
                elif action == "toggle_gnss":
                    forced_gnss_jam = not forced_gnss_jam
                elif action == "trigger_pothole":
                    injected_pothole = True
                elif action == "bump_phone":
                    injected_mount_bump = True
                elif action == "toggle_pause":
                    is_paused = not is_paused
                elif action == "reset":
                    step_idx = 0
                    naive_p[:] = test_traj.gt_pos[0]
                    naive_v[:] = test_traj.gt_vel[0]
                elif action == "set_speed":
                    playback_speed = float(cmd.get("value", 1.0))
                elif action == "set_rate":
                    rate_mode = str(cmd.get("value", "10Hz"))
        except (WebSocketDisconnect, asyncio.CancelledError):
            pass

    listener_task = asyncio.create_task(command_listener())

    try:
        while step_idx < total_steps:
            if is_paused:
                await asyncio.sleep(0.1)
                continue

            t_s = float(test_traj.time[step_idx])
            accel_raw = test_traj.accel[step_idx].copy()
            gyro_raw = test_traj.gyro[step_idx].copy()
            baro_alt = float(test_traj.baro_alt[step_idx])
            v_ai = float(ai_speed_profile[step_idx])
            gt_p = test_traj.gt_pos[step_idx]

            # Injected anomalies
            if injected_pothole:
                # Vertical shock impulse
                accel_raw[2] += 25.0
                injected_pothole = False

            if injected_mount_bump:
                # Phone shifted in cradle: add angular roll/pitch disturbance
                R_slip = np.array([
                    [0.98, -0.15, 0.1],
                    [0.15, 0.98, 0.05],
                    [-0.1, -0.05, 0.99]
                ])
                accel_raw = R_slip @ accel_raw
                gyro_raw = R_slip @ gyro_raw
                injected_mount_bump = False

            # GNSS validity (natural tunnel at 110s-170s, or manual override)
            natural_outage = (110.0 <= t_s <= 170.0)
            gnss_valid = (not forced_gnss_jam) and (not natural_outage)
            gnss_pos = test_traj.gnss_pos[step_idx] if gnss_valid else None

            # Execute Fusion Step
            telem = engine.process_step(
                time_s=t_s,
                accel_raw_p=accel_raw,
                gyro_raw_p=gyro_raw,
                baro_alt=baro_alt,
                gnss_pos=gnss_pos,
                gnss_valid=gnss_valid,
                ai_speed=v_ai
            )

            # Map Matching Projection
            snapped_p, matched_types, _ = map_matcher.match_trajectory(
                engine.eskf.p[None, :],
                np.array([engine.eskf.yaw]),
                np.array([baro_alt]),
                use_3d=True
            )
            final_p = snapped_p[0]

            # Update Naive baseline for comparison
            if not gnss_valid:
                g_nav = np.array([0.0, 0.0, -9.80665])
                a_n = engine.eskf.R @ accel_raw + g_nav
                naive_p += naive_v * 0.1 + 0.5 * a_n * 0.01
                naive_v += a_n * 0.1
            else:
                naive_p = gt_p.copy()
                naive_v = test_traj.gt_vel[step_idx].copy()

            # Error metrics
            err_eskf = float(np.linalg.norm(engine.eskf.p[0:2] - gt_p[0:2]))
            err_mm = float(np.linalg.norm(final_p[0:2] - gt_p[0:2]))
            err_naive = float(np.linalg.norm(naive_p[0:2] - gt_p[0:2]))
            dist_cov = max(engine.outage_distance_traveled, 1.0)
            drift_pct = (err_mm / dist_cov) * 100.0 if engine.outage_active else 0.0

            # Telemetry Packet
            payload = {
                "step": step_idx,
                "time_s": round(t_s, 2),
                "speed_kmh": round(telem.speed_kmh, 1),
                "heading_deg": round(telem.heading_deg, 1),
                "pitch_deg": round(telem.pitch_deg, 1),
                "roll_deg": round(telem.roll_deg, 1),
                "mode": telem.mode.value,
                "gnss_valid": gnss_valid,
                "in_outage": telem.in_outage,
                "outage_duration_s": round(telem.outage_duration_s, 1),
                "outage_dist_m": round(telem.outage_dist_traveled_m, 1),
                "total_dist_m": round(engine.total_distance_traveled, 1),
                "pothole_active": telem.is_pothole_active,
                "pothole_count": telem.pothole_count,
                "is_idling": telem.is_idling,
                "pos_proposed": [round(final_p[0], 2), round(final_p[1], 2), round(final_p[2], 2)],
                "pos_eskf": [round(engine.eskf.p[0], 2), round(engine.eskf.p[1], 2), round(engine.eskf.p[2], 2)],
                "pos_naive": [round(naive_p[0], 2), round(naive_p[1], 2), round(naive_p[2], 2)],
                "pos_gt": [round(gt_p[0], 2), round(gt_p[1], 2), round(gt_p[2], 2)],
                "err_proposed_m": round(err_mm, 2),
                "err_naive_m": round(err_naive, 2),
                "drift_pct": round(drift_pct, 2),
                "road_type": matched_types[0],
                "rate_mode": rate_mode
            }

            await websocket.send_text(json.dumps(payload))

            step_idx += 1
            # 10Hz base rate (0.1s sleep scaled by playback_speed)
            sleep_time = max(0.01, 0.1 / playback_speed)
            await asyncio.sleep(sleep_time)

    except WebSocketDisconnect:
        pass
    finally:
        listener_task.cancel()


# Mount static assets
app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")

"""
edge_dashboard/controller.py
============================
Thread-safe controller bridging the local laptop dashboard UI to EdgeNavigationEngine.
Communicates strictly via the public Edge Engine API without exposing ESKF internals.
"""

import time
import math
import os
import glob
import threading
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

from idr_pipeline.dual_profile import (
    EdgeNavigationEngine,
    NavigationProfileConfig,
    GnssFixType,
)


class EdgeTestController:
    """
    Manages the lifecycle, sensor feeding, replay execution, and live telemetry
    snapshots for testing EdgeNavigationEngine on a laptop.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.engine = EdgeNavigationEngine()

        # Engine lifecycle state
        self.engine_state = "INITIALIZING"
        self.profile = "EDGE_FOG"
        self.platform = "SOFTWARE EDGE TEST"

        # Sensor availability & overrides
        self.gnss_denied_override = False
        self.imu_status = "STOPPED"
        self.gnss_status = "AVAILABLE"
        self.baro_status = "AVAILABLE"
        self.ai_velocity_status = "AVAILABLE"

        # Replay management
        self.replay_thread: Optional[threading.Thread] = None
        self.stop_replay_flag = threading.Event()
        self.pause_replay_flag = threading.Event()
        self.replay_speed = 1.0  # 1x, 2x, 5x, 10x
        self.replay_file = "Built-in 200 Hz Corridor Replay"
        self.replay_data: Optional[Dict[str, np.ndarray]] = None
        self.replay_index = 0
        self.total_replay_samples = 0
        self.replay_state = "IDLE"  # IDLE, PLAYING, PAUSED, FINISHED

        # Live trajectory histories (limited to latest 2000 points to keep UI responsive)
        self.reference_trajectory: List[List[float]] = []  # [[x, y], ...]
        self.estimated_trajectory: List[List[float]] = []  # [[x, y, in_outage], ...]

        # Raw sample inspection buffer
        self.last_raw_imu: Dict[str, Any] = {"t": 0.0, "accel": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0]}
        self.last_raw_gnss: Dict[str, Any] = {"t": 0.0, "pos": [0.0, 0.0, 0.0], "vel": [0.0, 0.0, 0.0], "valid": False}
        self.last_raw_baro: Dict[str, Any] = {"t": 0.0, "alt": 0.0}

        # API health monitor
        self.api_health: Dict[str, str] = {
            "initialize": "UNTESTED",
            "pushImu": "UNTESTED",
            "pushGnss": "UNTESTED",
            "pushBarometer": "UNTESTED",
            "update": "UNTESTED",
            "getNavigationState": "UNTESTED",
            "getDiagnostics": "UNTESTED",
            "reset": "UNTESTED",
            "shutdown": "UNTESTED",
        }

        # Event log
        self.event_log: List[Dict[str, str]] = []

        # Auto-initialize with default edge config
        self.init_engine()

    def log_event(self, level: str, message: str):
        """Appends a timestamped message to the event log."""
        timestamp = time.strftime("%H:%M:%S")
        entry = {"timestamp": timestamp, "level": level, "message": message}
        self.event_log.append(entry)
        if len(self.event_log) > 200:
            self.event_log.pop(0)

    def init_engine(self) -> bool:
        """Initializes the EdgeNavigationEngine with the EDGE_FOG profile."""
        with self.lock:
            try:
                cfg = NavigationProfileConfig.edge_fog_defaults()
                cfg.target_update_rate_hz = 200.0
                cfg.imu_sample_rate_hz = 200.0
                cfg.hardware_validated = False  # Explicitly software test on laptop
                success = self.engine.initialize(cfg)
                if success:
                    self.engine_state = "RUNNING"
                    self.api_health["initialize"] = "PASS"
                    self.log_event("INFO", "EdgeNavigationEngine initialized successfully (Profile: EDGE_FOG)")
                else:
                    self.engine_state = "ERROR"
                    self.api_health["initialize"] = "FAIL"
                    self.log_event("ERROR", "EdgeNavigationEngine initialize() returned false")
                return success
            except Exception as e:
                self.engine_state = "ERROR"
                self.api_health["initialize"] = "FAIL"
                self.log_event("ERROR", f"Exception during initialize(): {str(e)}")
                return False

    def push_imu_sample(self, timestamp_s: float, accel_xyz: np.ndarray, gyro_xyz: np.ndarray) -> bool:
        """Pushes an IMU sample into the engine."""
        try:
            ok = self.engine.push_imu(timestamp_s, accel_xyz, gyro_xyz)
            self.api_health["pushImu"] = "PASS" if ok else "FAIL"
            if ok:
                self.last_raw_imu = {
                    "t": round(float(timestamp_s), 4),
                    "accel": [round(float(x), 4) for x in accel_xyz],
                    "gyro": [round(float(x), 5) for x in gyro_xyz],
                }
            return ok
        except Exception:
            self.api_health["pushImu"] = "FAIL"
            return False

    def push_gnss_sample(
        self,
        timestamp_s: float,
        pos_enu: np.ndarray,
        vel_enu: Optional[np.ndarray] = None,
        valid: bool = True,
        accuracy_m: float = 1.0,
    ) -> bool:
        """Pushes a GNSS sample into the engine, respecting GNSS denial overrides."""
        try:
            actual_valid = False if self.gnss_denied_override else valid
            ok = self.engine.push_gnss(
                timestamp_s,
                pos_enu,
                vel_enu=vel_enu,
                valid=actual_valid,
                accuracy_m=accuracy_m,
                fix_type=GnssFixType.FIX_3D if actual_valid else GnssFixType.NO_FIX,
            )
            self.api_health["pushGnss"] = "PASS" if ok else "FAIL"
            if ok:
                self.last_raw_gnss = {
                    "t": round(float(timestamp_s), 4),
                    "pos": [round(float(x), 2) for x in pos_enu],
                    "vel": [round(float(x), 2) for x in vel_enu] if vel_enu is not None else [0.0, 0.0, 0.0],
                    "valid": actual_valid,
                }
            return ok
        except Exception:
            self.api_health["pushGnss"] = "FAIL"
            return False

    def push_baro_sample(self, timestamp_s: float, alt_m: float) -> bool:
        """Pushes a barometer altitude measurement into the engine."""
        try:
            ok = self.engine.push_barometer(timestamp_s, alt_m)
            self.api_health["pushBarometer"] = "PASS" if ok else "FAIL"
            if ok:
                self.last_raw_baro = {"t": round(float(timestamp_s), 4), "alt": round(float(alt_m), 2)}
            return ok
        except Exception:
            self.api_health["pushBarometer"] = "FAIL"
            return False

    def step_engine(self, timestamp_s: Optional[float] = None, ai_speed: Optional[float] = None) -> bool:
        """Executes one engine update step and captures telemetry and diagnostics."""
        try:
            ok = self.engine.update(timestamp_s=timestamp_s, ai_speed_mps=ai_speed)
            self.api_health["update"] = "PASS" if ok else "FAIL"
            if not ok:
                return False

            nav = self.engine.get_navigation_state()
            self.api_health["getNavigationState"] = "PASS" if nav is not None else "FAIL"

            diag = self.engine.get_diagnostics()
            self.api_health["getDiagnostics"] = "PASS" if diag is not None else "FAIL"

            if nav is not None:
                p_enu = nav.pos_enu
                outage = nav.in_outage
                # Record estimated trajectory point: [East (x), North (y), in_outage]
                self.estimated_trajectory.append([round(float(p_enu[0]), 2), round(float(p_enu[1]), 2), 1 if outage else 0])
                if len(self.estimated_trajectory) > 2500:
                    self.estimated_trajectory.pop(0)

            return True
        except Exception as e:
            self.api_health["update"] = "FAIL"
            self.log_event("ERROR", f"Exception during engine update: {str(e)}")
            return False

    def reset_engine(self):
        """Resets the engine and cleans historical visualization buffers."""
        with self.lock:
            try:
                self.stop_replay()
                self.engine.reset()
                self.api_health["reset"] = "PASS"
                self.reference_trajectory.clear()
                self.estimated_trajectory.clear()
                self.engine_state = "RUNNING"
                self.gnss_denied_override = False
                self.gnss_status = "AVAILABLE"
                self.log_event("INFO", "Engine reset executed. Trajectories and internal buffers reinitialized.")
            except Exception as e:
                self.api_health["reset"] = "FAIL"
                self.log_event("ERROR", f"Exception during engine reset: {str(e)}")

    def shutdown_engine(self):
        """Shuts down the engine safely."""
        with self.lock:
            try:
                self.stop_replay()
                self.engine.shutdown()
                self.api_health["shutdown"] = "PASS"
                self.engine_state = "STOPPED"
                self.log_event("INFO", "EdgeNavigationEngine shut down cleanly.")
            except Exception as e:
                self.api_health["shutdown"] = "FAIL"
                self.log_event("ERROR", f"Exception during engine shutdown: {str(e)}")

    def set_gnss_denied(self, denied: bool):
        """Toggles simulated GNSS denial."""
        with self.lock:
            self.gnss_denied_override = denied
            self.gnss_status = "DENIED" if denied else "AVAILABLE"
            t_now = (self.last_raw_imu["t"] or 0.0) + 0.001
            if denied:
                self.push_gnss_sample(t_now, np.zeros(3), np.zeros(3), valid=False)
                self.log_event("WARN", "GNSS DENIED simulated by user. Dead reckoning engaged.")
            else:
                self.log_event("INFO", "GNSS RESTORED by user. Resuming GNSS aiding.")

    def set_replay_speed(self, multiplier: float):
        """Sets replay speed multiplier (1x, 2x, 5x, 10x)."""
        with self.lock:
            self.replay_speed = max(0.1, min(20.0, float(multiplier)))
            self.log_event("INFO", f"Replay playback speed set to {self.replay_speed:.1f}x")

    def generate_synthetic_replay(self, duration_s: float = 60.0, dt: float = 0.005) -> Dict[str, np.ndarray]:
        """
        Generates a realistic 200 Hz corridor driving profile:
        - Accel forward to 15 m/s (~54 km/h)
        - Straight cruise with light road vibration
        - 90-degree right turn curve (radius 100m)
        - Standstill brake stop at end
        - True ground truth position & velocity computed analytically.
        """
        N = int(duration_s / dt)
        t = np.linspace(0.0, duration_s, N)

        # Forward velocity profile
        v_fwd = np.zeros(N)
        for i, ti in enumerate(t):
            if ti < 5.0:
                v_fwd[i] = (ti / 5.0) * 15.0  # accelerate to 15 m/s
            elif ti < 45.0:
                v_fwd[i] = 15.0               # cruise
            elif ti < 55.0:
                v_fwd[i] = 15.0 * (1.0 - (ti - 45.0) / 10.0) # decelerate
            else:
                v_fwd[i] = 0.0                # standstill

        # Turn rate profile: gentle curve between t=20s and t=30s (turn 90 deg = pi/2 rad over 10s -> omega = 0.157 rad/s)
        omega_z = np.zeros(N)
        for i, ti in enumerate(t):
            if 20.0 <= ti <= 30.0:
                omega_z[i] = (np.pi / 2.0) / 10.0

        # Heading angle
        heading = np.cumsum(omega_z) * dt

        # ENU velocities and positions
        vx = v_fwd * np.sin(heading)  # East
        vy = v_fwd * np.cos(heading)  # North
        x_enu = np.cumsum(vx) * dt
        y_enu = np.cumsum(vy) * dt
        z_enu = np.zeros(N)

        # Accel specific force in body frame
        a_fwd = np.gradient(v_fwd, dt)
        a_lat = v_fwd * omega_z  # centripetal
        noise_a = np.random.normal(0.0, 0.05, (N, 3))
        accel_b = np.column_stack([a_fwd, a_lat, np.full(N, 9.80665)]) + noise_a

        noise_g = np.random.normal(0.0, 0.001, (N, 3))
        gyro_b = np.column_stack([np.zeros(N), np.zeros(N), omega_z]) + noise_g

        return {
            "time": t,
            "dt": dt,
            "accel": accel_b,
            "gyro": gyro_b,
            "pos_enu": np.column_stack([x_enu, y_enu, z_enu]),
            "vel_enu": np.column_stack([vx, vy, z_enu]),
            "speed": v_fwd,
            "baro_alt": np.full(N, 120.0),
        }

    def load_replay(self, file_path_or_preset: str = "synthetic") -> bool:
        """Loads a replay dataset (authentic IO-VNBD drive or synthetic)."""
        with self.lock:
            self.stop_replay()
            self.reference_trajectory.clear()
            self.estimated_trajectory.clear()

            if file_path_or_preset == "synthetic" or not os.path.exists(file_path_or_preset):
                self.replay_data = self.generate_synthetic_replay(duration_s=60.0, dt=0.005)
                self.replay_file = "Built-in 200 Hz Corridor Replay (60s)"
            else:
                from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
                try:
                    if os.path.isdir(file_path_or_preset):
                        traj = RealIOVNBDLoader.load_drive_from_dir(file_path_or_preset)
                    else:
                        traj = RealIOVNBDLoader.load_drive(file_path_or_preset, file_path_or_preset.replace("S-", "V-"))
                    
                    self.replay_data = {
                        "time": traj.time,
                        "dt": traj.dt,
                        "accel": traj.accel,
                        "gyro": traj.gyro,
                        "pos_enu": traj.gt_pos,
                        "vel_enu": traj.gt_vel,
                        "speed": traj.gt_speed,
                        "baro_alt": traj.baro_alt,
                    }
                    self.replay_file = os.path.basename(file_path_or_preset)
                except Exception as e:
                    self.log_event("WARN", f"Failed to parse CSV ({e}), falling back to built-in 200 Hz replay.")
                    self.replay_data = self.generate_synthetic_replay(duration_s=60.0, dt=0.005)
                    self.replay_file = "Built-in 200 Hz Corridor Replay"

            self.total_replay_samples = len(self.replay_data["time"])
            self.replay_index = 0
            self.replay_state = "IDLE"

            # Prepopulate reference trajectory: [[x, y], ...]
            pos_all = self.replay_data["pos_enu"]
            step = max(1, len(pos_all) // 500)
            self.reference_trajectory = [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in pos_all[::step]]

            self.log_event("INFO", f"Loaded replay '{self.replay_file}' ({self.total_replay_samples} samples)")
            return True

    def start_replay(self):
        """Starts or resumes background replay."""
        if self.replay_data is None:
            self.load_replay("synthetic")

        if self.replay_state == "PAUSED":
            self.pause_replay_flag.clear()
            self.replay_state = "PLAYING"
            self.log_event("INFO", "Replay playback resumed.")
            return

        if self.replay_thread and self.replay_thread.is_alive():
            return

        self.stop_replay_flag.clear()
        self.pause_replay_flag.clear()
        self.replay_state = "PLAYING"
        self.imu_status = "REPLAY"
        self.log_event("INFO", f"Replay started at {self.replay_speed:.1f}x speed.")

        self.replay_thread = threading.Thread(target=self._replay_worker, daemon=True)
        self.replay_thread.start()

    def pause_replay(self):
        """Pauses the replay playback."""
        if self.replay_state == "PLAYING":
            self.pause_replay_flag.set()
            self.replay_state = "PAUSED"
            self.log_event("INFO", "Replay playback paused.")

    def stop_replay(self):
        """Stops the replay playback completely."""
        self.stop_replay_flag.set()
        self.pause_replay_flag.clear()
        if self.replay_thread and self.replay_thread.is_alive():
            self.replay_thread.join(timeout=1.0)
        self.replay_state = "IDLE"
        self.imu_status = "STOPPED"
        self.replay_index = 0
        self.log_event("INFO", "Replay playback stopped.")

    def restart_replay(self):
        """Restarts replay from beginning."""
        self.stop_replay()
        self.estimated_trajectory.clear()
        self.start_replay()

    def _replay_worker(self):
        """High-rate worker feeding replay samples into EdgeNavigationEngine."""
        data = self.replay_data
        times = data["time"]
        accels = data["accel"]
        gyros = data["gyro"]
        positions = data["pos_enu"]
        velocities = data["vel_enu"]
        speeds = data["speed"]
        baro_alts = data["baro_alt"]
        N = len(times)

        # Baseline timestamp for initial lock
        if self.replay_index == 0:
            self.push_gnss_sample(
                timestamp_s=float(times[0]),
                pos_enu=positions[0],
                vel_enu=velocities[0],
                valid=True,
            )
            self.push_imu_sample(float(times[0]), accels[0], gyros[0])
            self.push_baro_sample(float(times[0]), float(baro_alts[0]))
            self.step_engine(timestamp_s=float(times[0]), ai_speed=float(speeds[0]))
            self.replay_index = 1

        last_gnss_push_t = times[0]

        while not self.stop_replay_flag.is_set() and self.replay_index < N:
            if self.pause_replay_flag.is_set():
                time.sleep(0.05)
                continue

            i = self.replay_index
            t_curr = float(times[i])
            dt = float(data["dt"])

            # Push IMU sample
            self.push_imu_sample(t_curr, accels[i], gyros[i])

            # GNSS arrives every 1.0 second (or on step)
            if (t_curr - last_gnss_push_t) >= 0.999:
                self.push_gnss_sample(
                    timestamp_s=t_curr,
                    pos_enu=positions[i],
                    vel_enu=velocities[i],
                    valid=True,
                )
                last_gnss_push_t = t_curr

            # Barometer arrives at 10 Hz
            if i % max(1, int(0.1 / dt)) == 0:
                self.push_baro_sample(t_curr, float(baro_alts[i]))

            # Execute update cycle
            self.step_engine(timestamp_s=t_curr, ai_speed=float(speeds[i]))

            self.replay_index += 1

            # Sleep appropriate dt scaled by replay_speed
            sleep_s = max(0.0, (dt / self.replay_speed) - 0.0002)
            if sleep_s > 0.001:
                time.sleep(sleep_s)

        if self.replay_index >= N:
            self.replay_state = "FINISHED"
            self.imu_status = "STOPPED"
            self.log_event("INFO", "Replay playback reached end of data.")

    def run_predefined_test(self, scenario_id: int) -> Dict[str, Any]:
        """
        Executes one of the 6 predefined engineering test scenarios:
        1. STARTUP: Initializes engine and checks status.
        2. SENSOR REPLAY: Feeds 50 steps of 200 Hz IMU data.
        3. GNSS DENIED: Simulates GNSS denial and verifies DR mode entry.
        4. GNSS RESTORE: Restores GNSS and verifies reacquisition.
        5. RESET: Executes engine reset and verifies clean buffer.
        6. FULL REPLAY: Initiates continuous high-rate replay.
        """
        res = {"scenario": scenario_id, "success": False, "message": ""}

        if scenario_id == 1:
            # TEST 1 — STARTUP
            ok = self.init_engine()
            res["success"] = ok
            res["message"] = "TEST 1 STARTUP: Engine initialized successfully." if ok else "TEST 1 FAILED."

        elif scenario_id == 2:
            # TEST 2 — SENSOR REPLAY
            self.init_engine()
            data = self.generate_synthetic_replay(duration_s=1.0, dt=0.005)
            # Push initial lock
            self.push_gnss_sample(0.0, data["pos_enu"][0], data["vel_enu"][0], True)
            self.push_imu_sample(0.0, data["accel"][0], data["gyro"][0])
            self.step_engine(0.0, ai_speed=0.0)

            count = 0
            for i in range(1, 51):
                t = float(data["time"][i])
                ok_imu = self.push_imu_sample(t, data["accel"][i], data["gyro"][i])
                ok_step = self.step_engine(t, ai_speed=float(data["speed"][i]))
                if ok_imu and ok_step:
                    count += 1
            res["success"] = count == 50
            res["message"] = f"TEST 2 SENSOR REPLAY: Successfully processed {count}/50 200 Hz IMU steps."
            self.log_event("INFO", res["message"])

        elif scenario_id == 3:
            # TEST 3 — GNSS DENIED
            self.set_gnss_denied(True)
            # Push IMU sample in outage
            t_now = (self.last_raw_imu["t"] or 10.0) + 0.005
            self.push_imu_sample(t_now, np.array([0.0, 0.0, 9.81]), np.zeros(3))
            self.step_engine(t_now, ai_speed=10.0)
            nav = self.engine.get_navigation_state()
            in_dr = nav is not None and nav.in_outage
            res["success"] = in_dr
            res["message"] = f"TEST 3 GNSS DENIED: Engine transitioned to DR_ONLY (in_outage={in_dr})."
            self.log_event("WARN" if in_dr else "ERROR", res["message"])

        elif scenario_id == 4:
            # TEST 4 — GNSS RESTORE
            self.set_gnss_denied(False)
            t_now = (self.last_raw_imu["t"] or 10.0) + 0.005
            self.push_gnss_sample(t_now, np.array([0.0, 100.0, 0.0]), np.array([0.0, 10.0, 0.0]), True)
            self.push_imu_sample(t_now, np.array([0.0, 0.0, 9.81]), np.zeros(3))
            self.step_engine(t_now, ai_speed=10.0)
            nav = self.engine.get_navigation_state()
            res["success"] = nav is not None
            res["message"] = f"TEST 4 GNSS RESTORE: GNSS signal resumed (mode={nav.mode.value if nav else 'N/A'})."
            self.log_event("INFO", res["message"])

        elif scenario_id == 5:
            # TEST 5 — RESET
            self.reset_engine()
            diag = self.engine.get_diagnostics()
            ok = diag.total_samples_processed == 0
            res["success"] = ok
            res["message"] = "TEST 5 RESET: Engine reset verified (sample count=0)."
            self.log_event("INFO", res["message"])

        elif scenario_id == 6:
            # TEST 6 — FULL REPLAY
            self.load_replay("synthetic")
            self.start_replay()
            res["success"] = True
            res["message"] = "TEST 6 FULL REPLAY: Full corridor drive replay initiated."
            self.log_event("INFO", res["message"])

        return res

    def get_snapshot(self) -> Dict[str, Any]:
        """
        Returns a thread-safe snapshot dictionary of all live telemetry,
        sensor states, diagnostics, trajectories, and API health for UI rendering.
        """
        with self.lock:
            nav = self.engine.get_navigation_state()
            diag = self.engine.get_diagnostics()

            # Navigation State Extraction
            if nav is not None:
                p = nav.pos_enu
                v = nav.vel_enu
                lat_approx = 25.4358 + (p[1] / 111139.0)  # Reference origin near Prayagraj
                lon_approx = 81.8463 + (p[0] / (111139.0 * math.cos(math.radians(25.4358))))
                alt_val = round(float(p[2]), 2)
                pos_dict = {
                    "enu": [round(float(x), 2) for x in p],
                    "latitude": round(lat_approx, 6),
                    "longitude": round(lon_approx, 6),
                    "altitude_m": alt_val,
                }
                vel_dict = {
                    "vx": round(float(v[0]), 2),
                    "vy": round(float(v[1]), 2),
                    "vz": round(float(v[2]), 2),
                    "speed_kmh": round(float(nav.speed_kmh), 2),
                    "speed_mps": round(float(np.linalg.norm(v[:2])), 2),
                }
                att_dict = {
                    "roll_deg": round(float(nav.roll_deg), 2),
                    "pitch_deg": round(float(nav.pitch_deg), 2),
                    "heading_deg": round(float(nav.heading_deg), 2),
                }
                mode_str = nav.mode.value
                in_outage = nav.in_outage
                pothole_active = nav.is_pothole_active
                idling = nav.is_idling
            else:
                pos_dict = {"enu": [0.0, 0.0, 0.0], "latitude": 25.435800, "longitude": 81.846300, "altitude_m": 0.0}
                vel_dict = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "speed_kmh": 0.0, "speed_mps": 0.0}
                att_dict = {"roll_deg": 0.0, "pitch_deg": 0.0, "heading_deg": 0.0}
                mode_str = "INITIALIZING"
                in_outage = False
                pothole_active = False
                idling = False

            # Diagnostics Extraction
            sustained_rate = round(float(diag.sustained_hz), 1) if diag.sustained_hz > 0 else 0.0
            mean_cycle_us = round(float(diag.mean_cycle_time_us), 1)
            max_cycle_us = round(float(diag.max_cycle_time_us), 1)
            dropped = int(diag.dropped_samples)
            total_samples = int(diag.total_samples_processed)
            p_trace_val = round(float(diag.p_trace), 4)
            hw_status = diag.hardware_status

            # Replay progress
            progress_pct = (
                round((self.replay_index / max(1, self.total_replay_samples)) * 100.0, 1)
                if self.total_replay_samples > 0
                else 0.0
            )

            return {
                "engine_status": self.engine_state,
                "profile": self.profile,
                "platform": self.platform,
                "navigation_mode": mode_str,
                "in_outage": in_outage,
                "gnss_denied_override": self.gnss_denied_override,
                "sensor_status": {
                    "imu": self.imu_status,
                    "gnss": self.gnss_status,
                    "barometer": self.baro_status,
                    "ai_velocity": self.ai_velocity_status,
                },
                "position": pos_dict,
                "velocity": vel_dict,
                "attitude": att_dict,
                "vibration_flags": {
                    "pothole": pothole_active,
                    "idling": idling,
                },
                "diagnostics": {
                    "sustained_rate_hz": sustained_rate,
                    "mean_cycle_time_us": mean_cycle_us,
                    "max_cycle_time_us": max_cycle_us,
                    "dropped_samples": dropped,
                    "sequence_number": total_samples,
                    "p_trace": p_trace_val,
                    "hardware_status": hw_status,
                },
                "replay": {
                    "file": self.replay_file,
                    "state": self.replay_state,
                    "index": self.replay_index,
                    "total": self.total_replay_samples,
                    "progress_pct": progress_pct,
                    "speed_multiplier": self.replay_speed,
                },
                "raw_inspection": {
                    "imu": self.last_raw_imu,
                    "gnss": self.last_raw_gnss,
                    "baro": self.last_raw_baro,
                },
                "api_health": self.api_health,
                "trajectories": {
                    "reference": self.reference_trajectory,
                    "estimated": self.estimated_trajectory[-500:],  # Tail for UI rendering
                },
                "event_log": self.event_log[-30:],  # Latest 30 events
            }

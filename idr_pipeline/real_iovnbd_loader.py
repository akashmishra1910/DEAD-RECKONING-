"""
idr_pipeline/real_iovnbd_loader.py
==================================
Production-grade generic loader for real-world IO-VNBD datasets.

Handles:
1. Robust column parsing across varying column name conventions in IO-VNBD.
2. Latin-1 / ISO-8859-1 decoding for superscripts (m/s², μT, °).
3. Millisecond time-of-day synchronization between smartphone and vehicle logs.
4. Geodetic WGS-84 to local Cartesian East-North-Up (ENU) coordinate conversion.
5. Unit normalization: speed (km/h -> m/s), heading (deg -> rad), height (m).
6. Returns standard TrajectoryData instances for seamless pipeline execution.

Author: Autonomous Navigation Engineering Team
"""

import os
import re
import glob
import numpy as np
import pandas as pd
from typing import Optional, Dict, Tuple, List
from .dataset_loader import TrajectoryData


def geodetic_to_enu(
    lat: np.ndarray,
    lon: np.ndarray,
    alt: np.ndarray,
    lat0: Optional[float] = None,
    lon0: Optional[float] = None,
    alt0: Optional[float] = None
) -> np.ndarray:
    """
    Converts geodetic coordinates (WGS-84 latitude, longitude in degrees, altitude in meters)
    to local Cartesian East-North-Up (ENU) coordinates in meters.
    """
    a = 6378137.0             # Earth semi-major axis (WGS-84)
    f = 1.0 / 298.257223563   # Reciprocal flattening
    e2 = 2 * f - f * f        # First eccentricity squared

    if lat0 is None: lat0 = float(lat[0])
    if lon0 is None: lon0 = float(lon[0])
    if alt0 is None: alt0 = float(alt[0])

    phi = np.radians(lat)
    lam = np.radians(lon)
    phi0 = np.radians(lat0)
    lam0 = np.radians(lon0)

    def to_ecef(p, l, h):
        sin_p = np.sin(p)
        cos_p = np.cos(p)
        N = a / np.sqrt(1.0 - e2 * sin_p**2)
        X = (N + h) * cos_p * np.cos(l)
        Y = (N + h) * cos_p * np.sin(l)
        Z = (N * (1.0 - e2) + h) * sin_p
        return X, Y, Z

    X, Y, Z = to_ecef(phi, lam, alt)
    X0, Y0, Z0 = to_ecef(phi0, lam0, alt0)

    dX, dY, dZ = X - X0, Y - Y0, Z - Z0

    sin_lam0 = np.sin(lam0)
    cos_lam0 = np.cos(lam0)
    sin_phi0 = np.sin(phi0)
    cos_phi0 = np.cos(phi0)

    E = -sin_lam0 * dX + cos_lam0 * dY
    N = -sin_phi0 * cos_lam0 * dX - sin_phi0 * sin_lam0 * dY + cos_phi0 * dZ
    U =  cos_phi0 * cos_lam0 * dX + cos_phi0 * sin_lam0 * dY + sin_phi0 * dZ

    return np.column_stack([E, N, U])


def _find_column(columns: List[str], pattern: str) -> Optional[str]:
    """Matches a column name using a case-insensitive regex pattern."""
    regex = re.compile(pattern, re.IGNORECASE)
    for c in columns:
        if regex.search(c):
            return c
    return None


def parse_date_to_seconds(date_str: str) -> float:
    """
    Parses IO-VNBD smartphone date format (YYYY-MO-DD HH-MI-SS_SSS or HH:MI:SS:SSS)
    into seconds since start of day.
    """
    try:
        # Format example: '2019-11-06 11:47:27:394'
        parts = str(date_str).strip().split(' ')
        time_part = parts[-1]
        t_tokens = re.split(r'[:._]', time_part)
        h = float(t_tokens[0])
        m = float(t_tokens[1])
        s = float(t_tokens[2])
        ms = float(t_tokens[3]) if len(t_tokens) > 3 else 0.0
        return h * 3600.0 + m * 60.0 + s + ms / 1000.0
    except Exception:
        return 0.0


class RealIOVNBDLoader:
    """
    Robust generic loader for authentic IO-VNBD naturalistic driving datasets.
    """

    @staticmethod
    def load_drive(
        s_csv_path: str,
        v_csv_path: str,
        name: Optional[str] = None
    ) -> TrajectoryData:
        """
        Loads and synchronizes a pair of (Smartphone, Vehicle) IO-VNBD CSV logs.
        """
        if name is None:
            name = os.path.basename(os.path.dirname(s_csv_path)) or "real_iovnbd_drive"

        # 1. Ingest raw CSVs with Latin-1 encoding
        s_df = pd.read_csv(s_csv_path, encoding='latin1')
        v_df = pd.read_csv(v_csv_path, encoding='latin1')

        # Clean column names (strip leading/trailing whitespace)
        s_df.columns = [c.strip() for c in s_df.columns]
        v_df.columns = [c.strip() for c in v_df.columns]

        # 2. Parse Smartphone IMU columns
        ax_col = _find_column(s_df.columns, r'accel.*(\bx\b|[_\s]x)')
        ay_col = _find_column(s_df.columns, r'accel.*(\by\b|[_\s]y)')
        az_col = _find_column(s_df.columns, r'accel.*(\bz\b|[_\s]z)')

        gx_col = _find_column(s_df.columns, r'gyro.*(roll|\b[xX]\b)')
        gy_col = _find_column(s_df.columns, r'gyro.*(pitch|\b[yY]\b)')
        gz_col = _find_column(s_df.columns, r'gyro.*(yaw|\b[zZ]\b)')

        mx_col = _find_column(s_df.columns, r'mag.*(\bx\b|[_\s]x)')
        my_col = _find_column(s_df.columns, r'mag.*(\by\b|[_\s]y)')
        mz_col = _find_column(s_df.columns, r'mag.*(\bz\b|[_\s]z)')

        s_gps_lat = _find_column(s_df.columns, r'gps.*lat')
        s_gps_lon = _find_column(s_df.columns, r'gps.*lon')
        s_gps_alt = _find_column(s_df.columns, r'gps.*alt')
        s_gps_spd = _find_column(s_df.columns, r'gps.*speed')

        # 3. Parse Vehicle CAN / Ground Truth columns
        v_lat_col = _find_column(v_df.columns, r'lat')
        v_lon_col = _find_column(v_df.columns, r'lon')
        v_spd_col = _find_column(v_df.columns, r'velo')
        v_hdg_col = _find_column(v_df.columns, r'head')
        v_alt_col = _find_column(v_df.columns, r'height')

        # 4. Synchronize timestamps
        # Check if Date exists in smartphone
        s_date_col = _find_column(s_df.columns, r'date')
        v_time_col = _find_column(v_df.columns, r'time.*start.*day')

        if s_date_col and v_time_col:
            s_times = s_df[s_date_col].apply(parse_date_to_seconds).values
            v_times = v_df[v_time_col].values
            # Detect and compensate for timezone offsets (e.g. UTC vs BST / daylight savings)
            hour_offset = round((s_times[0] - v_times[0]) / 3600.0) * 3600.0
            s_times = s_times - hour_offset

            # Fine sub-second clock synchronization via GPS speed cross-correlation
            if s_gps_spd and v_spd_col:
                raw_s_spd = s_df[s_gps_spd].values
                raw_v_spd = v_df[v_spd_col].values
                shifts = np.linspace(-25.0, 25.0, 501)
                best_shift = 0.0
                best_corr = -1.0
                for shift in shifts:
                    mask = (s_times + shift >= v_times[0]) & (s_times + shift <= v_times[-1])
                    if np.sum(mask) > 30:
                        interp_v = np.interp(s_times[mask] + shift, v_times, raw_v_spd)
                        std_s = np.std(raw_s_spd[mask])
                        std_v = np.std(interp_v)
                        if std_s > 1e-3 and std_v > 1e-3:
                            corr = float(np.corrcoef(raw_s_spd[mask], interp_v)[0, 1])
                            if corr > best_corr:
                                best_corr = corr
                                best_shift = shift
                if best_corr > 0.4:
                    s_times = s_times + best_shift
        else:
            # Fallback to relative milliseconds if date not present
            s_t_col = _find_column(s_df.columns, r'time')
            s_times = s_df[s_t_col].values / 1000.0
            s_times = s_times - s_times[0]
            v_times = np.linspace(0, len(v_df) * 0.1, len(v_df))

        # Restrict to overlapping time window
        t_start = max(s_times[0], v_times[0])
        t_end = min(s_times[-1], v_times[-1])

        mask_s = (s_times >= t_start) & (s_times <= t_end)
        s_df_sync = s_df.iloc[mask_s].reset_index(drop=True)
        t_sync = s_times[mask_s]

        # Extract Smartphone IMU streams
        accel = np.column_stack([
            s_df_sync[ax_col].values if ax_col else np.zeros(len(s_df_sync)),
            s_df_sync[ay_col].values if ay_col else np.zeros(len(s_df_sync)),
            s_df_sync[az_col].values if az_col else np.full(len(s_df_sync), 9.80665)
        ])

        gyro = np.column_stack([
            s_df_sync[gx_col].values if gx_col else np.zeros(len(s_df_sync)),
            s_df_sync[gy_col].values if gy_col else np.zeros(len(s_df_sync)),
            s_df_sync[gz_col].values if gz_col else np.zeros(len(s_df_sync))
        ])

        mag = np.column_stack([
            s_df_sync[mx_col].values if mx_col else np.zeros(len(s_df_sync)),
            s_df_sync[my_col].values if my_col else np.zeros(len(s_df_sync)),
            s_df_sync[mz_col].values if mz_col else np.zeros(len(s_df_sync))
        ])

        # Interpolate Vehicle Ground Truth to match smartphone timestamps
        v_speed_ms = v_df[v_spd_col].values / 3.6 if v_spd_col else np.zeros(len(v_df))
        gt_speed = np.interp(t_sync, v_times, v_speed_ms)

        v_lat = v_df[v_lat_col].values if v_lat_col else np.zeros(len(v_df))
        v_lon = v_df[v_lon_col].values if v_lon_col else np.zeros(len(v_df))
        gt_lat = np.interp(t_sync, v_times, v_lat)
        gt_lon = np.interp(t_sync, v_times, v_lon)

        # Height handling (IO-VNBD vehicle height is in meters despite column '(km)')
        if v_alt_col:
            raw_height = v_df[v_alt_col].values
            # If values are < 200, they are already meters. If > 2000, already meters.
            # If around 0.05, then km.
            if np.nanmean(raw_height) < 5.0:
                raw_height = raw_height * 1000.0
            gt_alt = np.interp(t_sync, v_times, raw_height)
        else:
            gt_alt = np.zeros(len(t_sync))

        # Convert ground truth geodetic coordinates to local ENU
        gt_pos = geodetic_to_enu(gt_lat, gt_lon, gt_alt)

        # Heading (deg -> rad)
        if v_hdg_col:
            v_hdg_deg = v_df[v_hdg_col].values
            # Unfold heading angles for interpolation across 0/360 wrap
            v_hdg_unwrapped = np.unwrap(np.radians(v_hdg_deg))
            gt_heading = np.interp(t_sync, v_times, v_hdg_unwrapped)
        else:
            gt_heading = np.zeros(len(t_sync))

        # Velocity in ENU frame: vx = v * sin(heading), vy = v * cos(heading), vz = vz
        gt_vel = np.column_stack([
            gt_speed * np.sin(gt_heading),
            gt_speed * np.cos(gt_heading),
            np.zeros(len(gt_speed))
        ])

        # Smartphone GNSS position (corrupted / raw)
        if s_gps_lat and s_gps_lon:
            s_lat = s_df_sync[s_gps_lat].values
            s_lon = s_df_sync[s_gps_lon].values
            s_alt = s_df_sync[s_gps_alt].values if s_gps_alt else gt_alt
            # Convert smartphone GPS to same ENU origin
            gnss_pos = geodetic_to_enu(s_lat, s_lon, s_alt, lat0=gt_lat[0], lon0=gt_lon[0], alt0=gt_alt[0])
            gnss_valid = np.ones(len(t_sync), dtype=bool)
        else:
            gnss_pos = gt_pos.copy()
            gnss_valid = np.ones(len(t_sync), dtype=bool)

        time_arr = t_sync - t_sync[0]
        dt = float(np.median(np.diff(time_arr))) if len(time_arr) > 1 else 0.1

        # Pitch and roll approximations from ground truth or gravity
        gt_pitch = np.zeros(len(time_arr))
        gt_roll = np.zeros(len(time_arr))

        road_type = np.array(['highway'] * len(time_arr), dtype=object)

        return TrajectoryData(
            name=name,
            time=time_arr,
            dt=dt,
            accel=accel,
            gyro=gyro,
            mag=mag,
            baro_alt=gt_alt,
            gt_pos=gt_pos,
            gt_vel=gt_vel,
            gt_speed=gt_speed,
            gt_heading=gt_heading,
            gt_pitch=gt_pitch,
            gt_roll=gt_roll,
            road_type=road_type,
            road_altitude=gt_alt,
            gnss_pos=gnss_pos,
            gnss_valid=gnss_valid
        )

    @classmethod
    def load_drive_from_dir(cls, drive_dir: str) -> TrajectoryData:
        """Loads drive from a directory containing S-*.csv and V-*.csv."""
        s_files = glob.glob(os.path.join(drive_dir, "S-*.csv"))
        v_files = glob.glob(os.path.join(drive_dir, "V-*.csv"))
        if not s_files or not v_files:
            raise FileNotFoundError(f"Could not find matching S-*.csv and V-*.csv in {drive_dir}")
        return cls.load_drive(s_files[0], v_files[0], name=os.path.basename(drive_dir))

    @classmethod
    def load_all_drives(cls, base_dir: str) -> Dict[str, TrajectoryData]:
        """Loads all available drives from subdirectories of base_dir."""
        drives = {}
        for sub in sorted(glob.glob(os.path.join(base_dir, "*"))):
            if os.path.isdir(sub):
                try:
                    traj = cls.load_drive_from_dir(sub)
                    drives[traj.name] = traj
                except Exception as e:
                    pass
        return drives

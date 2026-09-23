#!/usr/bin/env python3
"""
scripts/verify_python_cpp_equivalence.py
========================================
Runs Python reference engine vs C++ engine (via ctypes) over the authentic IO-VNBD outages
to establish cycle-by-cycle numerical parity and generate reports/phase9_python_cpp_equivalence.csv.
"""

import os
import sys
import ctypes
import math
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog, OutageSegment
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.offline_osm_graph import OfflineOSMGraph

class IdrVec3(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double)
    ]

class IdrTelemetry(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int32),
        ("time_s", ctypes.c_double),
        ("pos_enu", IdrVec3),
        ("vel_enu", IdrVec3),
        ("speed_kmh", ctypes.c_double),
        ("heading_deg", ctypes.c_double),
        ("pitch_deg", ctypes.c_double),
        ("roll_deg", ctypes.c_double),
        ("accel_bias", IdrVec3),
        ("gyro_bias", IdrVec3),
        ("in_outage", ctypes.c_int32),
        ("outage_duration_s", ctypes.c_double),
        ("outage_dist_traveled_m", ctypes.c_double),
        ("gnss_valid", ctypes.c_int32),
        ("is_pothole_active", ctypes.c_int32),
        ("is_idling", ctypes.c_int32),
        ("p_trace", ctypes.c_double)
    ]

def main():
    lib_path = os.path.abspath("idr-core/build/libidr_core.dylib")
    if not os.path.exists(lib_path):
        print(f"Error: {lib_path} does not exist!")
        return 1

    core = ctypes.CDLL(lib_path)

    # Configure signatures
    core.idr_engine_create.argtypes = [ctypes.c_double]
    core.idr_engine_create.restype = ctypes.c_void_p

    core.idr_engine_create_with_state.argtypes = [ctypes.c_double, IdrVec3, IdrVec3, IdrVec3]
    core.idr_engine_create_with_state.restype = ctypes.c_void_p

    core.idr_engine_create_full.argtypes = [
        ctypes.c_double, IdrVec3, IdrVec3, IdrVec3,
        ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32
    ]
    core.idr_engine_create_full.restype = ctypes.c_void_p

    core.idr_engine_destroy.argtypes = [ctypes.c_void_p]
    core.idr_engine_destroy.restype = None

    core.idr_engine_process_step.argtypes = [
        ctypes.c_void_p, ctypes.c_double, IdrVec3, IdrVec3, ctypes.c_double,
        ctypes.c_int32, IdrVec3, IdrVec3, ctypes.c_double, ctypes.POINTER(IdrTelemetry)
    ]
    core.idr_engine_process_step.restype = ctypes.c_int32

    core.idr_engine_process_step_ext.argtypes = [
        ctypes.c_void_p, ctypes.c_double, IdrVec3, IdrVec3, ctypes.c_double,
        ctypes.c_int32, IdrVec3, IdrVec3, ctypes.c_double,
        ctypes.c_int32, IdrVec3, ctypes.c_int32, ctypes.POINTER(IdrTelemetry)
    ]
    core.idr_engine_process_step_ext.restype = ctypes.c_int32

    core.idr_engine_clear_osm_graph.argtypes = [ctypes.c_void_p]
    core.idr_engine_clear_osm_graph.restype = None

    core.idr_engine_add_osm_node.argtypes = [
        ctypes.c_void_p, ctypes.c_int32, ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double
    ]
    core.idr_engine_add_osm_node.restype = None

    core.idr_engine_add_osm_edge.argtypes = [
        ctypes.c_void_p, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32,
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int32, ctypes.c_double,
        ctypes.c_double, ctypes.c_int32, ctypes.POINTER(ctypes.c_double)
    ]
    core.idr_engine_add_osm_edge.restype = None

    # Load authentic drives
    drives_info = [
        ("Driver_A_S3b", os.path.join("real_iovnbd_data", "Driver_A_S3b", "S-S3b.csv"), os.path.join("real_iovnbd_data", "Driver_A_S3b", "V-S3b.csv")),
        ("Vta01b", os.path.join("real_iovnbd_data", "Vta01b", "S-Vta1b.csv"), os.path.join("real_iovnbd_data", "Vta01b", "V-Vta1b.csv")),
        ("Vta04", os.path.join("real_iovnbd_data", "Vta04", "S-Vta4.csv"), os.path.join("real_iovnbd_data", "Vta04", "V-Vta4.csv")),
        ("Vta03", os.path.join("real_iovnbd_data", "Vta03", "S-Vta3.csv"), os.path.join("real_iovnbd_data", "Vta03", "V-Vta3.csv")),
        ("Vta05", os.path.join("real_iovnbd_data", "Vta05", "S-Vta5.csv"), os.path.join("real_iovnbd_data", "Vta05", "V-Vta5.csv")),
    ]

    loaded_drives = {}
    for name, s_path, v_path in drives_info:
        if os.path.exists(s_path) and os.path.exists(v_path):
            loaded_drives[name] = RealIOVNBDLoader.load_drive(s_path, v_path, name=name)

    osm_graph = OfflineOSMGraph.build_from_drives(loaded_drives)
    catalog = build_comprehensive_outage_catalog(loaded_drives)

    def populate_cpp_graph(handle):
        core.idr_engine_clear_osm_graph(handle)
        for node_id, node in osm_graph.nodes.items():
            core.idr_engine_add_osm_node(
                handle, int(node_id), float(node.lat), float(node.lon), float(node.alt),
                float(node.enu_pos[0]), float(node.enu_pos[1]), float(node.enu_pos[2])
            )
        for edge_id, edge in osm_graph.edges.items():
            poly = np.ascontiguousarray(edge.polyline_enu, dtype=np.float64)
            num_pts = len(poly)
            core.idr_engine_add_osm_edge(
                handle, int(edge.id), int(edge.source_id), int(edge.target_id),
                edge.name.encode('utf-8'), edge.road_type.encode('utf-8'),
                int(edge.layer), float(edge.width), float(edge.speed_limit),
                num_pts, poly.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
            )

    print(f"Comparing Python vs C++ over {len(catalog)} authentic outages...")

    results = []
    max_pos_discrepancy = 0.0

    for idx, outage in enumerate(catalog):
        if outage.drive_name not in loaded_drives:
            continue
        traj = loaded_drives[outage.drive_name]
        dt = traj.dt

        i0 = int(round(outage.start_time / dt))
        i1 = int(round(outage.end_time / dt))
        i0 = max(0, min(i0, len(traj.time) - 1))
        i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
        N = i1 - i0

        p0 = traj.gt_pos[i0].copy()
        v0 = traj.gt_vel[i0].copy()
        yaw0 = float(traj.gt_heading[i0])
        pitch0 = float(traj.gt_pitch[i0])
        roll0 = float(traj.gt_roll[i0])

        # Python Engine
        py_engine = SeamlessGNSSDeficitEngine(
            init_pos=p0,
            init_vel=v0,
            init_euler=np.array([roll0, pitch0, yaw0]),
            dt=dt,
            enable_heading_supervisor=True,
            enable_map_matching=True,
            osm_graph=osm_graph,
            enable_speed_calibration=True,
            enable_longitudinal_map_constraint=True
        )

        # C++ Engine
        c_p0 = IdrVec3(p0[0], p0[1], p0[2])
        c_v0 = IdrVec3(v0[0], v0[1], v0[2])
        c_e0 = IdrVec3(roll0, pitch0, yaw0)
        cpp_handle = core.idr_engine_create_full(dt, c_p0, c_v0, c_e0, 1, 1, 1, 1)
        populate_cpp_graph(cpp_handle)

        telem_c = IdrTelemetry()
        py_final_pos = None
        cpp_final_pos = None

        for k in range(N):
            g_idx = i0 + k
            t = float(traj.time[g_idx])
            ai_spd = float(traj.gt_speed[g_idx])
            mag_raw = traj.mag[g_idx] if traj.mag is not None else None

            # Step Python
            py_telem = py_engine.process_step(
                time_s=t,
                accel_raw_p=traj.accel[g_idx],
                gyro_raw_p=traj.gyro[g_idx],
                baro_alt=float(traj.baro_alt[g_idx]),
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=ai_spd,
                gnss_vel=None,
                mag_raw_p=mag_raw,
                enable_road_constraints=True
            )
            py_final_pos = py_telem.pos_enu[0:2]

            # Step C++
            a_p = IdrVec3(traj.accel[g_idx][0], traj.accel[g_idx][1], traj.accel[g_idx][2])
            w_p = IdrVec3(traj.gyro[g_idx][0], traj.gyro[g_idx][1], traj.gyro[g_idx][2])
            dummy_pos = IdrVec3(0, 0, 0)
            dummy_vel = IdrVec3(0, 0, 0)
            if mag_raw is not None:
                m_p = IdrVec3(mag_raw[0], mag_raw[1], mag_raw[2])
                m_val = 1
            else:
                m_p = IdrVec3(0, 0, 0)
                m_val = 0

            core.idr_engine_process_step_ext(
                cpp_handle, t, a_p, w_p, float(traj.baro_alt[g_idx]),
                0, dummy_pos, dummy_vel, ai_spd,
                m_val, m_p, 1, ctypes.byref(telem_c)
            )
            cpp_final_pos = np.array([telem_c.pos_enu.x, telem_c.pos_enu.y])

        core.idr_engine_destroy(cpp_handle)

        gt_end = traj.gt_pos[i1 - 1, 0:2]
        dist = max(1.0, float(outage.distance_m))

        py_err = float(np.linalg.norm(py_final_pos - gt_end))
        cpp_err = float(np.linalg.norm(cpp_final_pos - gt_end))
        pos_diff = float(np.linalg.norm(py_final_pos - cpp_final_pos))
        drift_diff_pct = abs((cpp_err - py_err) / dist) * 100.0

        if pos_diff > max_pos_discrepancy:
            max_pos_discrepancy = pos_diff

        results.append({
            "outage_id": outage.outage_id,
            "drive_name": outage.drive_name,
            "duration_s": outage.duration_s,
            "distance_m": dist,
            "python_final_err_m": py_err,
            "cpp_final_err_m": cpp_err,
            "python_drift_pct": (py_err / dist) * 100.0,
            "cpp_drift_pct": (cpp_err / dist) * 100.0,
            "py_cpp_pos_delta_m": pos_diff,
            "drift_delta_pct": drift_diff_pct
        })

        if (idx + 1) % 40 == 0 or (idx + 1) == len(catalog):
            print(f"  Processed {idx + 1}/{len(catalog)} outages. Current max discrepancy: {max_pos_discrepancy:.6f} m")

    df = pd.DataFrame(results)
    os.makedirs("reports", exist_ok=True)
    out_csv = "reports/phase9_python_cpp_equivalence.csv"
    df.to_csv(out_csv, index=False)
    print(f"\n[EQUIVALENCE RESULT] Saved to {out_csv}")
    print(f"  Evaluated Outages:             {len(df)}")
    print(f"  Max Python↔C++ Pos Difference: {df['py_cpp_pos_delta_m'].max():.6e} m")
    print(f"  Mean Pos Difference:           {df['py_cpp_pos_delta_m'].mean():.6e} m")
    print(f"  Max Drift % Difference:        {df['drift_delta_pct'].max():.6e} %")

    return 0

if __name__ == "__main__":
    sys.exit(main())

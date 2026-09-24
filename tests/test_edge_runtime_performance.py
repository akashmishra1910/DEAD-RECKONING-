import time
import numpy as np
import pytest
from idr_pipeline.dual_profile import (
    NavigationProfileConfig,
    EdgeNavigationEngine,
)

def test_edge_runtime_timing_path():
    """
    Benchmarks the high-rate software timing path of EdgeNavigationEngine.
    Simulates 400 cycles at 200 Hz (dt = 0.005 s).
    Measures per-cycle execution latency, mean cycle time, and sustained rate.
    """
    engine = EdgeNavigationEngine()
    cfg = NavigationProfileConfig.edge_fog_defaults()
    cfg.target_update_rate_hz = 200.0
    cfg.imu_sample_rate_hz = 200.0
    engine.initialize(cfg)

    # Initial GNSS lock
    engine.push_gnss(0.0, np.zeros(3), np.array([0.0, 15.0, 0.0]), True)
    engine.push_imu(0.0, np.array([0.0, 0.0, 9.80665]), np.zeros(3))
    engine.update(0.0, ai_speed_mps=15.0)

    num_samples = 400 # 2 seconds of 200 Hz data
    dt = 0.005

    # Run in GNSS-denied mode (DR only) to measure the core filter propagation & update overhead
    start_total = time.perf_counter()
    for i in range(1, num_samples + 1):
        t = i * dt
        engine.push_imu(t, np.array([0.0, 0.0, 9.80665]), np.array([0.001, 0.0, 0.002]))
        ok = engine.update(t, ai_speed_mps=15.0)
        assert ok is True
    total_elapsed = time.perf_counter() - start_total

    diag = engine.get_diagnostics()
    mean_us = diag["mean_cycle_time_us"]
    max_us = diag["max_cycle_time_us"]
    sustained_hz = diag["sustained_hz"]

    print(f"\n[EDGE ENGINE RUNTIME BENCHMARK]")
    print(f"Total cycles processed: {diag['total_samples_processed']}")
    print(f"Total wall time for 400 cycles: {total_elapsed*1000:.2f} ms")
    print(f"Mean cycle time: {mean_us:.2f} us")
    print(f"Max cycle time: {max_us:.2f} us")
    print(f"Sustained software throughput: {sustained_hz:.1f} Hz (target >= 200 Hz)")
    print(f"Hardware validation status: {diag['hardware_status']}")

    # The 200 Hz budget is 5000 us per sample. The software timing path should easily exceed this.
    assert mean_us < 5000.0, f"Mean cycle time {mean_us} us exceeded 5000 us (200 Hz budget)"
    assert sustained_hz > 200.0, f"Sustained rate {sustained_hz} Hz fell below 200 Hz requirement"
    assert "PENDING" in diag["hardware_status"]

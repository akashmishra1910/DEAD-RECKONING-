#include "idr/navigation_engine.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#ifdef __APPLE__
#include <mach/mach.h>
#elif defined(__linux__) || defined(__ANDROID__)
#include <sys/resource.h>
#include <unistd.h>
#include <fstream>
#elif defined(_WIN32)
#include <windows.h>
#include <psapi.h>
#endif

// Global allocation counter for tactical loop verification
static bool g_track_allocations = false;
static uint64_t g_tactical_alloc_count = 0;
static uint64_t g_tactical_alloc_bytes = 0;

void* operator new(std::size_t size) {
    if (g_track_allocations) {
        g_tactical_alloc_count++;
        g_tactical_alloc_bytes += size;
    }
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void operator delete(void* p) noexcept {
    std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}

void* operator new[](std::size_t size) {
    if (g_track_allocations) {
        g_tactical_alloc_count++;
        g_tactical_alloc_bytes += size;
    }
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void operator delete[](void* p) noexcept {
    std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
    std::free(p);
}

static size_t get_peak_rss_bytes() {
#ifdef __APPLE__
    struct mach_task_basic_info info;
    mach_msg_type_number_t info_count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &info_count) == KERN_SUCCESS) {
        return info.resident_size_max;
    }
#elif defined(__linux__) || defined(__ANDROID__)
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        return (size_t)usage.ru_maxrss * 1024;
    }
#endif
    return 0;
}

static size_t get_current_rss_bytes() {
#ifdef __APPLE__
    struct mach_task_basic_info info;
    mach_msg_type_number_t info_count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &info_count) == KERN_SUCCESS) {
        return info.resident_size;
    }
#elif defined(__linux__) || defined(__ANDROID__)
    std::ifstream statm("/proc/self/statm");
    if (statm.is_open()) {
        unsigned long size, resident;
        if (statm >> size >> resident) {
            long page_size = sysconf(_SC_PAGESIZE);
            return resident * page_size;
        }
    }
#endif
    return 0;
}

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << "IDR-CORE PRODUCTION ENGINE: 100,000-CYCLE TACTICAL BENCHMARK" << std::endl;
    std::cout << "============================================================" << std::endl;

    const size_t TOTAL_CYCLES = 100000;
    const double DT = 0.01; // 100 Hz simulation

    // Step 1: Pre-initialize engine outside allocation tracking
    idr::Vector3 init_pos(0.0, 0.0, 0.0);
    idr::Vector3 init_vel(10.0, 0.0, 0.0); // 10 m/s East
    idr::Vector3 init_euler(0.0, 0.0, 1.57079632679); // 90 deg heading (East)

    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();

    auto engine = std::make_unique<idr::SeamlessGNSSDeficitEngine>(
        init_pos, init_vel, init_euler,
        11.345, 9.0, DT,
        true, true, graph, 4, true, true
    );

    // Warm-up Phase A: 30 stationary samples → triggers leveling (leveling_samples=25)
    for (size_t i = 0; i < 30; ++i) {
        double t = i * DT;
        idr::Vector3 a_p(0.0, 0.0, 9.80665);   // pure gravity, stationary
        idr::Vector3 w_p(0.0, 0.0, 0.0);        // no rotation
        idr::Vector3 gnss_pos(0.0, 0.0, 0.0);
        idr::Vector3 gnss_vel(0.0, 0.0, 0.0);
        idr::Vector3 mag_p(20.0, 5.0, 42.0);
        engine->process_step(t, a_p, w_p, 100.0,
            gnss_pos, true, 0.0, gnss_vel, mag_p, false);
    }
    // Warm-up Phase B: 20 forward-surge samples → fills transient_accels_ → triggers alignment
    for (size_t i = 0; i < 20; ++i) {
        double t = (30 + i) * DT;
        idr::Vector3 a_p(1.5, 0.0, 9.80665);    // forward surge
        idr::Vector3 w_p(0.0, 0.0, 0.0);
        idr::Vector3 gnss_pos(1.5 * i * DT, 0.0, 0.0);
        idr::Vector3 gnss_vel(1.5, 0.0, 0.0);
        idr::Vector3 mag_p(20.0, 5.0, 42.0);
        engine->process_step(t, a_p, w_p, 100.0,
            gnss_pos, true, 1.5, gnss_vel, mag_p, false);
    }
    // Warm-up Phase C: 100 cruising samples at steady state to settle caches
    for (size_t i = 0; i < 100; ++i) {
        double t = (50 + i) * DT;
        idr::Vector3 a_p(0.0, 0.0, 9.80665);
        idr::Vector3 w_p(0.0, 0.0, 0.05 * std::sin(0.1 * t));
        idr::Vector3 gnss_pos(10.0 * t, 0.0, 0.0);
        idr::Vector3 gnss_vel(10.0, 0.0, 0.0);
        idr::Vector3 mag_p(20.0, 5.0, 42.0);
        engine->process_step(t, a_p, w_p, 100.0,
            gnss_pos, true, 10.0, gnss_vel, mag_p, true);
    }

    std::vector<double> latencies_us;
    latencies_us.reserve(TOTAL_CYCLES);

    size_t rss_before = get_current_rss_bytes();

    std::cout << "\nStarting 100,000 continuous tactical loop cycles with allocation interception..." << std::endl;

    // ACTIVATE HEAP ALLOCATION MONITORING
    g_tactical_alloc_count = 0;
    g_tactical_alloc_bytes = 0;
    g_track_allocations = true;

    auto t_start_total = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < TOTAL_CYCLES; ++i) {
        double t = 10.0 + i * DT;

        // Dynamics: simulated acceleration and turn dynamics
        idr::Vector3 a_p(0.05 * std::sin(0.2 * t), 0.02 * std::cos(0.3 * t), 9.80665);
        idr::Vector3 w_p(0.001 * std::cos(0.1 * t), 0.002 * std::sin(0.1 * t), 0.15 * std::sin(0.05 * t));
        double baro_alt = 100.0 + 0.5 * std::sin(0.02 * t);
        idr::Vector3 mag_p(22.0 + std::cos(0.05 * t), 4.5 + std::sin(0.05 * t), 43.0);
        double ai_speed = 10.0 + 2.0 * std::sin(0.1 * t);

        // GNSS outage active for cycles 1000 to 90000 (vast majority in dead reckoning)
        bool gnss_active = (i < 1000 || i >= 95000);
        std::optional<idr::Vector3> g_pos = std::nullopt;
        std::optional<idr::Vector3> g_vel = std::nullopt;
        if (gnss_active) {
            g_pos = idr::Vector3(init_pos.x + 10.0 * (t - 10.0), 0.0, 100.0);
            g_vel = idr::Vector3(10.0, 0.0, 0.0);
        }

        auto t_cycle_start = std::chrono::high_resolution_clock::now();

        auto telem = engine->process_step(
            t, a_p, w_p, baro_alt,
            g_pos, gnss_active,
            ai_speed, g_vel,
            mag_p, true
        );

        auto t_cycle_end = std::chrono::high_resolution_clock::now();
        double us = std::chrono::duration<double, std::micro>(t_cycle_end - t_cycle_start).count();
        latencies_us.push_back(us);

        (void)telem; // prevent optimization
    }

    auto t_end_total = std::chrono::high_resolution_clock::now();
    g_track_allocations = false;

    double total_wall_sec = std::chrono::duration<double>(t_end_total - t_start_total).count();
    double sustained_hz = static_cast<double>(TOTAL_CYCLES) / total_wall_sec;

    size_t rss_after = get_current_rss_bytes();
    size_t peak_rss = get_peak_rss_bytes();

    // Statistical percentiles
    std::sort(latencies_us.begin(), latencies_us.end());
    double min_lat = latencies_us.front();
    double max_lat = latencies_us.back();
    double sum_lat = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0);
    double mean_lat = sum_lat / latencies_us.size();
    double p50_lat = latencies_us[static_cast<size_t>(TOTAL_CYCLES * 0.50)];
    double p95_lat = latencies_us[static_cast<size_t>(TOTAL_CYCLES * 0.95)];
    double p99_lat = latencies_us[static_cast<size_t>(TOTAL_CYCLES * 0.99)];

    double variance = 0.0;
    for (double v : latencies_us) {
        variance += (v - mean_lat) * (v - mean_lat);
    }
    double jitter_std = std::sqrt(variance / latencies_us.size());

    std::cout << "\n================ BENCHMARK RESULTS ================" << std::endl;
    std::cout << "Total Continuous Cycles:     " << TOTAL_CYCLES << std::endl;
    std::cout << "Total Wall Time:             " << total_wall_sec << " s" << std::endl;
    std::cout << "Sustained Execution Rate:    " << sustained_hz << " Hz (Requirement: >= 200 Hz)" << std::endl;
    std::cout << "Latency Metrics (per cycle):" << std::endl;
    std::cout << "  - Min Latency:             " << min_lat << " us (" << (min_lat / 1000.0) << " ms)" << std::endl;
    std::cout << "  - Median (P50):            " << p50_lat << " us (" << (p50_lat / 1000.0) << " ms)" << std::endl;
    std::cout << "  - Mean Latency:            " << mean_lat << " us (" << (mean_lat / 1000.0) << " ms)" << std::endl;
    std::cout << "  - 95th Percentile (P95):   " << p95_lat << " us (" << (p95_lat / 1000.0) << " ms)" << std::endl;
    std::cout << "  - 99th Percentile (P99):   " << p99_lat << " us (" << (p99_lat / 1000.0) << " ms)" << std::endl;
    std::cout << "  - Max Latency:             " << max_lat << " us (" << (max_lat / 1000.0) << " ms)" << std::endl;
    std::cout << "  - Latency Jitter (std):    " << jitter_std << " us" << std::endl;
    std::cout << "Memory Metrics:" << std::endl;
    std::cout << "  - Resident Set (Start):    " << (rss_before / (1024.0 * 1024.0)) << " MB" << std::endl;
    std::cout << "  - Resident Set (End):      " << (rss_after / (1024.0 * 1024.0)) << " MB" << std::endl;
    std::cout << "  - Peak Resident Set (RSS): " << (peak_rss / (1024.0 * 1024.0)) << " MB (Requirement: < 4.0 MB)" << std::endl;
    std::cout << "Dynamic Allocation Verification:" << std::endl;
    std::cout << "  - Allocations in Loop:     " << g_tactical_alloc_count << std::endl;
    std::cout << "  - Allocated Bytes in Loop: " << g_tactical_alloc_bytes << std::endl;
    std::cout << "====================================================" << std::endl;

    if (g_tactical_alloc_count == 0) {
        std::cout << "\n[AUDIT VERIFICATION] ZERO-HEAP ALLOCATION GUARANTEE: STRICTLY CONFIRMED (0 ALLOCATIONS)." << std::endl;
    } else {
        std::cerr << "\n[AUDIT VIOLATION] " << g_tactical_alloc_count << " DYNAMIC ALLOCATIONS DETECTED IN TACTICAL LOOP!" << std::endl;
        return 1;
    }

    return 0;
}

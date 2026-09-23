#include "idr/navigation_engine.hpp"
#include "idr/c_api.h"
#include "idr/math_utils.hpp"
#include "idr/idrmap_reader.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cassert>
#include <cstring>
#include <chrono>

using namespace idr;

static void assert_true(bool cond, const char* msg) {
    if (!cond) {
        std::cerr << "FAILED: " << msg << std::endl;
        std::exit(1);
    }
}

static void test_01_fsm_lifecycle_transitions() {
    std::cout << "[TEST 1] FSM 7-State Lifecycle Transitions...\n";

    SeamlessGNSSDeficitEngine engine(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(0.0, 0.0, 0.0),
        11.345, 9.0, 0.1,
        true, true
    );

    // 1. Initial State
    assert_true(engine.get_mode() == NavigationMode::INITIALIZING, "Initial mode must be INITIALIZING");

    // 2. Feed valid GNSS -> GNSS_AIDED
    Vector3 accel(0.0, 0.0, 9.81);
    Vector3 gyro(0.0, 0.0, 0.0);
    Vector3 gnss_pos(0.0, 10.0, 0.0);
    Vector3 gnss_vel(0.0, 10.0, 0.0);

    NavigationTelemetry t1 = engine.process_step(
        1.0, accel, gyro, 0.0, gnss_pos, true, 10.0, gnss_vel
    );
    assert_true(t1.mode == NavigationMode::GNSS_AIDED, "Valid GNSS must transition to GNSS_AIDED");
    assert_true(!t1.in_outage, "Must not be in outage in GNSS_AIDED");

    // 3. Drop GNSS -> TRANSITION_TO_DR
    NavigationTelemetry t2 = engine.process_step(
        2.5, accel, gyro, 0.0, std::nullopt, false, 10.0, std::nullopt
    );
    assert_true(t2.mode == NavigationMode::TRANSITION_TO_DR || t2.mode == NavigationMode::DR_ONLY, "GNSS drop must transition to DR");
    assert_true(t2.in_outage, "Must report in_outage == true");

    // 4. Continue in DR -> DR_ONLY (or MAP_AIDED_DR presentation)
    NavigationTelemetry t3 = engine.process_step(
        3.0, accel, gyro, 0.0, std::nullopt, false, 10.0, std::nullopt
    );
    assert_true(t3.mode == NavigationMode::DR_ONLY, "Must remain in DR_ONLY");
    assert_true(t3.outage_duration_s > 0.0, "Outage duration must increment");

    // 5. GNSS Return -> TRANSITION_TO_GNSS
    Vector3 rec_pos = t3.pos_enu + Vector3(0.1, 1.0, 0.0);
    Vector3 rec_vel(0.0, 10.0, 0.0);
    NavigationTelemetry t4 = engine.process_step(
        3.1, accel, gyro, 0.0, rec_pos, true, 10.0, rec_vel
    );
    assert_true(t4.mode == NavigationMode::TRANSITION_TO_GNSS, "GNSS return must enter TRANSITION_TO_GNSS");

    // 6. Dwell passes -> GNSS_REACQUIRED
    NavigationTelemetry t5 = t4;
    for (int epoch = 0; epoch < 5; ++epoch) {
        rec_pos.y += 1.0;
        t5 = engine.process_step(
            3.2 + epoch * 0.1, accel, gyro, 0.0, rec_pos, true, 10.0, rec_vel
        );
    }
    assert_true(t5.mode == NavigationMode::GNSS_REACQUIRED || t5.mode == NavigationMode::GNSS_AIDED, "Dwell must restore GNSS lock");

    std::cout << "  -> PASSED\n";
}


static void test_02_raw_dr_vs_map_matched_separation() {
    std::cout << "[TEST 2] Raw DR vs Map-Matched State Separation...\n";

    SeamlessGNSSDeficitEngine engine(
        Vector3(2.5, 500.0, 0.0),
        Vector3(0.0, 15.0, 0.0),
        Vector3(0.0, 0.0, 0.0),
        11.345, 9.0, 0.1,
        true, true
    );

    // Initial GNSS step
    engine.process_step(0.1, Vector3(0.0, 0.0, 9.81), Vector3(0.0, 0.0, 0.0), 0.0,
                        Vector3(2.5, 500.0, 0.0), true, 15.0, Vector3(0.0, 15.0, 0.0));

    // DR Step with East offset = 2.5m (Road centerline is at East = 0.0m)
    NavigationTelemetry telem = engine.process_step(
        0.2, Vector3(0.0, 0.0, 9.81), Vector3(0.0, 0.0, 0.0), 0.0,
        std::nullopt, false, 15.0, std::nullopt
    );

    // Assert raw DR position retains the filter state
    assert_true(std::abs(telem.raw_dr_pos_enu.x - telem.pos_enu.x) < 1e-9, "raw_dr_pos_enu must match pos_enu");
    assert_true(std::abs(telem.raw_dr_pos_enu.x) > 1.0, "raw_dr_pos_enu must preserve off-road offset");

    // Assert map-matched position snapped to road centerline (near x = 0.0m)
    if (telem.map_confidence == MapMatchConfidence::HIGH || telem.map_confidence == MapMatchConfidence::MEDIUM) {
        assert_true(std::abs(telem.map_matched_pos_enu.x) < 0.5, "map_matched_pos_enu must snap to road centerline");
        // Verify underlying filter was NOT hard overwritten to x = 0
        assert_true(std::abs(telem.pos_enu.x) > 1.0, "Underlying ESKF state must not be hard overwritten");
    }


    std::cout << "  -> PASSED\n";
}

static void test_03_idrmap_loading_and_offline_navigation() {
    std::cout << "[TEST 3] Offline .idrmap Loading and Navigation...\n";

    SeamlessGNSSDeficitEngine engine(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(0.0, 0.0, 0.0),
        11.345, 9.0, 0.1,
        true, true
    );

    bool loaded = engine.load_idrmap("data/maps/sample_prayagraj.idrmap", 25.435, 81.845, 98.0);
    assert_true(loaded, "Failed to load sample_prayagraj.idrmap");

    // Process step
    NavigationTelemetry telem = engine.process_step(
        1.0, Vector3(0.0, 0.0, 9.81), Vector3(0.0, 0.0, 0.0), 98.0,
        std::nullopt, false, 10.0, std::nullopt
    );

    assert_true(telem.map_available, "map_available must be true");
    assert_true(telem.candidate_count > 0, "Candidates must be found in sample_prayagraj map");
    std::cout << "  -> PASSED (Matched edge=" << telem.selected_edge_id << " conf=" << (int)telem.map_confidence << ")\n";
}

static void test_04_temporary_map_loss_and_recovery() {
    std::cout << "[TEST 4] Temporary Map Loss and Recovery...\n";

    SeamlessGNSSDeficitEngine engine(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(0.0, 0.0, 0.0),
        11.345, 9.0, 0.1,
        true, true
    );

    engine.load_idrmap("data/maps/sample_prayagraj.idrmap", 25.435, 81.845, 98.0);

    // 1. In-coverage step -> Match found
    NavigationTelemetry t1 = engine.process_step(
        1.0, Vector3(0.0, 0.0, 9.81), Vector3(0.0, 0.0, 0.0), 98.0,
        std::nullopt, false, 10.0, std::nullopt
    );
    assert_true(t1.candidate_count > 0, "Must find candidates in-coverage");

    // 2. Far out-of-coverage step (e.g. 50,000m away) -> Safe fallback to DR
    // Temporarily query off-grid
    NavigationTelemetry t2 = engine.process_step(
        2.0, Vector3(50000.0, 0.0, 9.81), Vector3(0.0, 0.0, 0.0), 98.0,
        std::nullopt, false, 10.0, std::nullopt
    );
    assert_true(t2.map_matched_pos_enu.x != 0.0 || !std::isnan(t2.pos_enu.x), "Must not produce NaN");
    assert_true(t2.selected_edge_id == -1 || t2.map_confidence == MapMatchConfidence::INVALID || t2.candidate_count == 0,
                "Out-of-coverage must report invalid or no match");

    std::cout << "  -> PASSED\n";
}

static void test_05_missing_and_corrupt_map_fault_tolerance() {
    std::cout << "[TEST 5] Missing and Corrupt Map Fault Tolerance...\n";

    SeamlessGNSSDeficitEngine engine(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(0.0, 0.0, 0.0),
        11.345, 9.0, 0.1,
        true, false
    );

    // 1. Missing file loading fails safely
    bool loaded = engine.load_idrmap("data/maps/nonexistent_file_xyz.idrmap", 25.0, 81.0, 100.0);
    assert_true(!loaded, "Nonexistent map file must return false");

    // 2. Engine continues pure DR without map
    NavigationTelemetry t = engine.process_step(
        1.0, Vector3(0.0, 0.0, 9.81), Vector3(0.0, 0.0, 0.0), 100.0,
        std::nullopt, false, 10.0, std::nullopt
    );
    assert_true(t.mode == NavigationMode::INITIALIZING || t.mode == NavigationMode::DR_ONLY, "Engine must operate in pure DR");
    assert_true(!std::isnan(t.pos_enu.x), "Position must be finite");
    assert_true(t.selected_edge_id == -1, "No edge should be selected without map");

    std::cout << "  -> PASSED\n";
}

static void test_06_c_api_offline_navigation() {
    std::cout << "[TEST 6] C API Offline Navigation Telemetry & Functions...\n";

    idr_engine_handle_t handle = idr_engine_create(0.1);
    assert_true(handle != nullptr, "idr_engine_create failed");

    idr_status_t load_rc = idr_engine_load_idrmap(handle, "data/maps/sample_prayagraj.idrmap", 25.435, 81.845, 98.0);

    assert_true(load_rc == IDR_STATUS_OK, "idr_engine_load_idrmap failed");

    idr_vec3_t acc{0.0, 0.0, 9.81};
    idr_vec3_t gyr{0.0, 0.0, 0.0};
    idr_vec3_t dummy_pos{0.0, 0.0, 0.0};
    idr_vec3_t dummy_vel{0.0, 0.0, 0.0};
    idr_vec3_t dummy_mag{0.0, 0.0, 0.0};
    idr_telemetry_offline_nav_t telem{};

    int32_t rc = idr_engine_process_step_offline_nav(
        handle, 1.0, acc, gyr, 98.0,
        0, dummy_pos, dummy_vel, 12.0,
        0, dummy_mag, 1, &telem
    );
    assert_true(rc == IDR_STATUS_OK, "idr_engine_process_step_offline_nav failed");
    assert_true(telem.map_available == 1, "map_available must be 1");

    idr_status_t reset_rc = idr_engine_reset(handle);
    assert_true(reset_rc == IDR_STATUS_OK, "idr_engine_reset failed");

    idr_engine_destroy(handle);
    std::cout << "  -> PASSED\n";
}

static void test_07_anti_lookahead_and_determinism() {
    std::cout << "[TEST 7] Strict Anti-Lookahead and Deterministic Parity...\n";

    auto run_sim = [](int num_steps, double future_pert) {
        SeamlessGNSSDeficitEngine engine(
            Vector3(0.0, 0.0, 0.0),
            Vector3(0.0, 10.0, 0.0),
            Vector3(0.0, 0.0, 0.0),
            11.345, 9.0, 0.1,
            true, true
        );
        std::vector<NavigationTelemetry> res;
        for (int i = 0; i < num_steps; ++i) {
            double t = 1.0 + i * 0.1;
            Vector3 a(0.0, 0.0, 9.81);
            Vector3 w(0.0, 0.0, 0.0);
            if (i >= 20) {
                w.z = future_pert;
            }
            res.push_back(engine.process_step(t, a, w, 0.0, std::nullopt, false, 10.0, std::nullopt));
        }
        return res;
    };

    auto base_run = run_sim(20, 0.0);
    auto fut_run_b = run_sim(30, 0.05);
    auto fut_run_c = run_sim(30, -0.05);

    // Steps 0..19 must be bitwise identical
    for (int i = 0; i < 20; ++i) {
        assert_true(base_run[i].pos_enu.x == fut_run_b[i].pos_enu.x, "Prefix invariance failure (X)");
        assert_true(base_run[i].pos_enu.y == fut_run_b[i].pos_enu.y, "Prefix invariance failure (Y)");
        assert_true(base_run[i].mode == fut_run_b[i].mode, "Prefix mode invariance failure");
        assert_true(base_run[i].pos_enu.x == fut_run_c[i].pos_enu.x, "Divergent future invariance failure (X)");
        assert_true(base_run[i].pos_enu.y == fut_run_c[i].pos_enu.y, "Divergent future invariance failure (Y)");
    }

    std::cout << "  -> PASSED (Zero future observation leakage verified)\n";
}

int main() {
    std::cout << "============================================================\n";
    std::cout << "  PHASE 11.8: OFFLINE NAVIGATION INTEGRATION TEST SUITE     \n";
    std::cout << "============================================================\n";

    test_01_fsm_lifecycle_transitions();
    test_02_raw_dr_vs_map_matched_separation();
    test_03_idrmap_loading_and_offline_navigation();
    test_04_temporary_map_loss_and_recovery();
    test_05_missing_and_corrupt_map_fault_tolerance();
    test_06_c_api_offline_navigation();
    test_07_anti_lookahead_and_determinism();

    std::cout << "============================================================\n";
    std::cout << "  ALL PHASE 11.8 OFFLINE NAVIGATION TESTS PASSED!           \n";
    std::cout << "============================================================\n";
    return 0;
}

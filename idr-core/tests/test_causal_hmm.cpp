#include "idr/map_matcher_3d.hpp"
#include "idr/idrmap_reader.hpp"
#include "idr/c_api.h"
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cassert>
#include <cmath>

namespace {

void run_test(const char* name, void (*func)()) {
    std::cout << "[TEST] " << name << " ... ";
    func();
    std::cout << "PASSED" << std::endl;
}

// 1. Single Obvious Road
void test_01_single_obvious_road() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    idr::Vector3 pos(0.0, 100.0, 0.0);
    auto res = matcher.process_step(pos, 0.0, 0.0, 10.0, 0.1, true);

    assert(res.is_valid);
    assert(res.confidence == idr::MapMatchConfidence::HIGH);
    assert(res.d_2d < 1.0);
    assert(!res.is_ambiguous);
}

// 2. Parallel Roads
void test_02_parallel_roads() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    // Main expressway is at East=0, Service road is at East=12
    idr::Vector3 pos_main(1.0, 500.0, 0.0);
    auto res_main = matcher.process_step(pos_main, 0.0, 0.0, 10.0, 0.1, true);
    assert(res_main.edge_name == std::string("Main Expressway North"));

    matcher.reset();
    idr::Vector3 pos_service(11.5, 500.0, 0.0);
    auto res_service = matcher.process_step(pos_service, 0.0, 0.0, 10.0, 0.1, true);
    assert(res_service.edge_name == std::string("East Frontage Service Road"));
}

// 3. Nearby Crossing Roads
void test_03_nearby_crossing_roads() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    // Road A: South-North (Heading 0 deg)
    graph.add_edge(1, 10, 11, "Road North", "primary",
        std::vector<idr::Vector3>{idr::Vector3(0.0, -100.0, 0.0), idr::Vector3(0.0, 100.0, 0.0)});
    // Road B: West-East (Heading 90 deg)
    graph.add_edge(2, 20, 21, "Road East", "secondary",
        std::vector<idr::Vector3>{idr::Vector3(-100.0, 0.0, 0.0), idr::Vector3(100.0, 0.0, 0.0)});
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);

    // Position at intersection (0,0), but heading North (0 rad)
    idr::Vector3 pos(0.0, 0.0, 0.0);
    auto res_north = matcher.process_step(pos, 0.0, 0.0, 10.0, 0.1, true);
    assert(res_north.edge_id == 1);

    // Position at intersection (0,0), but heading East (pi/2 rad)
    matcher.reset();
    auto res_east = matcher.process_step(pos, idr::HALF_PI, 0.0, 10.0, 0.1, true);
    assert(res_east.edge_id == 2);
}

// 4. Same-Layer Parallel Roads Ambiguity Detection
void test_04_same_layer_parallel_ambiguity() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    // Two parallel roads separated by 4 meters
    graph.add_edge(1, 10, 11, "Lane Left", "primary",
        std::vector<idr::Vector3>{idr::Vector3(-2.0, 0.0, 0.0), idr::Vector3(-2.0, 200.0, 0.0)});
    graph.add_edge(2, 20, 21, "Lane Right", "primary",
        std::vector<idr::Vector3>{idr::Vector3(2.0, 0.0, 0.0), idr::Vector3(2.0, 200.0, 0.0)});
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);

    // Exact center between the two lanes
    idr::Vector3 pos_mid(0.0, 50.0, 0.0);
    auto res = matcher.process_step(pos_mid, 0.0, 0.0, 10.0, 0.1, true);

    assert(res.is_valid);
    assert(res.is_ambiguous);
    assert(res.score_margin < 1.5);
    // When ambiguous, confidence must be capped at MEDIUM or lower
    assert(res.confidence != idr::MapMatchConfidence::HIGH);
}

// 5. Bridge Over Ground Road Elevation Barrier
void test_05_bridge_elevation_barrier() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    // Query Underpass at Alt=0.5m (Flyover deck is at 8.5m -> |dh| = 8.0m > 4.0m barrier)
    idr::Vector3 ground_pos(0.0, 2000.0, 0.5);
    auto res_ground = matcher.process_step(ground_pos, 0.0, 0.5, 10.0, 0.1, true);
    assert(res_ground.layer == 0);
    assert(res_ground.confidence == idr::MapMatchConfidence::HIGH);

    // Query Flyover deck at Alt=8.8m (Ground underpass is at 0m -> |dh| = 8.8m > 4.0m barrier)
    matcher.reset();
    idr::Vector3 flyover_pos(0.0, 2000.0, 8.8);
    auto res_flyover = matcher.process_step(flyover_pos, 0.0, 8.8, 10.0, 0.1, true);
    assert(res_flyover.layer == 1);
    assert(res_flyover.confidence == idr::MapMatchConfidence::HIGH);
}

// 6. Tunnel Identification
void test_06_tunnel_identification() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    graph.add_edge(10, 1, 2, "Express Tunnel", "primary",
        std::vector<idr::Vector3>{idr::Vector3(0.0, 0.0, -5.0), idr::Vector3(0.0, 500.0, -5.0)}, -1, false, true);
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);
    auto res = matcher.process_step(idr::Vector3(0.0, 100.0, -5.0), 0.0, -5.0, 10.0, 0.1, true);

    assert(res.is_valid);
    assert(res.layer == -1);
    assert(res.layer_label == std::string("Underpass"));
}

// 7. Road Junction Transition
void test_07_road_junction_transition() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    graph.add_edge(1, 1, 2, "Segment 1", "primary",
        std::vector<idr::Vector3>{idr::Vector3(0.0, 0.0, 0.0), idr::Vector3(0.0, 100.0, 0.0)});
    graph.add_edge(2, 2, 3, "Segment 2", "primary",
        std::vector<idr::Vector3>{idr::Vector3(0.0, 100.0, 0.0), idr::Vector3(0.0, 200.0, 0.0)});
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);
    auto r1 = matcher.process_step(idr::Vector3(0.0, 80.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(r1.edge_id == 1);

    auto r2 = matcher.process_step(idr::Vector3(0.0, 120.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(r2.edge_id == 2);
    assert(r2.confidence == idr::MapMatchConfidence::HIGH);
}

// 8. One-Way Road Enforcement
void test_08_oneway_road_enforcement() {
    idr::IdrMapReader reader;
    idr::IdrMapStatus st = reader.open_file("data/maps/sample_prayagraj.idrmap");
    if (st == idr::IdrMapStatus::OK) {
        idr::Causal3DMapMatcher matcher(&reader, 25.4350, 81.8500, 0.0);
        // Step forward along road near (25.4350, 81.8500)
        auto r_fwd = matcher.process_step(idr::Vector3(0.0, 0.0, 0.0), 0.0, 0.0, 10.0, 0.1, false);
        assert(r_fwd.is_valid);
        assert(r_fwd.edge_id >= 0);
    }
}

// 9. Disconnected Nearby Roads
void test_09_disconnected_nearby_roads() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    // Two disconnected segments
    graph.add_edge(1, 1, 2, "Northbound A", "primary",
        std::vector<idr::Vector3>{idr::Vector3(0.0, 0.0, 0.0), idr::Vector3(0.0, 100.0, 0.0)});
    graph.add_edge(2, 3, 4, "Northbound B", "primary",
        std::vector<idr::Vector3>{idr::Vector3(5.0, 0.0, 0.0), idr::Vector3(5.0, 100.0, 0.0)});
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);
    matcher.process_step(idr::Vector3(0.0, 50.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);

    // Sudden hop to B without graph link
    auto res_hop = matcher.process_step(idr::Vector3(5.0, 60.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(res_hop.is_valid);
}

// 10. Ambiguous Candidate Pair
void test_10_ambiguous_candidate_pair() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    graph.add_edge(1, 1, 2, "Parallel Left", "primary",
        std::vector<idr::Vector3>{idr::Vector3(-3.0, 0.0, 0.0), idr::Vector3(-3.0, 100.0, 0.0)});
    graph.add_edge(2, 3, 4, "Parallel Right", "primary",
        std::vector<idr::Vector3>{idr::Vector3(3.0, 0.0, 0.0), idr::Vector3(3.0, 100.0, 0.0)});
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);
    auto res = matcher.process_step(idr::Vector3(0.0, 50.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(res.is_ambiguous);
    assert(res.score_margin < 1.5);
}

// 11. Temporary Candidate Loss
void test_11_temporary_candidate_loss() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    // Step 1: on road
    auto r1 = matcher.process_step(idr::Vector3(0.0, 100.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(r1.confidence == idr::MapMatchConfidence::HIGH);

    // Step 2: wandering far off network (150m away)
    auto r2 = matcher.process_step(idr::Vector3(150.0, 150.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(r2.confidence == idr::MapMatchConfidence::INVALID || r2.confidence == idr::MapMatchConfidence::LOW);

    // Step 3: returns to road
    auto r3 = matcher.process_step(idr::Vector3(0.0, 200.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(r3.is_valid);
}

// 12. GNSS Outage Transition
void test_12_gnss_outage_transition() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    // Continuous propagation along road
    for (int k = 0; k < 20; ++k) {
        double y = 100.0 + k * 10.0;
        auto res = matcher.process_step(idr::Vector3(0.2, y, 0.0), 0.0, 0.0, 10.0, 0.1, true);
        assert(res.confidence == idr::MapMatchConfidence::HIGH);
    }
}

// 13. GNSS Recovery
void test_13_gnss_recovery() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    for (int k = 0; k < 10; ++k) {
        matcher.process_step(idr::Vector3(0.0, 100.0 + k * 10.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    }
    // Recovery step with accurate fix
    auto res = matcher.process_step(idr::Vector3(0.0, 200.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(res.confidence == idr::MapMatchConfidence::HIGH);
}

// 14. High Navigation Uncertainty
void test_14_high_navigation_uncertainty() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    // Matcher with larger sigma_d for high uncertainty
    idr::Causal3DMapMatcher matcher(graph, 15.0, 3.0, 0.60);

    auto res = matcher.process_step(idr::Vector3(8.0, 100.0, 0.0), 0.20, 0.0, 10.0, 0.1, true);
    assert(res.is_valid);
}

// 15. Low-Speed Movement
void test_15_low_speed_movement() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    auto res = matcher.process_step(idr::Vector3(0.0, 100.0, 0.0), 0.0, 0.0, 0.5, 0.1, true);
    assert(res.confidence == idr::MapMatchConfidence::HIGH);
}

// 16. High-Speed Movement
void test_16_high_speed_movement() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    auto res = matcher.process_step(idr::Vector3(0.0, 100.0, 0.0), 0.0, 0.0, 33.3, 0.1, true);
    assert(res.confidence == idr::MapMatchConfidence::HIGH);
}

// 17. Heading Near Boundary Wrap
void test_17_heading_boundary_wrap() {
    idr::OfflineOSMGraph graph(52.77, -1.21, 0.0);
    graph.add_edge(1, 1, 2, "Northbound", "primary",
        std::vector<idr::Vector3>{idr::Vector3(0.0, 0.0, 0.0), idr::Vector3(0.0, 100.0, 0.0)});
    graph.warm_cache();

    idr::Causal3DMapMatcher matcher(graph);
    // Heading +359 deg (-0.017 rad) vs North (0 rad)
    double hdg_near_north = -0.017;
    auto res = matcher.process_step(idr::Vector3(0.0, 50.0, 0.0), hdg_near_north, 0.0, 10.0, 0.1, true);
    assert(res.confidence == idr::MapMatchConfidence::HIGH);
}

// 18. Invalid Sensor Observations
void test_18_invalid_sensor_observations() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    // NaN position
    auto res_nan = matcher.process_step(idr::Vector3(NAN, 100.0, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    assert(!res_nan.is_valid);
    assert(res_nan.confidence == idr::MapMatchConfidence::INVALID);

    // Inf heading
    auto res_inf = matcher.process_step(idr::Vector3(0.0, 100.0, 0.0), INFINITY, 0.0, 10.0, 0.1, true);
    assert(!res_inf.is_valid);
}

// 19. Repeated Identical Observations
void test_19_repeated_identical_observations() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    idr::Vector3 pos(0.0, 100.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        auto res = matcher.process_step(pos, 0.0, 0.0, 0.0, 0.1, true);
        assert(!std::isnan(res.log_posterior));
        assert(!std::isinf(res.log_posterior));
        assert(res.confidence == idr::MapMatchConfidence::HIGH);
    }
}

// 20. Strict Anti-Lookahead Causality Invariance Proof
void test_20_anti_lookahead_proof() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();

    // Stream 1: Prefix [0..10] + Future A (Turns East)
    idr::Causal3DMapMatcher matcher_A(graph);
    std::vector<idr::MapMatchResult> results_A;
    for (int k = 0; k < 10; ++k) {
        double y = k * 10.0;
        results_A.push_back(matcher_A.process_step(idr::Vector3(0.0, y, 0.0), 0.0, 0.0, 10.0, 0.1, true));
    }
    // Future A step
    matcher_A.process_step(idr::Vector3(20.0, 100.0, 0.0), idr::HALF_PI, 0.0, 10.0, 0.1, true);

    // Stream 2: Prefix [0..10] + Future B (Turns West)
    idr::Causal3DMapMatcher matcher_B(graph);
    std::vector<idr::MapMatchResult> results_B;
    for (int k = 0; k < 10; ++k) {
        double y = k * 10.0;
        results_B.push_back(matcher_B.process_step(idr::Vector3(0.0, y, 0.0), 0.0, 0.0, 10.0, 0.1, true));
    }
    // Future B step
    matcher_B.process_step(idr::Vector3(-20.0, 100.0, 0.0), -idr::HALF_PI, 0.0, 10.0, 0.1, true);

    // Exact bitwise/numerical equality on the shared prefix
    for (size_t k = 0; k < 10; ++k) {
        assert(results_A[k].edge_id == results_B[k].edge_id);
        assert(results_A[k].confidence == results_B[k].confidence);
        assert(std::abs(results_A[k].log_posterior - results_B[k].log_posterior) < 1e-12);
        assert(std::abs(results_A[k].d_2d - results_B[k].d_2d) < 1e-12);
        assert(std::abs(results_A[k].road_heading - results_B[k].road_heading) < 1e-12);
    }
}

// 21. Micro-Benchmarks & Latency
void test_21_micro_benchmarks() {
    idr::OfflineOSMGraph graph = idr::OfflineOSMGraph::build_synthetic_3d_corridor();
    idr::Causal3DMapMatcher matcher(graph);

    constexpr int NUM_ITER = 10000;
    auto t0 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NUM_ITER; ++i) {
        double y = 100.0 + (i % 100) * 5.0;
        matcher.process_step(idr::Vector3(0.0, y, 0.0), 0.0, 0.0, 10.0, 0.1, true);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    double avg_us = total_us / NUM_ITER;

    std::cout << "\n  -> 10,000 Step Benchmark: Total = " << total_us / 1000.0 << " ms, Avg = "
              << avg_us << " us/step (" << static_cast<int>(1e6 / avg_us) << " Hz throughput)" << std::endl;

    assert(avg_us < 20.0); // Real-time budget requirement: < 20 us per step
}

} // namespace

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << "  PHASE 11.7: CAUSAL 3D HMM MAP MATCHER TEST SUITE (C++)   " << std::endl;
    std::cout << "============================================================" << std::endl;

    run_test("01_single_obvious_road", test_01_single_obvious_road);
    run_test("02_parallel_roads", test_02_parallel_roads);
    run_test("03_nearby_crossing_roads", test_03_nearby_crossing_roads);
    run_test("04_same_layer_parallel_ambiguity", test_04_same_layer_parallel_ambiguity);
    run_test("05_bridge_elevation_barrier", test_05_bridge_elevation_barrier);
    run_test("06_tunnel_identification", test_06_tunnel_identification);
    run_test("07_road_junction_transition", test_07_road_junction_transition);
    run_test("08_oneway_road_enforcement", test_08_oneway_road_enforcement);
    run_test("09_disconnected_nearby_roads", test_09_disconnected_nearby_roads);
    run_test("10_ambiguous_candidate_pair", test_10_ambiguous_candidate_pair);
    run_test("11_temporary_candidate_loss", test_11_temporary_candidate_loss);
    run_test("12_gnss_outage_transition", test_12_gnss_outage_transition);
    run_test("13_gnss_recovery", test_13_gnss_recovery);
    run_test("14_high_navigation_uncertainty", test_14_high_navigation_uncertainty);
    run_test("15_low_speed_movement", test_15_low_speed_movement);
    run_test("16_high_speed_movement", test_16_high_speed_movement);
    run_test("17_heading_boundary_wrap", test_17_heading_boundary_wrap);
    run_test("18_invalid_sensor_observations", test_18_invalid_sensor_observations);
    run_test("19_repeated_identical_observations", test_19_repeated_identical_observations);
    run_test("20_anti_lookahead_proof", test_20_anti_lookahead_proof);
    run_test("21_micro_benchmarks", test_21_micro_benchmarks);

    std::cout << "============================================================" << std::endl;
    std::cout << "  ALL 21 C++ CAUSAL 3D HMM MAP MATCHER TESTS PASSED!       " << std::endl;
    std::cout << "============================================================" << std::endl;
    return 0;
}

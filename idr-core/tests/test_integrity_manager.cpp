#include "idr/integrity_manager.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace idr;

void test_channel_nis() {
    ChannelNISStats ch{"test_channel", 1, 9.0};
    assert(ch.current_nis == 0.0);
    assert(ch.consecutive_rejections == 0);

    // Accept below threshold
    bool acc1 = ch.update(4.5);
    assert(acc1 == true);
    assert(ch.consecutive_rejections == 0);
    assert(ch.acceptance_count == 1);
    assert(ch.rejection_count == 0);

    // Reject above threshold
    bool acc2 = ch.update(12.0);
    assert(acc2 == false);
    assert(ch.consecutive_rejections == 1);
    assert(ch.rejection_count == 1);

    // Reject second time
    bool acc3 = ch.update(15.0);
    assert(acc3 == false);
    assert(ch.consecutive_rejections == 2);
    assert(ch.rejection_count == 2);

    // Accept again -> reset consecutive rejections
    bool acc4 = ch.update(2.0);
    assert(acc4 == true);
    assert(ch.consecutive_rejections == 0);
    assert(ch.rejection_count == 2);

    std::cout << "[PASS] test_channel_nis" << std::endl;
}

void test_continuous_map_confidence() {
    // 1. Ideal candidate (d_2d=0, dh=0, d_psi=0, high margin, low yaw variance, 5 matches)
    double conf_ideal = IntegrityManager::compute_continuous_map_confidence(
        0.0, 0.0, 0.0, 5.0, 0.001, 5, 4.0, 5.0, 0.30
    );
    assert(conf_ideal > 0.85);

    // 2. 3D Elevation barrier breach (dh = 4.5m > 4.0m)
    double conf_barrier = IntegrityManager::compute_continuous_map_confidence(
        0.0, 4.5, 0.0, 5.0, 0.001, 5, 4.0, 5.0, 0.30
    );
    assert(conf_barrier == 0.0);

    // 3. Ambiguous candidate (margin = 0.0)
    double conf_ambig = IntegrityManager::compute_continuous_map_confidence(
        2.0, 0.5, 0.1, 0.0, 0.001, 5, 4.0, 5.0, 0.30
    );
    assert(conf_ambig < conf_ideal);

    // 4. Distant candidate (d_2d = 25m)
    double conf_dist = IntegrityManager::compute_continuous_map_confidence(
        25.0, 0.5, 0.1, 3.0, 0.001, 5, 4.0, 5.0, 0.30
    );
    assert(conf_dist < 0.35);

    std::cout << "[PASS] test_continuous_map_confidence" << std::endl;
}

void test_integrity_state_transitions() {
    IntegrityManager mgr;
    assert(mgr.get_state() == IntegrityState::HEALTHY);

    Matrix15x15 P = Matrix15x15::identity();
    P(0, 0) = 2.0; P(1, 1) = 2.0; P(2, 2) = 4.0; // pos trace = 8.0
    P(8, 8) = 0.001; // nominal yaw variance (~1.8 deg)^2

    // Nominal step with GNSS
    IntegrityReport rep1 = mgr.evaluate_step(
        1.0, false, true, P, true, 1.0, 0.2, 0.0, 4.0, 5,
        std::nullopt, std::nullopt, 2.5, 1.0, std::nullopt, std::nullopt, 0.5, 0.0
    );
    assert(rep1.state == IntegrityState::HEALTHY);
    assert(rep1.is_map_usable == true);

    // Outage step with good map
    IntegrityReport rep2 = mgr.evaluate_step(
        2.0, true, false, P, true, 1.5, 0.1, 0.0, 3.5, 6,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, 1.2, 2.0, 0.6, 0.0
    );
    assert(rep2.state == IntegrityState::DEGRADED);

    // Outage step with bad map (elevation barrier breach)
    IntegrityReport rep3 = mgr.evaluate_step(
        3.0, true, false, P, true, 2.0, 5.0, 0.0, 0.0, 1,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, 0.6, 0.0
    );
    assert(rep3.state == IntegrityState::MAP_UNCERTAIN);
    assert(rep3.is_map_usable == false);

    // Huge covariance divergence -> FAULT
    Matrix15x15 P_huge = Matrix15x15::identity() * 3000.0;
    IntegrityReport rep4 = mgr.evaluate_step(
        4.0, true, false, P_huge, false, 999.0, 999.0, 0.0, 0.0, 0,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, 0.6, 0.0
    );
    assert(rep4.state == IntegrityState::FAULT);

    std::cout << "[PASS] test_integrity_state_transitions" << std::endl;
}

int main() {
    std::cout << "Running IntegrityManager unit tests..." << std::endl;
    test_channel_nis();
    test_continuous_map_confidence();
    test_integrity_state_transitions();
    std::cout << "All IntegrityManager tests PASSED!" << std::endl;
    return 0;
}

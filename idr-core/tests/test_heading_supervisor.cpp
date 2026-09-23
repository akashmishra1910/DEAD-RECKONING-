#include "idr/heading_supervisor.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace idr;

int main() {
    std::cout << "[TEST] Starting Heading Supervisor Tests..." << std::endl;

    HeadingSupervisor sup(6.635, 2.5, 48.0, true, true);
    ErrorStateKalmanFilter eskf;

    // Test 1: GNSS course evaluation above threshold
    Vector3 gnss_vel(0.0, 10.0, 0.0); // 0 deg azimuth (North), matching initial yaw 0.0
    Matrix3x3 R_p2b = Matrix3x3::identity();

    auto telem1 = sup.process_step(
        1.0, eskf, R_p2b, gnss_vel, true, std::nullopt, 0.0, true
    );

    std::cout << "  GNSS Course filter heading: " << telem1.filter_heading_deg << std::endl;
    std::cout << "  Selected source: " << heading_source_to_string(telem1.selected_source) << std::endl;
    assert(telem1.selected_source == HeadingSourceType::GNSS_COURSE);
    assert(telem1.is_updated == true);
    assert(telem1.total_updates_count == 1);

    // Test 2: Magnetic disturbance rejection (high gradient or delta > 12 uT)
    Vector3 disturbed_mag(100.0, 100.0, 100.0); // > 173 uT, massive anomaly
    auto telem2 = sup.process_step(
        1.1, eskf, R_p2b, std::nullopt, false, disturbed_mag, 0.0, true
    );
    assert(telem2.mag_disturbed == true);
    assert(telem2.is_updated == false);
    std::cout << "  Magnetic disturbance correctly rejected: " << telem2.rejection_reason << std::endl;

    // Test 3: Normal tilt-compensated magnetometer when GNSS is denied
    // Level facing North (psi=0), field: [20, 0, -43] (norm=47.4 uT, ref=48 uT)
    sup.reset();
    Vector3 clean_mag(20.0, 0.0, -43.0);
    auto telem3 = sup.process_step(
        1.2, eskf, R_p2b, std::nullopt, false, clean_mag, 0.0, true
    );
    std::cout << "  Magnetometer update under GNSS denial: is_updated=" << telem3.is_updated
              << " source=" << heading_source_to_string(telem3.selected_source) << std::endl;
    assert(telem3.selected_source == HeadingSourceType::MAGNETOMETER);
    assert(telem3.is_updated == true);

    std::cout << "[TEST] Heading Supervisor Tests PASSED!" << std::endl;
    return 0;
}

#include "idr/navigation_engine.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace idr;

int main() {
    std::cout << "[TEST] Starting Complete 6-State FSM Navigation Engine Tests..." << std::endl;

    Vector3 init_pos(0.0, 0.0, 0.0);
    Vector3 init_vel(0.0, 10.0, 0.0); // 10 m/s North
    Vector3 init_euler(0.0, 0.0, 0.0);

    SeamlessGNSSDeficitEngine engine(init_pos, init_vel, init_euler, 11.345, 9.0, 0.1);

    // Initial state: INITIALIZING
    assert(engine.get_mode() == NavigationMode::INITIALIZING);

    // Step 1: Initial GNSS lock -> transitions to GNSS_AIDED
    Vector3 accel_phone(0.0, 0.0, 9.80665);
    Vector3 gyro_phone(0.0, 0.0, 0.0);
    Vector3 gnss_pos(0.0, 1.0, 0.0);
    Vector3 gnss_vel(0.0, 10.0, 0.0);

    auto t1 = engine.process_step(
        0.1, accel_phone, gyro_phone, 0.0, gnss_pos, true, 10.0, gnss_vel, std::nullopt, true
    );

    std::cout << "  After initial lock mode: " << navigation_mode_to_string(t1.mode) << std::endl;
    assert(t1.mode == NavigationMode::GNSS_AIDED);
    assert(t1.in_outage == false);

    // Step 2: Simulate GNSS outage onset -> transitions to DR_ONLY
    auto t2 = engine.process_step(
        0.2, accel_phone, gyro_phone, 0.0, std::nullopt, false, 10.0, std::nullopt, std::nullopt, true
    );

    std::cout << "  After GNSS loss mode: " << navigation_mode_to_string(t2.mode) << std::endl;
    assert(t2.mode == NavigationMode::DR_ONLY);
    assert(t2.in_outage == true);
    assert(t2.outage_duration_s > 0.0 || std::abs(t2.outage_duration_s) < 1e-4);

    // Step 3: Run 20 cycles in pure DR mode
    for (int i = 0; i < 20; ++i) {
        double time = 0.3 + i * 0.1;
        t2 = engine.process_step(
            time, accel_phone, gyro_phone, 0.0, std::nullopt, false, 10.0, std::nullopt, std::nullopt, true
        );
    }
    assert(t2.mode == NavigationMode::DR_ONLY);
    assert(t2.outage_duration_s > 1.5);
    std::cout << "  DR Mode survived 20 cycles. Outage distance traveled: " << t2.outage_dist_traveled_m << " m" << std::endl;

    // Step 4: GNSS returns with clean fix -> enters TRANSITION_TO_GNSS
    Vector3 gnss_return_pos = t2.pos_enu + Vector3(0.1, 0.1, 0.0);
    Vector3 gnss_return_vel(0.0, 10.0, 0.0);
    auto t3 = engine.process_step(
        2.3, accel_phone, gyro_phone, 0.0, gnss_return_pos, true, 10.0, gnss_return_vel, std::nullopt, true
    );
    std::cout << "  GNSS return mode: " << navigation_mode_to_string(t3.mode) << std::endl;
    assert(t3.mode == NavigationMode::TRANSITION_TO_GNSS);

    std::cout << "[TEST] 6-State FSM Navigation Engine Tests PASSED!" << std::endl;
    return 0;
}

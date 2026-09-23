#include "idr/eskf.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <cassert>

using namespace idr;

int main() {
    std::cout << "[TEST] Starting ESKF 15-State Filter Golden Parity Tests..." << std::endl;

    Vector3 init_pos(100.0, 200.0, 50.0);
    Vector3 init_vel(12.0, 0.5, -0.1);
    Vector3 init_euler(2.0 * DEG_TO_RAD, -1.5 * DEG_TO_RAD, 45.0 * DEG_TO_RAD);
    Vector3 init_ba(0.02, -0.01, 0.03);
    Vector3 init_bg(0.001, -0.0005, 0.002);

    ErrorStateKalmanFilter eskf(
        init_pos, init_vel, init_euler, init_ba, init_bg,
        0.15, 0.008, 1e-4, 1e-5
    );

    // Step 1: Predict 10 cycles (dt = 0.1)
    for (int i = 0; i < 10; ++i) {
        Vector3 acc(0.5, 0.2, 9.80665);
        Vector3 gyr(0.01, 0.02, 0.05);
        eskf.predict(acc, gyr, 0.1);
    }

    std::cout << "  Predict 10 pos: [" << eskf.get_position().x << ", " << eskf.get_position().y << ", " << eskf.get_position().z << "]" << std::endl;
    std::cout << "  Target pos: [112.332, 200.727, 49.8754]" << std::endl;
    assert(std::abs(eskf.get_position().x - 112.332072) < 1e-4);
    assert(std::abs(eskf.get_position().y - 200.726637) < 1e-4);
    assert(std::abs(eskf.get_position().z - 49.875420) < 1e-4);
    assert(std::abs(eskf.get_covariance().trace() - 6.534125) < 1e-4);

    // Step 2: AI Velocity
    eskf.update_ai_velocity(12.5, 0.35);
    std::cout << "  AI velocity pos.x: " << eskf.get_position().x << " (target: 112.956)" << std::endl;
    assert(std::abs(eskf.get_position().x - 112.955720) < 1e-4);
    assert(std::abs(eskf.get_covariance().trace() - 6.111362) < 1e-4);

    // Step 3: NHC
    eskf.update_nhc(0.05, 0.05);
    std::cout << "  NHC vel: [" << eskf.get_velocity().x << ", " << eskf.get_velocity().y << ", " << eskf.get_velocity().z << "]" << std::endl;
    assert(std::abs(eskf.get_velocity().x - 10.019554) < 1e-4);
    assert(std::abs(eskf.get_covariance().trace() - 5.271264) < 1e-4);

    // Step 4: Heading update
    auto res_hdg = eskf.update_heading(50.0 * DEG_TO_RAD, 0.05, 6.635);
    bool acc_hdg = res_hdg.first;
    double gam_hdg = res_hdg.second;
    assert(acc_hdg == false);
    assert(std::abs(gam_hdg - 23.136078) < 1e-3);
    std::cout << "  Heading gate correctly rejected outlier (gamma = " << gam_hdg << " > 6.635)" << std::endl;

    // Step 5: Barometric altitude update
    eskf.update_altitude(50.2, 0.8);
    assert(std::abs(eskf.get_position().z - 50.311566) < 1e-4);
    assert(std::abs(eskf.get_covariance().trace() - 3.618091) < 1e-4);
    std::cout << "  Altitude update passed." << std::endl;

    // Step 6: Road heading and road position
    auto res_rh = eskf.update_road_heading(52.0 * DEG_TO_RAD, 0.087, 9.0);
    bool acc_rh = res_rh.first;
    double gam_rh = res_rh.second;
    assert(acc_rh == false); // gamma 10.25 > 9.0

    Vector3 snapped_pos(eskf.get_position().x + 0.5, eskf.get_position().y - 0.2, eskf.get_position().z);
    Vector3 tangent_2d(std::sin(52.0 * DEG_TO_RAD), std::cos(52.0 * DEG_TO_RAD), 0.0);
    auto res_rp = eskf.update_road_position(snapped_pos, tangent_2d, 1.75, 16.0);
    bool acc_rp = res_rp.first;
    double gam_rp = res_rp.second;
    assert(acc_rp == true);
    assert(std::abs(gam_rp - 0.050872) < 1e-4);
    assert(std::abs(eskf.get_position().x - 110.408906) < 1e-4);
    std::cout << "  Road constraints update passed." << std::endl;


    // Step 7: GNSS pos & vel
    eskf.update_gnss(eskf.get_position() + Vector3(0.2, -0.1, 0.05), 2.5);
    eskf.update_gnss_velocity(eskf.get_velocity() + Vector3(0.05, -0.02, 0.01), 0.2);
    assert(std::abs(eskf.get_position().x - 110.457445) < 1e-4);
    assert(std::abs(eskf.get_velocity().x - 10.063571) < 1e-4);
    std::cout << "  GNSS pos & vel update passed." << std::endl;

    // Step 8: ZUPT
    eskf.update_zupt(0.01);
    assert(std::abs(eskf.get_velocity().x - 0.041222) < 1e-4);
    assert(std::abs(eskf.get_velocity().y - 0.008649) < 1e-4);
    std::cout << "  ZUPT update passed." << std::endl;

    std::cout << "[TEST] All ESKF 15-State Filter Tests PASSED with numerical parity!" << std::endl;
    return 0;
}

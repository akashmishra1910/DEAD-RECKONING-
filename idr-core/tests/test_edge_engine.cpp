#include "idr/edge_engine.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace idr;

int main() {
    std::cout << "[TEST] Starting Edge Engine C++17 API Tests..." << std::endl;

    EdgeNavigationEngine engine;
    assert(!engine.isInitialized());

    EngineConfig config;
    config.imu_sample_rate_hz = 200.0;
    config.target_update_rate_hz = 200.0;
    config.profile = NavigationProfile::EDGE_FOG;
    config.hardware_validated = false;

    bool init_ok = engine.initialize(config);
    assert(init_ok);
    assert(engine.isInitialized());

    // Push initial GNSS fix
    GnssSample gnss0;
    gnss0.timestamp_s = 0.0;
    gnss0.pos_enu = Vector3(0.0, 0.0, 0.0);
    gnss0.vel_enu = Vector3(0.0, 10.0, 0.0);
    gnss0.valid = true;
    gnss0.accuracy_m = 1.0;
    engine.pushGnss(gnss0);

    // Push initial IMU sample
    ImuSample imu0;
    imu0.timestamp_s = 0.0;
    imu0.accel_mps2 = Vector3(0.0, 0.0, 9.80665);
    imu0.gyro_rads = Vector3(0.0, 0.0, 0.0);
    imu0.in_g = false;
    imu0.in_deg_s = false;
    engine.pushImu(imu0);

    bool u0 = engine.update(10.0);
    assert(u0);

    // Run 200 Hz update cycles for 1 second (200 steps at dt=0.005s)
    const int num_steps = 200;
    const double dt = 0.005; // 200 Hz

    for (int i = 1; i <= num_steps; ++i) {
        double t = i * dt;
        ImuSample imu;
        imu.timestamp_s = t;
        imu.accel_mps2 = Vector3(0.0, 0.0, 9.80665);
        imu.gyro_rads = Vector3(0.0, 0.0, 0.0);
        imu.in_g = false;
        imu.in_deg_s = false;
        engine.pushImu(imu);

        // GNSS update once every second (at t=1.0)
        if (i == 200) {
            GnssSample gnss;
            gnss.timestamp_s = t;
            gnss.pos_enu = Vector3(0.0, 10.0, 0.0);
            gnss.vel_enu = Vector3(0.0, 10.0, 0.0);
            gnss.valid = true;
            gnss.accuracy_m = 1.0;
            engine.pushGnss(gnss);
        }

        bool update_ok = engine.update(10.0);
        assert(update_ok);
    }

    auto nav = engine.getNavigationState();
    std::cout << "  At t=" << nav.time_s << " pos=(" << nav.pos_enu.x << ", " << nav.pos_enu.y << ", " << nav.pos_enu.z << ")" << std::endl;
    std::cout << "  Velocity=(" << nav.vel_enu.x << ", " << nav.vel_enu.y << ", " << nav.vel_enu.z << ")" << std::endl;
    assert(nav.vel_enu.y > 5.0); // moving forward in North

    auto diag = engine.getDiagnostics();
    std::cout << "  Diagnostics: total_samples_processed=" << diag.total_samples_processed 
              << ", dropped_samples=" << diag.dropped_samples 
              << ", mean_cycle_time_us=" << diag.mean_cycle_time_us 
              << ", max_cycle_time_us=" << diag.max_cycle_time_us 
              << ", sustained_hz=" << diag.sustained_hz 
              << ", status=" << diag.hardware_status << std::endl;

    assert(diag.total_samples_processed == 201);
    assert(diag.dropped_samples == 0);
    assert(std::string(diag.hardware_status) == "SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE");

    // Test monotonicity / dropped packet detection
    ImuSample bad_imu;
    bad_imu.timestamp_s = 0.5; // past timestamp!
    engine.pushImu(bad_imu);
    bool bad_update = engine.update(10.0);
    assert(!bad_update);
    assert(engine.getDiagnostics().dropped_samples == 1);

    // Test reset
    engine.reset();
    assert(engine.getDiagnostics().total_samples_processed == 0);

    // Test shutdown
    engine.shutdown();
    assert(!engine.isInitialized());

    std::cout << "[TEST] Edge Engine C++17 API Tests PASSED!" << std::endl;
    return 0;
}

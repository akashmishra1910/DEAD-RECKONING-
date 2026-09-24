#pragma once

#include "idr/types.hpp"
#include "idr/navigation_engine.hpp"
#include <chrono>
#include <vector>
#include <memory>
#include <cstdint>

namespace idr {

enum class NavigationProfile {
    MOBILE = 0,
    EDGE_FOG = 1
};

struct ImuSample {
    double timestamp_s{0.0};
    Vector3 accel_mps2{0.0, 0.0, 0.0};
    Vector3 gyro_rads{0.0, 0.0, 0.0};
    bool in_g{false};
    bool in_deg_s{false};
};

struct GnssSample {
    double timestamp_s{0.0};
    Vector3 pos_enu{0.0, 0.0, 0.0};
    std::optional<Vector3> vel_enu{std::nullopt};
    bool valid{false};
    double accuracy_m{5.0};
};

struct BarometerSample {
    double timestamp_s{0.0};
    double alt_m{0.0};
};

struct EdgeCalibration {
    Matrix3x3 R_sensor_to_body{Matrix3x3::identity()};
    Vector3 accel_scale{1.0, 1.0, 1.0};
    Vector3 gyro_scale{1.0, 1.0, 1.0};
    Vector3 accel_bias{0.0, 0.0, 0.0};
    Vector3 gyro_bias{0.0, 0.0, 0.0};
    double time_offset_s{0.0};
};

struct EngineConfig {
    NavigationProfile profile{NavigationProfile::EDGE_FOG};
    double target_update_rate_hz{200.0};
    double imu_sample_rate_hz{200.0};
    double accel_noise_std{0.02};
    double gyro_noise_std{0.0005};
    double accel_bias_noise_std{1e-6};
    double gyro_bias_noise_std{1e-7};
    double lever_arm_m{0.0};
    bool enable_map_matching{false};
    EdgeCalibration calibration{};
    bool hardware_validated{false};
};

struct EdgeDiagnostics {
    uint64_t total_samples_processed{0};
    uint64_t dropped_samples{0};
    double mean_cycle_time_us{0.0};
    double max_cycle_time_us{0.0};
    double sustained_hz{0.0};
    NavigationMode current_mode{NavigationMode::INITIALIZING};
    bool in_outage{false};
    double p_trace{0.0};
    const char* hardware_status{"SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE"};
};

class EdgeNavigationEngine {
public:
    EdgeNavigationEngine();
    ~EdgeNavigationEngine();

    bool initialize(const EngineConfig& config);
    bool pushImu(const ImuSample& imu);
    bool pushGnss(const GnssSample& gnss);
    bool pushBarometer(const BarometerSample& baro);
    bool update(std::optional<double> ai_speed = std::nullopt);

    NavigationTelemetry getNavigationState() const;
    EdgeDiagnostics getDiagnostics() const;

    void reset();
    void shutdown();

    bool isInitialized() const { return initialized_; }

private:
    EngineConfig config_;
    bool initialized_{false};
    std::unique_ptr<SeamlessGNSSDeficitEngine> core_engine_{nullptr};

    std::optional<ImuSample> current_imu_{std::nullopt};
    std::optional<GnssSample> current_gnss_{std::nullopt};
    std::optional<BarometerSample> current_baro_{std::nullopt};

    double last_imu_timestamp_{-1.0};
    uint64_t total_cycles_{0};
    uint64_t dropped_samples_{0};
    NavigationTelemetry last_telemetry_{};

    std::vector<double> cycle_times_us_{};
};

} // namespace idr

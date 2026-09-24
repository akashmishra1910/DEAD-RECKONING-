#include "idr/edge_engine.hpp"
#include <chrono>
#include <cmath>
#include <numeric>

namespace idr {

EdgeNavigationEngine::EdgeNavigationEngine() = default;

EdgeNavigationEngine::~EdgeNavigationEngine() {
    shutdown();
}

bool EdgeNavigationEngine::initialize(const EngineConfig& config) {
    config_ = config;
    double dt = 1.0 / std::max(10.0, config.imu_sample_rate_hz);

    core_engine_ = std::make_unique<SeamlessGNSSDeficitEngine>(
        std::nullopt,
        std::nullopt,
        std::nullopt,
        11.345,
        9.0,
        dt,
        true,                      // enable heading supervisor
        config.enable_map_matching // enable map matching
    );

    initialized_ = true;
    last_imu_timestamp_ = -1.0;
    total_cycles_ = 0;
    dropped_samples_ = 0;
    cycle_times_us_.clear();
    cycle_times_us_.reserve(2000);

    return true;
}

bool EdgeNavigationEngine::pushImu(const ImuSample& imu) {
    if (!initialized_) return false;
    current_imu_ = imu;
    return true;
}

bool EdgeNavigationEngine::pushGnss(const GnssSample& gnss) {
    if (!initialized_) return false;
    current_gnss_ = gnss;
    return true;
}

bool EdgeNavigationEngine::pushBarometer(const BarometerSample& baro) {
    if (!initialized_) return false;
    current_baro_ = baro;
    return true;
}

bool EdgeNavigationEngine::update(std::optional<double> ai_speed) {
    if (!initialized_ || !core_engine_ || !current_imu_.has_value()) {
        return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    const auto& imu = current_imu_.value();

    // Timestamp monotonicity check (dropped packet detection)
    if (last_imu_timestamp_ >= 0.0 && imu.timestamp_s <= last_imu_timestamp_) {
        dropped_samples_++;
        return false;
    }
    last_imu_timestamp_ = imu.timestamp_s;

    // Unit conversion
    Vector3 accel_si = imu.accel_mps2;
    if (imu.in_g) {
        accel_si = accel_si * 9.80665;
    }

    Vector3 gyro_si = imu.gyro_rads;
    if (imu.in_deg_s) {
        constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
        gyro_si = gyro_si * DEG_TO_RAD;
    }

    // Scale Factor & Bias Calibration
    Vector3 accel_cal(
        (accel_si.x * config_.calibration.accel_scale.x) - config_.calibration.accel_bias.x,
        (accel_si.y * config_.calibration.accel_scale.y) - config_.calibration.accel_bias.y,
        (accel_si.z * config_.calibration.accel_scale.z) - config_.calibration.accel_bias.z
    );

    Vector3 gyro_cal(
        (gyro_si.x * config_.calibration.gyro_scale.x) - config_.calibration.gyro_bias.x,
        (gyro_si.y * config_.calibration.gyro_scale.y) - config_.calibration.gyro_bias.y,
        (gyro_si.z * config_.calibration.gyro_scale.z) - config_.calibration.gyro_bias.z
    );

    // Sensor-to-Body Mounting Rotation
    Vector3 accel_b = config_.calibration.R_sensor_to_body * accel_cal;
    Vector3 gyro_b = config_.calibration.R_sensor_to_body * gyro_cal;

    // Barometer extraction
    double baro_alt = 0.0;
    if (current_baro_.has_value()) {
        baro_alt = current_baro_->alt_m;
    }

    // GNSS extraction
    std::optional<Vector3> gnss_pos = std::nullopt;
    bool gnss_valid = false;
    if (current_gnss_.has_value() && current_gnss_->valid) {
        gnss_pos = current_gnss_->pos_enu;
        gnss_valid = true;
    }

    // Process single core navigation step
    last_telemetry_ = core_engine_->process_step(
        imu.timestamp_s + config_.calibration.time_offset_s,
        accel_b,
        gyro_b,
        baro_alt,
        gnss_pos,
        gnss_valid,
        ai_speed,
        current_gnss_.has_value() ? current_gnss_->vel_enu : std::nullopt,
        std::nullopt,
        true
    );

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_us = std::chrono::duration<double, std::micro>(end_time - start_time).count();

    if (cycle_times_us_.size() >= 2000) {
        cycle_times_us_.erase(cycle_times_us_.begin());
    }
    cycle_times_us_.push_back(elapsed_us);
    total_cycles_++;

    return true;
}

NavigationTelemetry EdgeNavigationEngine::getNavigationState() const {
    return last_telemetry_;
}

EdgeDiagnostics EdgeNavigationEngine::getDiagnostics() const {
    EdgeDiagnostics diag;
    diag.total_samples_processed = total_cycles_;
    diag.dropped_samples = dropped_samples_;
    diag.current_mode = last_telemetry_.mode;
    diag.in_outage = last_telemetry_.in_outage;

    if (!cycle_times_us_.empty()) {
        double sum = std::accumulate(cycle_times_us_.begin(), cycle_times_us_.end(), 0.0);
        diag.mean_cycle_time_us = sum / static_cast<double>(cycle_times_us_.size());

        double max_val = 0.0;
        for (double v : cycle_times_us_) {
            if (v > max_val) max_val = v;
        }
        diag.max_cycle_time_us = max_val;
        diag.sustained_hz = diag.mean_cycle_time_us > 0.0 ? (1e6 / diag.mean_cycle_time_us) : 0.0;
    }

    if (config_.hardware_validated) {
        diag.hardware_status = "HARDWARE_VALIDATED";
    } else {
        diag.hardware_status = "SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE";
    }

    return diag;
}

void EdgeNavigationEngine::reset() {
    if (initialized_) {
        EngineConfig cfg = config_;
        shutdown();
        initialize(cfg);
    }
}

void EdgeNavigationEngine::shutdown() {
    initialized_ = false;
    core_engine_.reset();
    current_imu_.reset();
    current_gnss_.reset();
    current_baro_.reset();
    cycle_times_us_.clear();
}

} // namespace idr

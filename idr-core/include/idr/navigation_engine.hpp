#pragma once

#include "idr/types.hpp"
#include "idr/math_utils.hpp"
#include "idr/eskf.hpp"
#include "idr/alignment_engine.hpp"
#include "idr/vibration_filter.hpp"
#include "idr/heading_supervisor.hpp"
#include "idr/osm_graph.hpp"
#include "idr/map_matcher_3d.hpp"
#include "idr/speed_calibrator.hpp"
#include "idr/longitudinal_sync.hpp"
#include <vector>
#include <memory>



namespace idr {



struct SupervisorDiagnostics {
    const char* last_transition_reason = "Initialization";
    double time_in_current_mode_s = 0.0;
    double gnss_pos_innovation_gamma = 0.0;
    double gnss_vel_innovation_gamma = 0.0;
    int reacquisition_confirmed_epochs = 0;
    int consecutive_outage_epochs = 0;
    int consecutive_valid_gnss_epochs = 0;
    double ai_vel_weight = 0.0;
    double gnss_vel_weight = 0.0;
    bool map_constraints_active = false;
    const char* map_confidence = "INVALID";
    double p_trace = 0.0;
    double speed_scale_est = 1.0;
    double speed_bias_est_mps = 0.0;
    bool speed_calibrated = false;
    bool longitudinal_constraint_applied = false;
    double longitudinal_innovation_m = 0.0;
};

struct NavigationTelemetry {
    NavigationMode mode = NavigationMode::INITIALIZING;
    double time_s = 0.0;
    Vector3 pos_enu{0.0, 0.0, 0.0};
    Vector3 vel_enu{0.0, 0.0, 0.0};
    double speed_kmh = 0.0;
    double heading_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    Vector3 accel_bias{0.0, 0.0, 0.0};
    Vector3 gyro_bias{0.0, 0.0, 0.0};
    bool in_outage = false;
    double outage_duration_s = 0.0;
    double outage_dist_traveled_m = 0.0;
    bool gnss_valid = false;
    bool is_pothole_active = false;
    int pothole_count = 0;
    bool is_idling = false;
    std::array<double, 15> covariance_diag{};
    std::optional<HeadingSupervisorTelemetry> heading_telemetry = std::nullopt;
    SupervisorDiagnostics diagnostics;

    // Explicit Separation of Raw DR and Map-Matched Presentation State (Phase 11.8)
    Vector3 raw_dr_pos_enu{0.0, 0.0, 0.0};
    Vector3 map_matched_pos_enu{0.0, 0.0, 0.0};
    int32_t selected_edge_id = -1;
    int32_t selected_layer = 0;
    const char* selected_edge_name = "Unknown";
    const char* selected_layer_label = "Ground";
    MapMatchConfidence map_confidence = MapMatchConfidence::INVALID;
    bool map_ambiguous = false;
    uint32_t candidate_count = 0;
    uint32_t hypothesis_count = 0;
    double score_margin = 0.0;
    bool map_available = false;
};

class SeamlessGNSSDeficitEngine {
public:
    explicit SeamlessGNSSDeficitEngine(
        const std::optional<Vector3>& init_pos = std::nullopt,
        const std::optional<Vector3>& init_vel = std::nullopt,
        const std::optional<Vector3>& init_euler = std::nullopt,
        double chi2_pos_threshold = 11.345,
        double chi2_vel_threshold = 9.0,
        double dt = 0.1,
        bool enable_heading_supervisor = true,
        bool enable_map_matching = true,
        const std::optional<OfflineOSMGraph>& osm_graph = std::nullopt,
        int min_reacquisition_dwell_epochs = 4,
        bool enable_speed_calibration = true,
        bool enable_longitudinal_map_constraint = true
    );

    NavigationTelemetry process_step(
        double time_s,
        const Vector3& accel_raw_p,
        const Vector3& gyro_raw_p,
        double baro_alt,
        const std::optional<Vector3>& gnss_pos,
        bool gnss_valid,
        std::optional<double> ai_speed = std::nullopt,
        const std::optional<Vector3>& gnss_vel = std::nullopt,
        const std::optional<Vector3>& mag_raw_p = std::nullopt,
        bool enable_road_constraints = true
    );

    NavigationMode get_mode() const { return mode_; }
    const ErrorStateKalmanFilter& get_eskf() const { return eskf_; }
    ErrorStateKalmanFilter& get_eskf() { return eskf_; }
    OfflineOSMGraph* get_osm_graph_mutable() {
        if (map_matcher_) {
            return &map_matcher_->get_graph();
        }
        return nullptr;
    }

    bool load_idrmap(const char* path, double lat0, double lon0, double alt0);
    bool set_idrmap_reader(const IdrMapReader* reader, double lat0, double lon0, double alt0);
    void reset();
    const std::optional<MapMatchResult>& get_last_map_match() const { return last_match_result_; }
    Causal3DMapMatcher* get_map_matcher() { return map_matcher_.get(); }

private:
    std::tuple<bool, bool, double, double> evaluate_gnss_outage_causal(
        double time_s,
        const std::optional<Vector3>& gnss_pos,
        const std::optional<Vector3>& gnss_vel,
        bool gnss_valid
    );

    void update_supervisor_state_machine(double time_s, bool is_outage, bool is_degraded);

    double dt_;
    double chi2_pos_threshold_;
    double chi2_vel_threshold_;
    int min_reacquisition_dwell_;

    InVehicleAlignmentEngine alignment_engine_;
    VibrationPotholeFilter vibration_filter_;
    ErrorStateKalmanFilter eskf_;

    bool enable_heading_supervisor_;
    std::unique_ptr<HeadingSupervisor> heading_supervisor_;

    bool enable_map_matching_;
    std::unique_ptr<Causal3DMapMatcher> map_matcher_;
    std::unique_ptr<IdrMapReader> owned_idrmap_reader_;

    bool enable_speed_calibration_;
    std::unique_ptr<OnlineSpeedScaleBiasCalibrator> speed_calibrator_;

    bool enable_longitudinal_map_constraint_;
    std::unique_ptr<CausalLongitudinalMapSynchronizer> longitudinal_synchronizer_;

    NavigationMode mode_ = NavigationMode::INITIALIZING;
    double mode_entry_time_ = 0.0;
    const char* last_transition_reason_ = "System Init";

    int consecutive_valid_gnss_ = 0;
    int consecutive_outage_epochs_ = 0;
    int reacquisition_epochs_ = 0;

    double last_gnss_time_ = -1.0;
    double outage_start_time_ = 0.0;
    double outage_distance_traveled_ = 0.0;
    double total_distance_traveled_ = 0.0;

    Vector3 last_pos_{0.0, 0.0, 0.0};
    std::optional<MapMatchResult> last_match_result_ = std::nullopt;
};


} // namespace idr

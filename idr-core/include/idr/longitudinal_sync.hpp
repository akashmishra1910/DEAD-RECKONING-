#pragma once

#include "idr/types.hpp"
#include "idr/eskf.hpp"
#include "idr/osm_graph.hpp"
#include "idr/map_matcher_3d.hpp"
#include <vector>

namespace idr {

struct LongitudinalConstraintResult {
    bool applied = false;
    double innovation_m = 0.0;
    double chi2_val = 0.0;
    double road_turn_s = 0.0;
    double estimated_s = 0.0;
    const char* confidence = "NONE";
};

class CausalLongitudinalMapSynchronizer {
public:
    explicit CausalLongitudinalMapSynchronizer(
        double min_yaw_rate_rads = 0.12,
        double r_along_m2 = 4.0,
        double chi2_threshold = 9.0
    );

    LongitudinalConstraintResult process_step(
        double time_s,
        ErrorStateKalmanFilter& eskf,
        const MapMatchResult& match_result,
        double yaw_rate_z,
        double dt,
        const OSMEdge* edge = nullptr,
        const OfflineOSMGraph* graph = nullptr
    );

private:
    double min_yaw_rate_;
    double r_along_;
    double chi2_threshold_;
    double last_turn_time_ = -999.0;
    std::array<std::pair<double, double>, 32> yaw_rate_history_{};
    size_t yaw_rate_count_ = 0;
    size_t yaw_rate_head_ = 0;

    // Scratch buffers for polyline curvature calculations
    std::vector<double> seg_azimuths_scratch_;
    std::vector<double> seg_lens_scratch_;
    std::vector<double> turn_angles_scratch_;
};

} // namespace idr

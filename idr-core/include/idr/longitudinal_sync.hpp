#pragma once

#include "idr/types.hpp"
#include "idr/eskf.hpp"
#include "idr/osm_graph.hpp"
#include "idr/map_matcher_3d.hpp"
#include <string>
#include <vector>

namespace idr {

struct LongitudinalConstraintResult {
    bool applied = false;
    double innovation_m = 0.0;
    double chi2_val = 0.0;
    double road_turn_s = 0.0;
    double estimated_s = 0.0;
    std::string confidence = "NONE";
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
    std::vector<std::pair<double, double>> yaw_rate_history_;
};

} // namespace idr

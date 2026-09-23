#pragma once

#include "idr/types.hpp"
#include "idr/osm_graph.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <cmath>

namespace idr {

enum class MapMatchConfidence {
    HIGH,
    MEDIUM,
    LOW,
    INVALID
};

inline const char* map_match_confidence_to_string(MapMatchConfidence c) {
    switch (c) {
        case MapMatchConfidence::HIGH: return "HIGH";
        case MapMatchConfidence::MEDIUM: return "MEDIUM";
        case MapMatchConfidence::LOW: return "LOW";
        case MapMatchConfidence::INVALID: return "INVALID";
        default: return "UNKNOWN";
    }
}

struct MapMatchResult {
    int edge_id = -1;
    std::string edge_name;
    std::string road_type;
    int layer = 0;
    std::string layer_label;
    Vector3 snapped_pos{0.0, 0.0, 0.0};
    double d_2d = 999.0;
    double delta_h = 999.0;
    double road_heading = 0.0;
    Vector3 road_tangent_2d{0.0, 1.0, 0.0};
    MapMatchConfidence confidence = MapMatchConfidence::INVALID;
    double log_posterior = -999.0;
    bool is_valid = false;
};

class Causal3DMapMatcher {
public:
    explicit Causal3DMapMatcher(
        const std::optional<OfflineOSMGraph>& graph = std::nullopt,
        double sigma_d = 5.0,
        double sigma_h = 1.8,
        double sigma_psi = 0.30,
        double beta = 5.0,
        double elevation_barrier = 4.0
    );

    void reset();

    double compute_emission_log_prob(
        double d_2d,
        double delta_h,
        double d_psi,
        bool use_3d = true
    ) const;

    MapMatchResult process_step(
        const Vector3& pos_dr,
        double heading_dr,
        double baro_alt,
        double speed_dr = 10.0,
        double dt = 0.1,
        bool use_3d = true
    );

    const OfflineOSMGraph& get_graph() const { return graph_; }
    OfflineOSMGraph& get_graph() { return graph_; }

private:
    OfflineOSMGraph graph_;
    double sigma_d_;
    double sigma_h_;
    double sigma_psi_;
    double beta_;
    double elevation_barrier_;

    std::unordered_map<int, double> active_hypotheses_;
    std::optional<Vector3> prev_pos_dr_ = std::nullopt;
    std::optional<int> last_matched_edge_id_ = std::nullopt;
    int consecutive_valid_steps_ = 0;
};

} // namespace idr

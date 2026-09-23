#pragma once

#include "idr/types.hpp"
#include "idr/math_utils.hpp"
#include "idr/osm_graph.hpp"
#include "idr/idrmap_reader.hpp"
#include <vector>
#include <unordered_map>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>

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

/**
 * @brief Individual HMM hypothesis tracking a candidate road edge.
 */
struct IdrMapHypothesis {
    uint32_t edge_id = 0;
    int8_t layer = 0;
    uint8_t road_class = 0;
    uint8_t flags = 0;
    double log_prob = -999.0;         /**< Normalized log posterior (0.0 = top) */
    double emission_log_prob = -999.0;
    double transition_log_prob = -999.0;
    double along_track_m = 0.0;
    double d_2d = 999.0;
    double delta_h = 999.0;
    double d_psi = 0.0;
    double road_heading_deg = 0.0;
    Vector3 snapped_pos{0.0, 0.0, 0.0};
    Vector3 road_tangent_2d{0.0, 1.0, 0.0};
    uint32_t u_node_id = 0;
    uint32_t v_node_id = 0;
    int consecutive_matches = 0;
    int age = 0;
};

/**
 * @brief Result of a causal 3D HMM map-matching step.
 */
struct MapMatchResult {
    int edge_id = -1;
    const char* edge_name = "";
    const char* road_type = "";
    int layer = 0;
    const char* layer_label = "Ground";
    Vector3 snapped_pos{0.0, 0.0, 0.0};
    double d_2d = 999.0;
    double delta_h = 999.0;
    double road_heading = 0.0;        /**< Azimuth in radians clockwise from North */
    Vector3 road_tangent_2d{0.0, 1.0, 0.0};
    MapMatchConfidence confidence = MapMatchConfidence::INVALID;
    double log_posterior = -999.0;
    double runner_up_log_posterior = -999.0;
    double score_margin = 0.0;
    bool is_ambiguous = false;
    bool is_valid = false;
    size_t candidate_count = 0;
    size_t hypothesis_count = 0;
};

/**
 * @brief Production Causal 3D HMM Map Matcher.
 *
 * Implements step-by-step causal Viterbi state tracking over 3D road graphs
 * (supporting both OfflineOSMGraph and IdrMapReader binary backends).
 */
class Causal3DMapMatcher {
public:
    static constexpr size_t MAX_HYPOTHESES = 8;
    static constexpr size_t MAX_CANDIDATES = 32;

    Causal3DMapMatcher(
        double sigma_d = 5.0,
        double sigma_h = 1.8,
        double sigma_psi = 0.30,
        double beta = 5.0,
        double elevation_barrier = 4.0
    );

    explicit Causal3DMapMatcher(
        const OfflineOSMGraph& graph,
        double sigma_d = 5.0,
        double sigma_h = 1.8,
        double sigma_psi = 0.30,
        double beta = 5.0,
        double elevation_barrier = 4.0
    );

    explicit Causal3DMapMatcher(
        const IdrMapReader* idrmap_reader,
        double lat0 = 0.0,
        double lon0 = 0.0,
        double alt0 = 0.0,
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

    const IdrMapReader* get_idrmap_reader() const { return idrmap_reader_; }
    void set_idrmap_reader(const IdrMapReader* reader, double lat0, double lon0, double alt0) {
        idrmap_reader_ = reader;
        lat0_ = lat0;
        lon0_ = lon0;
        alt0_ = alt0;
    }

    size_t get_active_hypotheses(IdrMapHypothesis* out_hypotheses, size_t max_hyp) const;
    size_t get_active_hypothesis_count() const { return active_hyp_count_; }

    struct CandidateScore {
        const OSMEdge* edge = nullptr;
        uint32_t edge_id = 0;
        const char* name = "";
        const char* road_type = "";
        int8_t layer = 0;
        uint8_t road_class = 0;
        uint8_t flags = 0;
        double score = -999.0;
        double emission_score = -999.0;
        double trans_score = -999.0;
        double d_2d = 0.0;
        double delta_h = 0.0;
        double d_psi = 0.0;
        double along_track_m = 0.0;
        double edge_length = 0.0;
        uint32_t u_node_id = 0;
        uint32_t v_node_id = 0;
        Vector3 snapped{0.0, 0.0, 0.0};
        Vector3 tangent{0.0, 1.0, 0.0};
        double azimuth = 0.0;
    };

private:
    OfflineOSMGraph graph_;
    const IdrMapReader* idrmap_reader_ = nullptr;
    double lat0_ = 0.0;
    double lon0_ = 0.0;
    double alt0_ = 0.0;

    double sigma_d_;
    double sigma_h_;
    double sigma_psi_;
    double beta_;
    double elevation_barrier_;

    std::array<IdrMapHypothesis, MAX_HYPOTHESES> active_hypotheses_{};
    size_t active_hyp_count_ = 0;

    bool has_prev_pos_dr_ = false;
    Vector3 prev_pos_dr_{0.0, 0.0, 0.0};

    bool has_last_matched_edge_id_ = false;
    int last_matched_edge_id_ = -1;
    int consecutive_valid_steps_ = 0;

    // Fixed-capacity zero-allocation scratch buffers
    mutable std::vector<int> scratch_edge_ids_;
    std::vector<const OSMEdge*> candidates_scratch_;
    std::array<CandidateScore, MAX_CANDIDATES> scored_candidates_{};
    size_t scored_count_ = 0;
};

} // namespace idr

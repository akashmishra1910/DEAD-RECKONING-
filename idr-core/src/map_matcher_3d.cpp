#include "idr/map_matcher_3d.hpp"
#include <algorithm>
#include <limits>

namespace idr {

Causal3DMapMatcher::Causal3DMapMatcher(
    const std::optional<OfflineOSMGraph>& graph,
    double sigma_d,
    double sigma_h,
    double sigma_psi,
    double beta,
    double elevation_barrier
) : graph_(graph.has_value() ? *graph : OfflineOSMGraph::build_synthetic_3d_corridor()),
    sigma_d_(sigma_d),
    sigma_h_(sigma_h),
    sigma_psi_(sigma_psi),
    beta_(beta),
    elevation_barrier_(elevation_barrier)
{
    reset();
    // Pre-allocate scratch vectors to eliminate per-cycle heap allocation
    candidates_scratch_.reserve(64);
    scored_scratch_.reserve(64);
    new_hypotheses_scratch_.reserve(64);
    scratch_edge_ids_.reserve(64);
    active_hypotheses_.reserve(32);
    // Pre-populate topological distance cache for all edge pairs
    graph_.warm_cache();
}

void Causal3DMapMatcher::reset() {
    active_hypotheses_.clear();
    prev_pos_dr_ = std::nullopt;
    last_matched_edge_id_ = std::nullopt;
    consecutive_valid_steps_ = 0;
}

double Causal3DMapMatcher::compute_emission_log_prob(
    double d_2d,
    double delta_h,
    double d_psi,
    bool use_3d
) const {
    double log_p_dist = -0.5 * (d_2d / sigma_d_) * (d_2d / sigma_d_);
    double log_p_psi = -0.5 * (d_psi / sigma_psi_) * (d_psi / sigma_psi_);

    if (!use_3d) {
        return log_p_dist + log_p_psi;
    }

    double log_p_alt;
    if (delta_h > elevation_barrier_) {
        // Severe elevation barrier penalty (P <= 10^-12)
        log_p_alt = -50.0 - 5.0 * (delta_h - elevation_barrier_);
    } else {
        log_p_alt = -0.5 * (delta_h / sigma_h_) * (delta_h / sigma_h_);
    }

    return log_p_dist + log_p_alt + log_p_psi;
}

MapMatchResult Causal3DMapMatcher::process_step(
    const Vector3& pos_dr,
    double heading_dr,
    double baro_alt,
    double speed_dr,
    double dt,
    bool use_3d
) {
    Vector3 obs_pt(pos_dr.x, pos_dr.y, baro_alt);

    double search_radius = std::max(35.0, 3.0 * sigma_d_);
    graph_.get_candidate_edges(obs_pt, search_radius, candidates_scratch_, scratch_edge_ids_);
    if (candidates_scratch_.empty()) {
        for (const auto& kv : graph_.get_edges()) {
            candidates_scratch_.push_back(&kv.second);
        }
    }

    if (candidates_scratch_.empty()) {
        MapMatchResult res;
        res.edge_id = -1;
        res.edge_name = "Unknown";
        res.road_type = "unknown";
        res.layer = 0;
        res.layer_label = "Ground";
        res.snapped_pos = pos_dr;
        res.d_2d = 999.0;
        res.delta_h = 999.0;
        res.road_heading = heading_dr;
        res.road_tangent_2d = Vector3(0.0, 1.0, 0.0);
        res.confidence = MapMatchConfidence::INVALID;
        res.log_posterior = -999.0;
        res.is_valid = false;
        return res;
    }

    double dr_dist;
    if (prev_pos_dr_.has_value()) {
        dr_dist = std::hypot(pos_dr.x - prev_pos_dr_->x, pos_dr.y - prev_pos_dr_->y);
    } else {
        dr_dist = speed_dr * dt;
    }

    scored_scratch_.clear();
    new_hypotheses_scratch_.clear();

    for (const auto* edge : candidates_scratch_) {
        auto [d2d, dh, snapped, tangent, az] = graph_.project_point_to_edge(obs_pt, *edge);

        double dpsi = std::abs(wrap_pi(heading_dr - az));
        double log_emission = compute_emission_log_prob(d2d, dh, dpsi, use_3d);

        double total_score;
        if (!active_hypotheses_.empty()) {
            double best_trans_score = -std::numeric_limits<double>::infinity();
            for (const auto& [prev_eid, prev_log_prob] : active_hypotheses_) {
                double top_dist = graph_.get_topological_distance(prev_eid, edge->id);
                double log_trans;
                if (std::isinf(top_dist)) {
                    log_trans = -40.0;
                } else {
                    double disc = std::abs(top_dist - dr_dist);
                    log_trans = -disc / beta_;
                }
                double tr = prev_log_prob + log_trans;
                if (tr > best_trans_score) {
                    best_trans_score = tr;
                }
            }
            total_score = best_trans_score + log_emission;
        } else {
            total_score = log_emission;
        }

        new_hypotheses_scratch_.push_back({edge->id, total_score});
        scored_scratch_.push_back({edge, total_score, d2d, dh, dpsi, snapped, tangent, az});
    }

    std::sort(scored_scratch_.begin(), scored_scratch_.end(), [](const CandidateScore& a, const CandidateScore& b) {
        return a.score > b.score;
    });

    const auto& best = scored_scratch_[0];

    // Prune hypotheses for next step (keep top 6)
    std::sort(new_hypotheses_scratch_.begin(), new_hypotheses_scratch_.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });
    if (new_hypotheses_scratch_.size() > 6) {
        new_hypotheses_scratch_.resize(6);
    }
    double max_s = new_hypotheses_scratch_[0].second;
    active_hypotheses_.clear();
    for (const auto& [eid, s] : new_hypotheses_scratch_) {
        active_hypotheses_.push_back({eid, s - max_s});
    }
    prev_pos_dr_ = pos_dr;

    const auto* best_edge = best.edge;
    double d2d = best.d_2d;
    double dh = best.delta_h;
    double dpsi = best.d_psi;

    MapMatchConfidence confidence;
    if (d2d < 5.0 && dpsi < (18.0 * DEG_TO_RAD) && (!use_3d || dh < 2.5)) {
        confidence = MapMatchConfidence::HIGH;
        consecutive_valid_steps_++;
    } else if (d2d < 15.0 && dpsi < (35.0 * DEG_TO_RAD) && (!use_3d || dh < 4.0)) {
        confidence = MapMatchConfidence::MEDIUM;
        consecutive_valid_steps_++;
    } else if (d2d < 30.0) {
        confidence = MapMatchConfidence::LOW;
        consecutive_valid_steps_ = std::max(0, consecutive_valid_steps_ - 1);
    } else {
        confidence = MapMatchConfidence::INVALID;
        consecutive_valid_steps_ = 0;
    }

    int layer = best_edge->layer;
    const char* layer_label;
    if (best_edge->road_type == "flyover" || layer >= 1) {
        layer_label = "Flyover";
    } else if (best_edge->is_tunnel || layer <= -1 || best_edge->road_type == "underpass") {
        layer_label = "Underpass";
    } else if (best_edge->road_type == "service") {
        layer_label = "Service Road";
    } else {
        layer_label = "Ground";
    }

    bool is_valid = (confidence != MapMatchConfidence::INVALID);
    if (is_valid) {
        last_matched_edge_id_ = best_edge->id;
    }

    MapMatchResult res;
    res.edge_id = best_edge->id;
    res.edge_name = best_edge->name.c_str();    // stable: OSMEdge in unordered_map never moved
    res.road_type = best_edge->road_type.c_str(); // stable: same reason
    res.layer = layer;
    res.layer_label = layer_label;
    res.snapped_pos = best.snapped;
    res.d_2d = d2d;
    res.delta_h = dh;
    res.road_heading = best.azimuth;
    res.road_tangent_2d = best.tangent;
    res.confidence = confidence;
    res.log_posterior = best.score;
    res.is_valid = is_valid;

    return res;
}

} // namespace idr

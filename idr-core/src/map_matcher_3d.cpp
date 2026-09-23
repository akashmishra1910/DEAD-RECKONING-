#include "idr/map_matcher_3d.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include <cstring>

namespace idr {

Causal3DMapMatcher::Causal3DMapMatcher(
    double sigma_d,
    double sigma_h,
    double sigma_psi,
    double beta,
    double elevation_barrier
) : graph_(OfflineOSMGraph::build_synthetic_3d_corridor()),
    idrmap_reader_(nullptr),
    sigma_d_(sigma_d),
    sigma_h_(sigma_h),
    sigma_psi_(sigma_psi),
    beta_(beta),
    elevation_barrier_(elevation_barrier)
{
    reset();
    candidates_scratch_.reserve(64);
    scratch_edge_ids_.reserve(64);
    graph_.warm_cache();
}

Causal3DMapMatcher::Causal3DMapMatcher(
    const OfflineOSMGraph& graph,
    double sigma_d,
    double sigma_h,
    double sigma_psi,
    double beta,
    double elevation_barrier
) : graph_(graph),
    idrmap_reader_(nullptr),
    sigma_d_(sigma_d),
    sigma_h_(sigma_h),
    sigma_psi_(sigma_psi),
    beta_(beta),
    elevation_barrier_(elevation_barrier)
{
    reset();
    candidates_scratch_.reserve(64);
    scratch_edge_ids_.reserve(64);
    graph_.warm_cache();
}

Causal3DMapMatcher::Causal3DMapMatcher(
    const IdrMapReader* idrmap_reader,
    double lat0,
    double lon0,
    double alt0,
    double sigma_d,
    double sigma_h,
    double sigma_psi,
    double beta,
    double elevation_barrier
) : graph_(OfflineOSMGraph::build_synthetic_3d_corridor()),
    idrmap_reader_(idrmap_reader),
    lat0_(lat0),
    lon0_(lon0),
    alt0_(alt0),
    sigma_d_(sigma_d),
    sigma_h_(sigma_h),
    sigma_psi_(sigma_psi),
    beta_(beta),
    elevation_barrier_(elevation_barrier)
{
    reset();
    candidates_scratch_.reserve(64);
    scratch_edge_ids_.reserve(64);
}

void Causal3DMapMatcher::reset() {
    active_hyp_count_ = 0;
    for (auto& h : active_hypotheses_) {
        h = IdrMapHypothesis{};
    }
    has_prev_pos_dr_ = false;
    prev_pos_dr_ = Vector3(0.0, 0.0, 0.0);
    has_last_matched_edge_id_ = false;
    last_matched_edge_id_ = -1;
    consecutive_valid_steps_ = 0;
    scored_count_ = 0;
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

size_t Causal3DMapMatcher::get_active_hypotheses(IdrMapHypothesis* out_hypotheses, size_t max_hyp) const {
    if (!out_hypotheses || max_hyp == 0) return 0;
    size_t count = std::min(active_hyp_count_, max_hyp);
    for (size_t i = 0; i < count; ++i) {
        out_hypotheses[i] = active_hypotheses_[i];
    }
    return count;
}

MapMatchResult Causal3DMapMatcher::process_step(
    const Vector3& pos_dr,
    double heading_dr,
    double baro_alt,
    double speed_dr,
    double dt,
    bool use_3d
) {
    // 0. Numerical input safety guard
    if (std::isnan(pos_dr.x) || std::isnan(pos_dr.y) || std::isnan(pos_dr.z) ||
        std::isinf(pos_dr.x) || std::isinf(pos_dr.y) || std::isinf(pos_dr.z) ||
        std::isnan(heading_dr) || std::isinf(heading_dr) ||
        std::isnan(baro_alt) || std::isinf(baro_alt) ||
        std::isnan(speed_dr) || std::isinf(speed_dr) ||
        std::isnan(dt) || dt <= 0.0) {
        MapMatchResult res;
        res.edge_id = -1;
        res.edge_name = "Invalid";
        res.road_type = "invalid";
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

    scored_count_ = 0;
    double search_radius = std::max(35.0, 3.0 * sigma_d_);

    // 1. Candidate Edge Retrieval (using either IdrMapReader or OfflineOSMGraph)
    if (idrmap_reader_ != nullptr && idrmap_reader_->is_open()) {
        auto wgs = enu_to_wgs84(pos_dr, lat0_, lon0_, alt0_);
        double lat = std::get<0>(wgs);
        double lon = std::get<1>(wgs);

        IdrRoadCandidate cand_pool[MAX_CANDIDATES];
        size_t n_cands = idrmap_reader_->query_nearest_roads(lat, lon, search_radius, MAX_CANDIDATES, cand_pool, MAX_CANDIDATES);

        for (size_t i = 0; i < n_cands && scored_count_ < MAX_CANDIDATES; ++i) {
            const auto& c = cand_pool[i];
            CandidateScore cs;
            cs.edge = nullptr;
            cs.edge_id = c.edge_id;
            cs.name = c.name;
            cs.road_type = (c.layer >= 1) ? "flyover" : ((c.flags & IDRMAP_FLAG_TUNNEL) ? "tunnel" : "road");
            cs.layer = c.layer;
            cs.road_class = c.road_class;
            cs.flags = c.flags;
            cs.d_2d = c.distance_m;

            // Compute snapped ENU position
            Vector3 snap_enu = wgs84_to_enu(c.proj_lat, c.proj_lon, baro_alt, lat0_, lon0_, alt0_);
            double expected_alt = alt0_ + static_cast<double>(c.layer) * 8.5;
            snap_enu.z = expected_alt;
            cs.snapped = snap_enu;

            cs.delta_h = std::abs(baro_alt - expected_alt);
            cs.azimuth = c.segment_heading_deg * DEG_TO_RAD;
            cs.tangent = Vector3(std::sin(cs.azimuth), std::cos(cs.azimuth), 0.0);

            // Compute heading discrepancy (taking oneway into account)
            double dpsi_fwd = std::abs(wrap_pi(heading_dr - cs.azimuth));
            if (!c.oneway) {
                double opp_az = wrap_pi(cs.azimuth + PI);
                double dpsi_rev = std::abs(wrap_pi(heading_dr - opp_az));
                cs.d_psi = std::min(dpsi_fwd, dpsi_rev);
            } else {
                cs.d_psi = dpsi_fwd;
            }

            cs.along_track_m = 0.0;
            cs.edge_length = c.length_m;
            cs.u_node_id = 0;
            cs.v_node_id = 0;

            scored_candidates_[scored_count_++] = cs;
        }
    } else {
        Vector3 obs_pt(pos_dr.x, pos_dr.y, baro_alt);
        graph_.get_candidate_edges(obs_pt, search_radius, candidates_scratch_, scratch_edge_ids_);
        if (candidates_scratch_.empty()) {
            for (const auto& kv : graph_.get_edges()) {
                candidates_scratch_.push_back(&kv.second);
            }
        }

        for (const auto* edge : candidates_scratch_) {
            if (scored_count_ >= MAX_CANDIDATES) break;
            auto proj = graph_.project_point_to_edge(obs_pt, *edge);
            double d2d = std::get<0>(proj);
            double dh = std::get<1>(proj);
            Vector3 snapped = std::get<2>(proj);
            Vector3 tangent = std::get<3>(proj);
            double az = std::get<4>(proj);

            double dpsi = std::abs(wrap_pi(heading_dr - az));

            CandidateScore cs;
            cs.edge = edge;
            cs.edge_id = static_cast<uint32_t>(edge->id);
            cs.name = edge->name.c_str();
            cs.road_type = edge->road_type.c_str();
            cs.layer = static_cast<int8_t>(edge->layer);
            cs.road_class = 0;
            cs.flags = edge->is_tunnel ? IDRMAP_FLAG_TUNNEL : 0;
            cs.d_2d = d2d;
            cs.delta_h = dh;
            cs.d_psi = dpsi;
            cs.along_track_m = 0.0;
            cs.edge_length = edge->length;
            cs.u_node_id = static_cast<uint32_t>(edge->source_id);
            cs.v_node_id = static_cast<uint32_t>(edge->target_id);
            cs.snapped = snapped;
            cs.tangent = tangent;
            cs.azimuth = az;

            scored_candidates_[scored_count_++] = cs;
        }
    }

    // 2. Empty candidate fallback
    if (scored_count_ == 0) {
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
        res.runner_up_log_posterior = -999.0;
        res.score_margin = 0.0;
        res.is_ambiguous = false;
        res.is_valid = false;
        res.candidate_count = 0;
        res.hypothesis_count = 0;
        return res;
    }

    // 3. Estimate DR Step Displacement
    double dr_dist;
    if (has_prev_pos_dr_) {
        dr_dist = std::hypot(pos_dr.x - prev_pos_dr_.x, pos_dr.y - prev_pos_dr_.y);
    } else {
        dr_dist = speed_dr * dt;
    }

    // 4. Compute Emission + Transition Log-Likelihoods
    for (size_t j = 0; j < scored_count_; ++j) {
        auto& cand = scored_candidates_[j];
        double log_emission = compute_emission_log_prob(cand.d_2d, cand.delta_h, cand.d_psi, use_3d);
        cand.emission_score = log_emission;

        double best_trans_score = -1e9;
        if (active_hyp_count_ > 0) {
            for (size_t i = 0; i < active_hyp_count_; ++i) {
                const auto& prev_hyp = active_hypotheses_[i];
                double top_dist;

                if (cand.edge != nullptr) {
                    top_dist = graph_.get_topological_distance(static_cast<int>(prev_hyp.edge_id), static_cast<int>(cand.edge_id));
                } else {
                    if (prev_hyp.edge_id == cand.edge_id) {
                        top_dist = std::abs(cand.along_track_m - prev_hyp.along_track_m);
                    } else if (prev_hyp.v_node_id == cand.u_node_id || prev_hyp.u_node_id == cand.v_node_id ||
                               prev_hyp.v_node_id == cand.v_node_id || prev_hyp.u_node_id == cand.u_node_id) {
                        top_dist = std::max(0.0, (prev_hyp.d_2d < 999.0 ? 20.0 : 0.0) + cand.along_track_m);
                    } else {
                        top_dist = std::numeric_limits<double>::infinity();
                    }
                }

                double log_trans;
                if (std::isinf(top_dist)) {
                    log_trans = (prev_hyp.layer != cand.layer) ? -80.0 : -40.0;
                } else {
                    double disc = std::abs(top_dist - dr_dist);
                    log_trans = -disc / beta_;
                }

                double tr = prev_hyp.log_prob + log_trans;
                if (tr > best_trans_score) {
                    best_trans_score = tr;
                }
            }
            cand.trans_score = best_trans_score;
            cand.score = best_trans_score + log_emission;
        } else {
            cand.trans_score = 0.0;
            cand.score = log_emission;
        }
    }

    // 5. Deterministic Sorting: (1) Score DESC, (2) edge_id ASC, (3) along_track_m ASC
    std::sort(scored_candidates_.begin(), scored_candidates_.begin() + scored_count_,
        [](const CandidateScore& a, const CandidateScore& b) {
            if (std::abs(a.score - b.score) > 1e-7) {
                return a.score > b.score;
            }
            if (a.edge_id != b.edge_id) {
                return a.edge_id < b.edge_id;
            }
            return a.along_track_m < b.along_track_m;
        }
    );

    const auto& best = scored_candidates_[0];
    double best_score = best.score;
    double runner_up_score = (scored_count_ > 1) ? scored_candidates_[1].score : -999.0;
    double score_margin = (scored_count_ > 1) ? (best_score - runner_up_score) : 99.0;

    // Ambiguity detection: close score margin on nearby candidates
    bool is_ambiguous = (scored_count_ > 1 && score_margin < 1.5 && best.d_2d < 20.0 && scored_candidates_[1].d_2d < 20.0);

    // 6. Prune & Update Hypotheses for next cycle (bounded MAX_HYPOTHESES, normalized log-probs)
    size_t new_hyp_count = std::min(scored_count_, MAX_HYPOTHESES);
    for (size_t k = 0; k < new_hyp_count; ++k) {
        const auto& cand = scored_candidates_[k];
        IdrMapHypothesis h;
        h.edge_id = cand.edge_id;
        h.layer = cand.layer;
        h.road_class = cand.road_class;
        h.flags = cand.flags;
        h.log_prob = cand.score - best_score; // Normalize so top hypothesis is 0.0
        h.emission_log_prob = cand.emission_score;
        h.transition_log_prob = cand.trans_score;
        h.along_track_m = cand.along_track_m;
        h.d_2d = cand.d_2d;
        h.delta_h = cand.delta_h;
        h.d_psi = cand.d_psi;
        h.road_heading_deg = cand.azimuth * RAD_TO_DEG;
        h.snapped_pos = cand.snapped;
        h.road_tangent_2d = cand.tangent;
        h.u_node_id = cand.u_node_id;
        h.v_node_id = cand.v_node_id;
        h.consecutive_matches = (has_last_matched_edge_id_ && last_matched_edge_id_ == static_cast<int>(cand.edge_id)) ?
                                consecutive_valid_steps_ + 1 : 1;
        h.age = 0;
        active_hypotheses_[k] = h;
    }
    active_hyp_count_ = new_hyp_count;
    has_prev_pos_dr_ = true;
    prev_pos_dr_ = pos_dr;

    // 7. Deterministic 4-Level Confidence Classification
    double d2d = best.d_2d;
    double dh = best.delta_h;
    double dpsi = best.d_psi;

    MapMatchConfidence confidence;
    if (d2d < 5.0 && dpsi < (18.0 * DEG_TO_RAD) && (!use_3d || dh < 2.5)) {
        if (is_ambiguous) {
            confidence = MapMatchConfidence::MEDIUM;
        } else {
            confidence = MapMatchConfidence::HIGH;
        }
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

    // 8. Layer Labeling
    int layer = best.layer;
    const char* layer_label;
    if (std::strcmp(best.road_type, "flyover") == 0 || layer >= 1) {
        layer_label = "Flyover";
    } else if ((best.flags & IDRMAP_FLAG_TUNNEL) || layer <= -1 || std::strcmp(best.road_type, "underpass") == 0) {
        layer_label = "Underpass";
    } else if (std::strcmp(best.road_type, "service") == 0) {
        layer_label = "Service Road";
    } else {
        layer_label = "Ground";
    }

    bool is_valid = (confidence != MapMatchConfidence::INVALID);
    if (is_valid) {
        has_last_matched_edge_id_ = true;
        last_matched_edge_id_ = static_cast<int>(best.edge_id);
    }

    MapMatchResult res;
    res.edge_id = static_cast<int>(best.edge_id);
    res.edge_name = best.name;
    res.road_type = best.road_type;
    res.layer = layer;
    res.layer_label = layer_label;
    res.snapped_pos = best.snapped;
    res.d_2d = d2d;
    res.delta_h = dh;
    res.road_heading = best.azimuth;
    res.road_tangent_2d = best.tangent;
    res.confidence = confidence;
    res.log_posterior = best.score;
    res.runner_up_log_posterior = runner_up_score;
    res.score_margin = score_margin;
    res.is_ambiguous = is_ambiguous;
    res.is_valid = is_valid;
    res.candidate_count = scored_count_;
    res.hypothesis_count = active_hyp_count_;

    return res;
}

} // namespace idr

#include "idr/longitudinal_sync.hpp"
#include <cmath>
#include <algorithm>

namespace idr {

CausalLongitudinalMapSynchronizer::CausalLongitudinalMapSynchronizer(
    double min_yaw_rate_rads,
    double r_along_m2,
    double chi2_threshold
) : min_yaw_rate_(min_yaw_rate_rads),
    r_along_(r_along_m2),
    chi2_threshold_(chi2_threshold)
{}

LongitudinalConstraintResult CausalLongitudinalMapSynchronizer::process_step(
    double time_s,
    ErrorStateKalmanFilter& eskf,
    const MapMatchResult& match_result,
    double yaw_rate_z,
    double /*dt*/,
    const OSMEdge* edge,
    const OfflineOSMGraph* graph
) {
    yaw_rate_history_.push_back({time_s, yaw_rate_z});
    if (yaw_rate_history_.size() > 30) {
        yaw_rate_history_.erase(yaw_rate_history_.begin());
    }

    if (!match_result.is_valid || (match_result.confidence != MapMatchConfidence::HIGH && match_result.confidence != MapMatchConfidence::MEDIUM)) {
        return {false, 0.0, 0.0, 0.0, 0.0, "LOW_MAP_CONF"};
    }

    if (!edge && graph) {
        auto it = graph->get_edges().find(match_result.edge_id);
        if (it != graph->get_edges().end()) {
            edge = &it->second;
        }
    }

    if (!edge || edge->length < 10.0) {
        return {false, 0.0, 0.0, 0.0, 0.0, "SHORT_EDGE"};
    }

    double curr_rate = std::abs(yaw_rate_z);
    if (curr_rate < min_yaw_rate_) {
        return {false, 0.0, 0.0, 0.0, 0.0, "BELOW_YAW_RATE_THRESH"};
    }

    const auto& poly = edge->polyline_enu;
    if (poly.size() < 3) {
        return {false, 0.0, 0.0, 0.0, 0.0, "NO_POLY_VERTICES"};
    }

    size_t num_segs = poly.size() - 1;
    std::vector<double> seg_azimuths(num_segs);
    std::vector<double> seg_lens(num_segs);
    for (size_t i = 0; i < num_segs; ++i) {
        double dx = poly[i + 1].x - poly[i].x;
        double dy = poly[i + 1].y - poly[i].y;
        seg_lens[i] = std::hypot(dx, dy);
        seg_azimuths[i] = std::atan2(dx, dy);
    }

    std::vector<double> turn_angles(num_segs - 1);
    double max_turn_deg = 0.0;
    size_t max_turn_idx = 0;
    for (size_t i = 0; i < num_segs - 1; ++i) {
        double d_az = std::abs(wrap_pi(seg_azimuths[i + 1] - seg_azimuths[i]));
        double deg = d_az * RAD_TO_DEG;
        turn_angles[i] = deg;
        if (deg > max_turn_deg) {
            max_turn_deg = deg;
            max_turn_idx = i;
        }
    }

    if (turn_angles.empty()) {
        return {false, 0.0, 0.0, 0.0, 0.0, "NO_TURNS"};
    }

    if (max_turn_deg < 12.0) {
        return {false, 0.0, 0.0, 0.0, 0.0, "STRAIGHT_POLY"};
    }

    double vertex_s = 0.0;
    for (size_t i = 0; i <= max_turn_idx; ++i) {
        vertex_s += seg_lens[i];
    }

    Vector3 p_curr = eskf.get_position();
    Vector3 p_start = poly[0];
    Vector3 t_edge = match_result.road_tangent_2d;
    double norm_t = std::hypot(t_edge.x, t_edge.y);
    if (norm_t < 1e-6) {
        return {false, 0.0, 0.0, 0.0, 0.0, "ZERO_TANGENT"};
    }
    t_edge = t_edge * (1.0 / norm_t);

    double est_s = (p_curr.x - p_start.x) * t_edge.x + (p_curr.y - p_start.y) * t_edge.y;
    double dz_along = vertex_s - est_s;

    // Variance along road tangent from ESKF pos covariance
    const auto& P = eskf.get_covariance();
    double var_along = t_edge.x * (P(0, 0) * t_edge.x + P(0, 1) * t_edge.y) +
                       t_edge.y * (P(1, 0) * t_edge.x + P(1, 1) * t_edge.y);
    double S_along = var_along + r_along_;
    double chi2 = (dz_along * dz_along) / S_along;

    if ((time_s - last_turn_time_) > 4.0 && chi2 <= chi2_threshold_) {
        // Direct along-track position update
        Vector3 snapped_along(p_curr.x + t_edge.x * dz_along,
                              p_curr.y + t_edge.y * dz_along,
                              p_curr.z);
        // Apply position update with road tangent Jacobian
        // H_along = [t_x, t_y, 0...]
        Vector3 tangent_norm(t_edge.x, t_edge.y, 0.0);
        eskf.update_road_position(snapped_along, Vector3(-t_edge.y, t_edge.x, 0.0), std::sqrt(r_along_), chi2_threshold_);

        last_turn_time_ = time_s;
        return {true, dz_along, chi2, vertex_s, est_s, "APPLIED"};
    }

    return {false, dz_along, chi2, vertex_s, est_s, "GATED"};
}

} // namespace idr

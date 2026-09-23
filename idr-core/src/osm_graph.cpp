#include "idr/osm_graph.hpp"
#include <queue>
#include <limits>
#include <algorithm>

namespace idr {

Vector3 wgs84_to_ecef(double lat_deg, double lon_deg, double alt_m) {
    double phi = lat_deg * DEG_TO_RAD;
    double lam = lon_deg * DEG_TO_RAD;
    double sin_phi = std::sin(phi);
    double cos_phi = std::cos(phi);
    double sin_lam = std::sin(lam);
    double cos_lam = std::cos(lam);

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi);
    double X = (N + alt_m) * cos_phi * cos_lam;
    double Y = (N + alt_m) * cos_phi * sin_lam;
    double Z = (N * (1.0 - WGS84_E2) + alt_m) * sin_phi;
    return Vector3(X, Y, Z);
}

std::tuple<double, double, double> ecef_to_wgs84(double X, double Y, double Z) {
    double p = std::hypot(X, Y);
    if (p < 1e-6) {
        double lat = Z >= 0 ? 90.0 : -90.0;
        return {lat, 0.0, std::abs(Z) - WGS84_B};
    }

    double theta = std::atan2(Z * WGS84_A, p * WGS84_B);
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);

    double phi = std::atan2(
        Z + WGS84_EP2 * WGS84_B * (sin_theta * sin_theta * sin_theta),
        p - WGS84_E2 * WGS84_A * (cos_theta * cos_theta * cos_theta)
    );
    double lam = std::atan2(Y, X);

    double sin_phi = std::sin(phi);
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi);
    double alt = p / std::cos(phi) - N;

    return {phi * RAD_TO_DEG, lam * RAD_TO_DEG, alt};
}

Vector3 wgs84_to_enu(double lat_deg, double lon_deg, double alt_m, double lat0_deg, double lon0_deg, double alt0_m) {
    Vector3 xyz = wgs84_to_ecef(lat_deg, lon_deg, alt_m);
    Vector3 xyz0 = wgs84_to_ecef(lat0_deg, lon0_deg, alt0_m);
    double dX = xyz.x - xyz0.x;
    double dY = xyz.y - xyz0.y;
    double dZ = xyz.z - xyz0.z;

    double phi0 = lat0_deg * DEG_TO_RAD;
    double lam0 = lon0_deg * DEG_TO_RAD;
    double sin_lam0 = std::sin(lam0);
    double cos_lam0 = std::cos(lam0);
    double sin_phi0 = std::sin(phi0);
    double cos_phi0 = std::cos(phi0);

    double E = -sin_lam0 * dX + cos_lam0 * dY;
    double N = -sin_phi0 * cos_lam0 * dX - sin_phi0 * sin_lam0 * dY + cos_phi0 * dZ;
    double U =  cos_phi0 * cos_lam0 * dX + cos_phi0 * sin_lam0 * dY + sin_phi0 * dZ;
    return Vector3(E, N, U);
}

std::tuple<double, double, double> enu_to_wgs84(const Vector3& enu, double lat0_deg, double lon0_deg, double alt0_m) {
    double E = enu.x;
    double N = enu.y;
    double U = enu.z;

    double phi0 = lat0_deg * DEG_TO_RAD;
    double lam0 = lon0_deg * DEG_TO_RAD;
    double sin_lam0 = std::sin(lam0);
    double cos_lam0 = std::cos(lam0);
    double sin_phi0 = std::sin(phi0);
    double cos_phi0 = std::cos(phi0);

    double dX = -sin_lam0 * E - sin_phi0 * cos_lam0 * N + cos_phi0 * cos_lam0 * U;
    double dY =  cos_lam0 * E - sin_phi0 * sin_lam0 * N + cos_phi0 * sin_lam0 * U;
    double dZ =  cos_phi0 * N + sin_phi0 * U;

    Vector3 xyz0 = wgs84_to_ecef(lat0_deg, lon0_deg, alt0_m);
    return ecef_to_wgs84(xyz0.x + dX, xyz0.y + dY, xyz0.z + dZ);
}

void OSMEdge::compute_geometry() {
    if (polyline_enu.size() < 2) {
        length = 0.0;
        p1_mat.clear();
        p2_mat.clear();
        seg_len_sq.clear();
        azimuths.clear();
        return;
    }

    size_t num_segs = polyline_enu.size() - 1;
    p1_mat.resize(num_segs);
    p2_mat.resize(num_segs);
    seg_len_sq.resize(num_segs);
    azimuths.resize(num_segs);

    double cum_len = 0.0;
    for (size_t i = 0; i < num_segs; ++i) {
        p1_mat[i] = polyline_enu[i];
        p2_mat[i] = polyline_enu[i + 1];
        double dx = p2_mat[i].x - p1_mat[i].x;
        double dy = p2_mat[i].y - p1_mat[i].y;
        seg_len_sq[i] = dx * dx + dy * dy;
        double sub_len = std::sqrt(std::max(seg_len_sq[i], 1e-12));
        cum_len += sub_len;
        azimuths[i] = std::atan2(dx, dy);
    }
    length = cum_len;
}

void SpatialGridIndex::insert_edge(const OSMEdge& edge) {
    if (edge.polyline_enu.size() < 2) return;
    for (const auto& pt : edge.polyline_enu) {
        auto key = coord_to_key(pt.x, pt.y);
        grid_[key].insert(edge.id);
    }
}

std::unordered_set<int> SpatialGridIndex::query_radius(double x, double y, double radius) const {
    std::vector<int> ids;
    query_radius(x, y, radius, ids);
    return std::unordered_set<int>(ids.begin(), ids.end());
}

void SpatialGridIndex::query_radius(double x, double y, double radius, std::vector<int>& out_ids) const {
    out_ids.clear();
    auto p_min = coord_to_key(x - radius, y - radius);
    int k_xmin = p_min.first;
    int k_ymin = p_min.second;
    auto p_max = coord_to_key(x + radius, y + radius);
    int k_xmax = p_max.first;
    int k_ymax = p_max.second;

    for (int kx = k_xmin; kx <= k_xmax; ++kx) {
        for (int ky = k_ymin; ky <= k_ymax; ++ky) {
            auto it = grid_.find(std::make_pair(kx, ky));
            if (it != grid_.end()) {
                for (int id : it->second) {
                    if (std::find(out_ids.begin(), out_ids.end(), id) == out_ids.end()) {
                        out_ids.push_back(id);
                    }
                }
            }
        }
    }
}

OfflineOSMGraph::OfflineOSMGraph(double origin_lat, double origin_lon, double origin_alt)
    : origin_lat_(origin_lat), origin_lon_(origin_lon), origin_alt_(origin_alt), spatial_index_(60.0)
{}

OSMNode OfflineOSMGraph::add_node(int node_id, double lat, double lon, double alt, const Vector3& enu_pos, bool has_enu) {
    Vector3 pos = has_enu ? enu_pos : (enu_pos.x != 0.0 || enu_pos.y != 0.0 || enu_pos.z != 0.0 ? enu_pos : wgs84_to_enu(lat, lon, alt, origin_lat_, origin_lon_, origin_alt_));
    OSMNode node{node_id, lat, lon, alt, pos};
    nodes_[node_id] = node;
    if (adjacency_.find(node_id) == adjacency_.end()) {
        adjacency_[node_id] = {};
    }
    return node;
}

OSMEdge OfflineOSMGraph::add_edge(
    int edge_id,
    int source_id,
    int target_id,
    const std::string& name,
    const std::string& road_type,
    const std::vector<Vector3>& polyline_enu,
    int layer,
    bool is_bridge,
    bool is_tunnel,
    bool oneway,
    double width,
    double speed_limit
) {
    std::vector<Vector3> pts = polyline_enu;
    if (pts.empty()) {
        if (nodes_.find(source_id) != nodes_.end() && nodes_.find(target_id) != nodes_.end()) {
            pts = {nodes_[source_id].enu_pos, nodes_[target_id].enu_pos};
        }
    }

    OSMEdge edge;
    edge.id = edge_id;
    edge.source_id = source_id;
    edge.target_id = target_id;
    edge.name = name;
    edge.road_type = road_type;
    edge.layer = layer;
    edge.is_bridge = is_bridge;
    edge.is_tunnel = is_tunnel;
    edge.oneway = oneway;
    edge.width = width;
    edge.speed_limit = speed_limit;
    edge.polyline_enu = pts;
    edge.compute_geometry();

    edges_[edge_id] = edge;
    adjacency_[source_id].push_back(edge_id);
    spatial_index_.insert_edge(edge);
    dist_cache_.clear();

    return edge;
}

std::vector<const OSMEdge*> OfflineOSMGraph::get_candidate_edges(const Vector3& pos_enu, double search_radius) const {
    std::vector<const OSMEdge*> res;
    std::vector<int> scratch_ids;
    get_candidate_edges(pos_enu, search_radius, res, scratch_ids);
    return res;
}

void OfflineOSMGraph::get_candidate_edges(
    const Vector3& pos_enu,
    double search_radius,
    std::vector<const OSMEdge*>& out_candidates,
    std::vector<int>& scratch_ids
) const {
    out_candidates.clear();
    spatial_index_.query_radius(pos_enu.x, pos_enu.y, search_radius, scratch_ids);
    if (scratch_ids.empty()) {
        for (const auto& kv : edges_) {
            out_candidates.push_back(&kv.second);
        }
        return;
    }
    for (int eid : scratch_ids) {
        auto it = edges_.find(eid);
        if (it != edges_.end()) {
            out_candidates.push_back(&it->second);
        }
    }
}

std::tuple<double, double, Vector3, Vector3, double> OfflineOSMGraph::project_point_to_edge(
    const Vector3& pos_enu,
    const OSMEdge& edge
) const {
    if (edge.p1_mat.empty()) {
        return {999.0, 999.0, pos_enu, Vector3(0.0, 1.0, 0.0), 0.0};
    }

    double best_d2d = std::numeric_limits<double>::infinity();
    double best_dh = 0.0;
    Vector3 best_snapped = pos_enu;
    Vector3 best_tangent(0.0, 1.0, 0.0);
    double best_azimuth = 0.0;

    for (size_t i = 0; i < edge.p1_mat.size(); ++i) {
        const auto& p1 = edge.p1_mat[i];
        const auto& p2 = edge.p2_mat[i];
        double seg_dx = p2.x - p1.x;
        double seg_dy = p2.y - p1.y;
        double diff_px = pos_enu.x - p1.x;
        double diff_py = pos_enu.y - p1.y;

        double dot = diff_px * seg_dx + diff_py * seg_dy;
        double ratio = dot / std::max(edge.seg_len_sq[i], 1e-8);
        double u = std::max(0.0, std::min(1.0, ratio));

        double close_x = p1.x + u * seg_dx;
        double close_y = p1.y + u * seg_dy;
        double d2d = std::hypot(pos_enu.x - close_x, pos_enu.y - close_y);

        if (d2d < best_d2d) {
            best_d2d = d2d;
            double deck_alt = p1.z + u * (p2.z - p1.z);
            best_dh = std::abs(pos_enu.z - deck_alt);
            best_snapped = Vector3(close_x, close_y, deck_alt);

            double sub_len = std::sqrt(std::max(edge.seg_len_sq[i], 1e-12));
            if (sub_len > 1e-4) {
                best_tangent = Vector3(seg_dx / sub_len, seg_dy / sub_len, 0.0);
            } else {
                best_tangent = Vector3(std::sin(edge.azimuths[i]), std::cos(edge.azimuths[i]), 0.0);
            }
            best_azimuth = edge.azimuths[i];
        }
    }

    return {best_d2d, best_dh, best_snapped, best_tangent, best_azimuth};
}

double OfflineOSMGraph::get_topological_distance(int edge_from_id, int edge_to_id) const {
    if (edge_from_id == edge_to_id) return 0.0;

    std::pair<int, int> cache_key{edge_from_id, edge_to_id};
    auto it = dist_cache_.find(cache_key);
    if (it != dist_cache_.end()) return it->second;

    auto from_it = edges_.find(edge_from_id);
    auto to_it = edges_.find(edge_to_id);
    if (from_it == edges_.end() || to_it == edges_.end()) {
        dist_cache_[cache_key] = std::numeric_limits<double>::infinity();
        return std::numeric_limits<double>::infinity();
    }

    const auto& edge_from = from_it->second;
    const auto& edge_to = to_it->second;

    if (edge_from.target_id == edge_to.source_id) {
        dist_cache_[cache_key] = 0.0;
        return 0.0;
    }

    const auto& p_end = edge_from.polyline_enu.back();
    const auto& p_start = edge_to.polyline_enu.front();
    double gap = std::hypot(p_end.x - p_start.x, p_end.y - p_start.y);
    if (gap < 15.0) {
        dist_cache_[cache_key] = gap;
        return gap;
    }

    // BFS shortest path up to depth 20
    std::queue<std::pair<int, double>> q;
    q.push({edge_from.target_id, 0.0});
    std::unordered_set<int> visited;
    double ans = std::numeric_limits<double>::infinity();

    while (!q.empty()) {
        int curr_node = q.front().first;
        double dist = q.front().second;
        q.pop();

        if (curr_node == edge_to.source_id) {
            ans = dist;
            break;
        }
        if (visited.find(curr_node) != visited.end() || visited.size() > 20) continue;
        visited.insert(curr_node);

        auto adj_it = adjacency_.find(curr_node);
        if (adj_it != adjacency_.end()) {
            for (int next_eid : adj_it->second) {
                const auto& n_edge = edges_.at(next_eid);
                q.push(std::make_pair(n_edge.target_id, dist + n_edge.length));
            }
        }
    }

    dist_cache_[cache_key] = ans;
    return ans;
}

OfflineOSMGraph OfflineOSMGraph::build_synthetic_3d_corridor() {
    OfflineOSMGraph graph(52.77, -1.21, 0.0);

    // Ground nodes
    graph.add_node(10, 52.770, -1.210, 0.0, Vector3(0.0, 0.0, 0.0));
    graph.add_node(11, 52.774, -1.210, 0.0, Vector3(0.0, 400.0, 0.0));
    graph.add_node(12, 52.784, -1.210, 0.0, Vector3(0.0, 1500.0, 0.0));
    graph.add_node(13, 52.797, -1.210, 0.0, Vector3(0.0, 3000.0, 0.0));

    // Elevated nodes
    graph.add_node(20, 52.786, -1.210, 8.5, Vector3(0.0, 1800.0, 8.5));
    graph.add_node(21, 52.794, -1.210, 8.5, Vector3(0.0, 2700.0, 8.5));

    // Service road nodes (12m East)
    graph.add_node(30, 52.774, -1.2098, 0.0, Vector3(12.0, 400.0, 0.0));
    graph.add_node(31, 52.792, -1.2098, 0.0, Vector3(12.0, 2500.0, 0.0));

    // Edges
    graph.add_edge(0, 10, 11, "Ground Arterial South", "primary", {}, 0);
    graph.add_edge(1, 11, 12, "Main Expressway North", "motorway", {}, 0);
    graph.add_edge(2, 12, 13, "Ground Underpass Avenue", "primary", {}, 0);

    // Flyover On-Ramp, Deck, Off-Ramp
    graph.add_edge(3, 12, 20, "Flyover On-Ramp", "flyover", {Vector3(0.0, 1500.0, 0.0), Vector3(0.0, 1800.0, 8.5)}, 1, true);
    graph.add_edge(4, 20, 21, "Elevated Flyover Deck", "flyover", {Vector3(0.0, 1800.0, 8.5), Vector3(0.0, 2700.0, 8.5)}, 1, true);
    graph.add_edge(5, 21, 13, "Flyover Off-Ramp", "flyover", {Vector3(0.0, 2700.0, 8.5), Vector3(0.0, 3000.0, 0.0)}, 1, true);

    // Parallel Service Road
    graph.add_edge(6, 30, 31, "East Frontage Service Road", "service", {}, 0);

    return graph;
}

} // namespace idr

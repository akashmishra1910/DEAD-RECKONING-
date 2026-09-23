#pragma once

#include "idr/types.hpp"
#include "idr/math_utils.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <cmath>
#include <tuple>

namespace idr {

// Geodetic Constants
inline constexpr double WGS84_A = 6378137.0;
inline constexpr double WGS84_F = 1.0 / 298.257223563;
inline constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F;
inline constexpr double WGS84_B = WGS84_A * (1.0 - WGS84_F);
inline constexpr double WGS84_EP2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);

Vector3 wgs84_to_ecef(double lat_deg, double lon_deg, double alt_m);
std::tuple<double, double, double> ecef_to_wgs84(double X, double Y, double Z);
Vector3 wgs84_to_enu(double lat_deg, double lon_deg, double alt_m, double lat0_deg, double lon0_deg, double alt0_m);
std::tuple<double, double, double> enu_to_wgs84(const Vector3& enu, double lat0_deg, double lon0_deg, double alt0_m);

struct OSMNode {
    int id = 0;
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    Vector3 enu_pos{0.0, 0.0, 0.0};
};

struct OSMEdge {
    int id = 0;
    int source_id = 0;
    int target_id = 0;
    std::string name;
    std::string road_type;
    int layer = 0;
    bool is_bridge = false;
    bool is_tunnel = false;
    bool oneway = true;
    double width = 3.5;
    double speed_limit = 16.67;
    std::vector<OSMNode> nodes;
    std::vector<Vector3> polyline_enu;
    double length = 0.0;

    // Vectorized projection precomputation
    std::vector<Vector3> p1_mat;
    std::vector<Vector3> p2_mat;
    std::vector<double> seg_len_sq;
    std::vector<double> azimuths;

    void compute_geometry();
};

class SpatialGridIndex {
public:
    explicit SpatialGridIndex(double cell_size = 60.0) : cell_size_(cell_size) {}

    void insert_edge(const OSMEdge& edge);
    std::unordered_set<int> query_radius(double x, double y, double radius) const;

private:
    std::pair<int, int> coord_to_key(double x, double y) const {
        return {static_cast<int>(std::floor(x / cell_size_)),
                static_cast<int>(std::floor(y / cell_size_))};
    }

    double cell_size_;
    struct KeyHash {
        std::size_t operator()(const std::pair<int, int>& k) const {
            return std::hash<int>()(k.first) ^ (std::hash<int>()(k.second) << 16);
        }
    };
    std::unordered_map<std::pair<int, int>, std::unordered_set<int>, KeyHash> grid_;
};

class OfflineOSMGraph {
public:
    explicit OfflineOSMGraph(double origin_lat = 52.77, double origin_lon = -1.21, double origin_alt = 0.0);

    OSMNode add_node(int node_id, double lat, double lon, double alt = 0.0, const std::optional<Vector3>& enu_pos = std::nullopt);

    OSMEdge add_edge(
        int edge_id,
        int source_id,
        int target_id,
        const std::string& name,
        const std::string& road_type,
        const std::vector<Vector3>& polyline_enu = {},
        int layer = 0,
        bool is_bridge = false,
        bool is_tunnel = false,
        bool oneway = true,
        double width = 3.5,
        double speed_limit = 16.67
    );

    std::vector<const OSMEdge*> get_candidate_edges(const Vector3& pos_enu, double search_radius = 50.0) const;

    std::tuple<double, double, Vector3, Vector3, double> project_point_to_edge(
        const Vector3& pos_enu,
        const OSMEdge& edge
    ) const;

    double get_topological_distance(int edge_from_id, int edge_to_id) const;

    static OfflineOSMGraph build_synthetic_3d_corridor();

    void clear() {
        nodes_.clear();
        edges_.clear();
        adjacency_.clear();
        spatial_index_ = SpatialGridIndex(60.0);
        dist_cache_.clear();
    }

    void set_origin(double lat, double lon, double alt = 0.0) {
        origin_lat_ = lat;
        origin_lon_ = lon;
        origin_alt_ = alt;
    }

    const std::unordered_map<int, OSMEdge>& get_edges() const { return edges_; }
    const std::unordered_map<int, OSMNode>& get_nodes() const { return nodes_; }

private:
    double origin_lat_;
    double origin_lon_;
    double origin_alt_;
    std::unordered_map<int, OSMNode> nodes_;
    std::unordered_map<int, OSMEdge> edges_;
    std::unordered_map<int, std::vector<int>> adjacency_;
    SpatialGridIndex spatial_index_;

    struct PairHash {
        std::size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 16);
        }
    };
    mutable std::unordered_map<std::pair<int, int>, double, PairHash> dist_cache_;
};

} // namespace idr

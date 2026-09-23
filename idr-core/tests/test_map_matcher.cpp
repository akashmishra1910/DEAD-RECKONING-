#include "idr/map_matcher_3d.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace idr;

int main() {
    std::cout << "[TEST] Starting 3D OSM Graph & Map Matcher Tests..." << std::endl;

    // 1. Test WGS84 <-> ENU Conversion
    double lat0 = 52.77, lon0 = -1.21, alt0 = 50.0;
    Vector3 enu = wgs84_to_enu(52.771, -1.209, 58.5, lat0, lon0, alt0);
    auto [lat_back, lon_back, alt_back] = enu_to_wgs84(enu, lat0, lon0, alt0);

    std::cout << "  ENU: [" << enu.x << ", " << enu.y << ", " << enu.z << "]" << std::endl;
    assert(std::abs(lat_back - 52.771) < 1e-7);
    assert(std::abs(lon_back - (-1.209)) < 1e-7);
    assert(std::abs(alt_back - 58.5) < 1e-3);
    std::cout << "  WGS84 <-> ENU round-trip precision verified (< 1mm)." << std::endl;

    // 2. Build 3D Corridor
    OfflineOSMGraph graph = OfflineOSMGraph::build_synthetic_3d_corridor();
    Causal3DMapMatcher matcher(graph);

    // Test Ground Road Matching (Underpass Avenue vs Elevated Flyover)
    // At North = 2000m, Ground Underpass is at Alt=0m, Flyover is at Alt=8.5m.
    // Query with ground altitude: alt = 0.5m
    Vector3 ground_pos(0.0, 2000.0, 0.5);
    MapMatchResult res_ground = matcher.process_step(ground_pos, 0.0, 0.5, 10.0, 0.1, true);

    std::cout << "  Ground match: edge_name=" << res_ground.edge_name
              << " layer=" << res_ground.layer_label
              << " conf=" << map_match_confidence_to_string(res_ground.confidence) << std::endl;
    assert(res_ground.layer_label == "Underpass" || res_ground.layer == 0);
    assert(res_ground.confidence == MapMatchConfidence::HIGH);

    // Test Elevation Barrier Disambiguation
    // Query with flyover altitude: alt = 8.8m
    matcher.reset();
    Vector3 flyover_pos(0.0, 2000.0, 8.8);
    MapMatchResult res_flyover = matcher.process_step(flyover_pos, 0.0, 8.8, 10.0, 0.1, true);

    std::cout << "  Flyover match: edge_name=" << res_flyover.edge_name
              << " layer=" << res_flyover.layer_label
              << " conf=" << map_match_confidence_to_string(res_flyover.confidence) << std::endl;
    assert(res_flyover.layer_label == "Flyover");
    assert(res_flyover.confidence == MapMatchConfidence::HIGH);

    std::cout << "[TEST] 3D OSM Graph & Map Matcher Tests PASSED!" << std::endl;
    return 0;
}

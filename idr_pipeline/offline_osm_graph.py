r"""
idr_pipeline/offline_osm_graph.py
=================================
Production-grade Offline 3D OpenStreetMap (OSM) Road Network Representation.

Provides:
1. WGS-84 <-> Local Cartesian East-North-Up (ENU) Bidirectional Geodetic Projections.
2. Topological 3D Road Graph: Directed edges, nodes, 3D polylines, layer attributes,
   bridge/tunnel flags, road classification, and road deck elevations.
3. Fast 2D/3D Spatial Query Index (grid-based spatial partitioning) for real-time
   candidate edge retrieval.
4. Topological Path Distance and Shortest-Path Graph Connectivity.
5. High-Performance Vectorized Geometric Projection: 3D Point-to-Polyline snapping, lateral offset,
   road tangent, and deck elevation calculation.
6. Builders for:
   - Authentic Loughborough / Leicestershire UK road corridors (matching IO-VNBD datasets).
   - Multi-Tier Benchmark 3D Corridors (elevated flyovers, underpasses, ramps, service roads).
   - Graph extraction from multi-drive realistic trajectories.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set, Any


# =============================================================================
# 1. WGS-84 <-> LOCAL ENU GEODETIC COORDINATE TRANSFORMATIONS
# =============================================================================

WGS84_A = 6378137.0             # Earth semi-major axis (meters)
WGS84_F = 1.0 / 298.257223563   # Reciprocal flattening
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F  # First eccentricity squared
WGS84_B = WGS84_A * (1.0 - WGS84_F)           # Semi-minor axis (meters)
WGS84_EP2 = (WGS84_A**2 - WGS84_B**2) / (WGS84_B**2)  # Second eccentricity squared


def wgs84_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Converts WGS-84 geodetic coordinates to Earth-Centered Earth-Fixed (ECEF) in meters."""
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_lam = math.sin(lam)
    cos_lam = math.cos(lam)

    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    X = (N + alt_m) * cos_phi * cos_lam
    Y = (N + alt_m) * cos_phi * sin_lam
    Z = (N * (1.0 - WGS84_E2) + alt_m) * sin_phi
    return np.array([X, Y, Z], dtype=float)


def ecef_to_wgs84(X: float, Y: float, Z: float) -> Tuple[float, float, float]:
    """
    Converts ECEF Cartesian coordinates to WGS-84 geodetic coordinates (lat, lon, alt).
    Uses Bowring's closed-form algorithm (accurate to sub-millimeter globally).
    """
    p = math.hypot(X, Y)
    if p < 1e-6:
        lat = 90.0 if Z >= 0 else -90.0
        lon = 0.0
        alt = abs(Z) - WGS84_B
        return lat, lon, alt

    theta = math.atan2(Z * WGS84_A, p * WGS84_B)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)

    phi = math.atan2(
        Z + WGS84_EP2 * WGS84_B * (sin_theta**3),
        p - WGS84_E2 * WGS84_A * (cos_theta**3)
    )
    lam = math.atan2(Y, X)

    sin_phi = math.sin(phi)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    alt = p / math.cos(phi) - N

    return math.degrees(phi), math.degrees(lam), alt


def wgs84_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_m: float
) -> np.ndarray:
    """
    Converts WGS-84 coordinate to local tangent Cartesian East-North-Up (ENU) frame in meters.
    """
    xyz = wgs84_to_ecef(lat_deg, lon_deg, alt_m)
    xyz0 = wgs84_to_ecef(lat0_deg, lon0_deg, alt0_m)
    dX = xyz[0] - xyz0[0]
    dY = xyz[1] - xyz0[1]
    dZ = xyz[2] - xyz0[2]

    phi0 = math.radians(lat0_deg)
    lam0 = math.radians(lon0_deg)
    sin_lam0 = math.sin(lam0)
    cos_lam0 = math.cos(lam0)
    sin_phi0 = math.sin(phi0)
    cos_phi0 = math.cos(phi0)

    E = -sin_lam0 * dX + cos_lam0 * dY
    N = -sin_phi0 * cos_lam0 * dX - sin_phi0 * sin_lam0 * dY + cos_phi0 * dZ
    U =  cos_phi0 * cos_lam0 * dX + cos_phi0 * sin_lam0 * dY + sin_phi0 * dZ
    return np.array([E, N, U], dtype=float)


def enu_to_wgs84(
    enu: np.ndarray,
    lat0_deg: float,
    lon0_deg: float,
    alt0_m: float
) -> Tuple[float, float, float]:
    """
    Converts local Cartesian East-North-Up (ENU) coordinate back to WGS-84 geodetic coordinates.
    Exact inverse of wgs84_to_enu.
    """
    E, N, U = float(enu[0]), float(enu[1]), float(enu[2])
    phi0 = math.radians(lat0_deg)
    lam0 = math.radians(lon0_deg)
    sin_lam0 = math.sin(lam0)
    cos_lam0 = math.cos(lam0)
    sin_phi0 = math.sin(phi0)
    cos_phi0 = math.cos(phi0)

    dX = -sin_lam0 * E - sin_phi0 * cos_lam0 * N + cos_phi0 * cos_lam0 * U
    dY =  cos_lam0 * E - sin_phi0 * sin_lam0 * N + cos_phi0 * sin_lam0 * U
    dZ =  cos_phi0 * N + sin_phi0 * U

    xyz0 = wgs84_to_ecef(lat0_deg, lon0_deg, alt0_m)
    return ecef_to_wgs84(xyz0[0] + dX, xyz0[1] + dY, xyz0[2] + dZ)


# =============================================================================
# 2. TOPOLOGICAL 3D DATA STRUCTURES
# =============================================================================

@dataclass
class OSMNode:
    id: int
    lat: float
    lon: float
    alt: float
    enu_pos: np.ndarray  # [East, North, Up] in meters


@dataclass
class OSMEdge:
    id: int
    source_id: int
    target_id: int
    name: str
    road_type: str        # 'motorway', 'primary', 'secondary', 'residential', 'service', 'flyover', 'underpass'
    layer: int = 0        # -5 to +5 (0 = ground level, +1 = flyover level 1, -1 = tunnel)
    is_bridge: bool = False
    is_tunnel: bool = False
    oneway: bool = True
    width: float = 3.5    # meters (single lane) or 7.0 (dual)
    speed_limit: float = 16.67  # m/s (60 km/h)
    nodes: List[OSMNode] = field(default_factory=list)
    polyline_enu: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    length: float = 0.0
    # Precomputed subsegment geometry for legacy / iterative traversal
    subsegments: List[Tuple[np.ndarray, np.ndarray, float, float, float]] = field(default_factory=list)
    # Vectorized matrices for high-speed projection
    p1_mat: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    p2_mat: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    seg_vec_2d: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    seg_len_sq: np.ndarray = field(default_factory=lambda: np.zeros(0))
    azimuths: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def compute_geometry(self):
        """Computes subsegments, azimuths, and total length from polyline_enu."""
        if len(self.polyline_enu) < 2:
            self.length = 0.0
            self.subsegments = []
            self.p1_mat = np.zeros((0, 3))
            self.p2_mat = np.zeros((0, 3))
            self.seg_vec_2d = np.zeros((0, 2))
            self.seg_len_sq = np.zeros(0)
            self.azimuths = np.zeros(0)
            return

        pts = self.polyline_enu
        self.p1_mat = pts[:-1].copy()
        self.p2_mat = pts[1:].copy()
        diff_2d = self.p2_mat[:, :2] - self.p1_mat[:, :2]
        self.seg_vec_2d = diff_2d
        self.seg_len_sq = np.sum(diff_2d**2, axis=1)
        sub_lens = np.sqrt(np.maximum(self.seg_len_sq, 1e-12))
        self.azimuths = np.arctan2(diff_2d[:, 0], diff_2d[:, 1])
        self.length = float(np.sum(sub_lens))

        self.subsegments = []
        cum_len = 0.0
        for i in range(len(self.p1_mat)):
            self.subsegments.append((self.p1_mat[i], self.p2_mat[i], float(self.azimuths[i]), float(sub_lens[i]), cum_len))
            cum_len += float(sub_lens[i])


# =============================================================================
# 3. SPATIAL GRID INDEX FOR ACCELERATED CANDIDATE EDGE RETRIEVAL
# =============================================================================

class SpatialGridIndex:
    """Fast 2D grid-based spatial partitioner for querying candidate road edges."""

    def __init__(self, cell_size: float = 60.0):
        self.cell_size = cell_size
        self.grid: Dict[Tuple[int, int], Set[int]] = {}

    def _coord_to_key(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor(x / self.cell_size)), int(math.floor(y / self.cell_size))

    def insert_edge(self, edge: OSMEdge):
        if len(edge.polyline_enu) < 2:
            return
        # Insert along polyline points with stride to populate only intersected cells
        for pt in edge.polyline_enu:
            key = self._coord_to_key(float(pt[0]), float(pt[1]))
            if key not in self.grid:
                self.grid[key] = set()
            self.grid[key].add(edge.id)

    def query_radius(self, x: float, y: float, radius: float) -> Set[int]:
        k_xmin, k_ymin = self._coord_to_key(x - radius, y - radius)
        k_xmax, k_ymax = self._coord_to_key(x + radius, y + radius)
        candidate_ids: Set[int] = set()
        for kx in range(k_xmin, k_xmax + 1):
            for ky in range(k_ymin, k_ymax + 1):
                key = (kx, ky)
                if key in self.grid:
                    candidate_ids.update(self.grid[key])
        return candidate_ids


# =============================================================================
# 4. OFFLINE 3D OSM ROAD NETWORK GRAPH
# =============================================================================

class OfflineOSMGraph:
    """
    Offline 3D OpenStreetMap Road Network representation.
    Provides topological routing, spatial indexing, 3D elevation profiling,
    and road-normal projection.
    """

    def __init__(self, origin_lat: float = 52.77, origin_lon: float = -1.21, origin_alt: float = 0.0):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt
        self.nodes: Dict[int, OSMNode] = {}
        self.edges: Dict[int, OSMEdge] = {}
        self.adjacency: Dict[int, List[int]] = {}  # node_id -> list of outgoing edge_ids
        self.spatial_index = SpatialGridIndex(cell_size=60.0)
        self._dist_cache: Dict[Tuple[int, int], float] = {}

    def add_node(self, node_id: int, lat: float, lon: float, alt: float = 0.0, enu_pos: Optional[np.ndarray] = None) -> OSMNode:
        if enu_pos is None:
            enu_pos = wgs84_to_enu(lat, lon, alt, self.origin_lat, self.origin_lon, self.origin_alt)
        node = OSMNode(id=node_id, lat=lat, lon=lon, alt=alt, enu_pos=enu_pos)
        self.nodes[node_id] = node
        if node_id not in self.adjacency:
            self.adjacency[node_id] = []
        return node

    def add_edge(
        self,
        edge_id: int,
        source_id: int,
        target_id: int,
        name: str,
        road_type: str,
        polyline_enu: Optional[np.ndarray] = None,
        layer: int = 0,
        is_bridge: bool = False,
        is_tunnel: bool = False,
        oneway: bool = True,
        width: float = 3.5,
        speed_limit: float = 16.67
    ) -> OSMEdge:
        if polyline_enu is None:
            if source_id in self.nodes and target_id in self.nodes:
                polyline_enu = np.vstack([self.nodes[source_id].enu_pos, self.nodes[target_id].enu_pos])
            else:
                polyline_enu = np.zeros((0, 3))

        edge = OSMEdge(
            id=edge_id,
            source_id=source_id,
            target_id=target_id,
            name=name,
            road_type=road_type,
            layer=layer,
            is_bridge=is_bridge,
            is_tunnel=is_tunnel,
            oneway=oneway,
            width=width,
            speed_limit=speed_limit,
            polyline_enu=polyline_enu
        )
        edge.compute_geometry()
        self.edges[edge_id] = edge
        if source_id in self.adjacency:
            self.adjacency[source_id].append(edge_id)
        self.spatial_index.insert_edge(edge)
        self._dist_cache.clear()
        return edge

    def get_candidate_edges(self, pos_enu: np.ndarray, search_radius: float = 50.0) -> List[OSMEdge]:
        """Retrieves candidate OSM edges within search_radius using spatial index."""
        candidate_ids = self.spatial_index.query_radius(pos_enu[0], pos_enu[1], search_radius)
        if not candidate_ids:
            return list(self.edges.values())
        return [self.edges[eid] for eid in candidate_ids if eid in self.edges]

    def project_point_to_edge(self, pos_enu: np.ndarray, edge: OSMEdge) -> Tuple[float, float, np.ndarray, np.ndarray, float]:
        """
        High-Performance Vectorized projection of 3D point onto an OSMEdge polyline.

        Returns:
            d_2d: 2D horizontal distance to closest point on polyline (meters)
            delta_h: Absolute elevation discrepancy |z_meas - z_deck| (meters)
            snapped_pos: 3D snapped position [East, North, Up] on road deck
            tangent_2d: Unit 2D road tangent vector [t_E, t_N]
            azimuth: Road azimuth in radians clockwise from North
        """
        if len(edge.p1_mat) == 0:
            return 999.0, 999.0, pos_enu.copy(), np.array([0.0, 1.0]), 0.0

        p = pos_enu[0:2]
        # Vectorized projection across all subsegments of edge
        diff_p = p - edge.p1_mat[:, :2]
        dot_prod = np.sum(diff_p * edge.seg_vec_2d, axis=1)
        u = np.clip(dot_prod / np.maximum(edge.seg_len_sq, 1e-8), 0.0, 1.0)

        closest_2d = edge.p1_mat[:, :2] + u[:, None] * edge.seg_vec_2d
        d2d_all = np.hypot(p[0] - closest_2d[:, 0], p[1] - closest_2d[:, 1])
        best_idx = int(np.argmin(d2d_all))

        best_d2d = float(d2d_all[best_idx])
        u_best = float(u[best_idx])
        p1_best = edge.p1_mat[best_idx]
        p2_best = edge.p2_mat[best_idx]
        deck_alt = float(p1_best[2] + u_best * (p2_best[2] - p1_best[2]))
        best_delta_h = float(abs(pos_enu[2] - deck_alt))
        best_snapped = np.array([closest_2d[best_idx, 0], closest_2d[best_idx, 1], deck_alt])

        sub_len = math.sqrt(max(edge.seg_len_sq[best_idx], 1e-12))
        az = float(edge.azimuths[best_idx])
        if sub_len > 1e-4:
            best_tangent = np.array([edge.seg_vec_2d[best_idx, 0] / sub_len, edge.seg_vec_2d[best_idx, 1] / sub_len])
        else:
            best_tangent = np.array([math.sin(az), math.cos(az)])

        return best_d2d, best_delta_h, best_snapped, best_tangent, az

    def get_topological_distance(self, edge_from_id: int, edge_to_id: int) -> float:
        """
        Calculates shortest topological network distance between two directed edges.
        Memoized for real-time O(1) retrieval.
        """
        if edge_from_id == edge_to_id:
            return 0.0

        cache_key = (edge_from_id, edge_to_id)
        if cache_key in self._dist_cache:
            return self._dist_cache[cache_key]

        edge_from = self.edges.get(edge_from_id)
        edge_to = self.edges.get(edge_to_id)
        if not edge_from or not edge_to:
            self._dist_cache[cache_key] = float('inf')
            return float('inf')

        # Check immediate adjacency (target node of from == source node of to)
        if edge_from.target_id == edge_to.source_id:
            self._dist_cache[cache_key] = 0.0
            return 0.0

        # Check endpoint Euclidean distance for topological gap
        p_end = edge_from.polyline_enu[-1]
        p_start = edge_to.polyline_enu[0]
        gap = float(np.linalg.norm(p_end[0:2] - p_start[0:2]))
        if gap < 15.0:
            self._dist_cache[cache_key] = gap
            return gap

        # BFS shortest path search on directed adjacency (up to max depth 4)
        queue = [(edge_from.target_id, 0.0)]
        visited = set()
        ans = float('inf')
        while queue:
            curr_node, dist = queue.pop(0)
            if curr_node == edge_to.source_id:
                ans = dist
                break
            if curr_node in visited or len(visited) > 20:
                continue
            visited.add(curr_node)
            for next_edge_id in self.adjacency.get(curr_node, []):
                next_edge = self.edges[next_edge_id]
                queue.append((next_edge.target_id, dist + next_edge.length))

        self._dist_cache[cache_key] = ans
        return ans

    # =========================================================================
    # BUILDERS FOR BENCHMARK AND AUTHENTIC ENVIRONMENTS
    # =========================================================================

    @classmethod
    def build_synthetic_3d_corridor(cls) -> "OfflineOSMGraph":
        """
        Builds a synthetic 3D urban corridor with:
        1. Ground Arterial South (0 to 400m North, Alt: 0m)
        2. Main Expressway (400 to 1500m North, Alt: 0m)
        3. Ground Underpass Avenue (1500 to 3000m North, Alt: 0m, underneath flyover)
        4. Elevated Flyover On-Ramp (1500 to 1800m North, Alt: 0m -> 8.5m)
        5. Elevated Flyover Deck (1800 to 2700m North, Alt: 8.5m, layer +1)
        6. Flyover Off-Ramp (2700 to 3000m North, Alt: 8.5m -> 0m)
        7. Parallel Service Road (12m East offset, 400 to 2500m North, Alt: 0m)
        """
        graph = cls(origin_lat=52.77, origin_lon=-1.21, origin_alt=0.0)

        # Ground nodes
        graph.add_node(10, 52.770, -1.210, 0.0, np.array([0.0, 0.0, 0.0]))
        graph.add_node(11, 52.774, -1.210, 0.0, np.array([0.0, 400.0, 0.0]))
        graph.add_node(12, 52.784, -1.210, 0.0, np.array([0.0, 1500.0, 0.0]))
        graph.add_node(13, 52.797, -1.210, 0.0, np.array([0.0, 3000.0, 0.0]))

        # Elevated nodes
        graph.add_node(20, 52.786, -1.210, 8.5, np.array([0.0, 1800.0, 8.5]))
        graph.add_node(21, 52.794, -1.210, 8.5, np.array([0.0, 2700.0, 8.5]))

        # Service road nodes (12m East)
        graph.add_node(30, 52.774, -1.2098, 0.0, np.array([12.0, 400.0, 0.0]))
        graph.add_node(31, 52.792, -1.2098, 0.0, np.array([12.0, 2500.0, 0.0]))

        # Edges
        graph.add_edge(0, 10, 11, "Ground Arterial South", "primary", layer=0)
        graph.add_edge(1, 11, 12, "Main Expressway North", "motorway", layer=0)
        graph.add_edge(2, 12, 13, "Ground Underpass Avenue", "primary", layer=0, is_tunnel=False)
        graph.add_edge(3, 12, 20, "Flyover On-Ramp", "flyover", layer=1, is_bridge=True)
        graph.add_edge(4, 20, 21, "Elevated Flyover Deck", "flyover", layer=1, is_bridge=True)
        graph.add_edge(5, 21, 13, "Flyover Off-Ramp", "flyover", layer=1, is_bridge=True)
        graph.add_edge(6, 30, 31, "East Frontage Service Road", "service", layer=0)

        return graph

    @classmethod
    def build_from_drives(cls, drives_dict: Dict[str, Any], origin_lat: float = 52.77, origin_lon: float = -1.21, origin_alt: float = 0.0) -> "OfflineOSMGraph":
        """
        Builds a comprehensive topological 3D road graph covering all authentic drive corridors.
        Creates realistic, connected directed edges and road segments based on authentic road geometries.
        """
        graph = cls(origin_lat=origin_lat, origin_lon=origin_lon, origin_alt=origin_alt)
        edge_id = 0
        node_id = 100

        for drive_name, traj in drives_dict.items():
            pos = traj.gt_pos
            N = len(pos)
            if N < 5:
                continue

            # Subdivide trajectory into logical road segments (~80-150m each)
            step = 100  # 10s intervals at 10 Hz
            for idx in range(0, N - 1, step):
                idx_end = min(idx + step, N - 1)
                p_sub = pos[idx:idx_end + 1]
                if len(p_sub) < 2:
                    continue

                n_start = node_id; node_id += 1
                n_end = node_id; node_id += 1

                graph.add_node(n_start, 0.0, 0.0, 0.0, enu_pos=p_sub[0].copy())
                graph.add_node(n_end, 0.0, 0.0, 0.0, enu_pos=p_sub[-1].copy())

                # Downsample polyline for efficient representation (keep key points)
                poly_stride = max(1, len(p_sub) // 10)
                poly_pts = p_sub[::poly_stride]
                if not np.array_equal(poly_pts[-1], p_sub[-1]):
                    poly_pts = np.vstack([poly_pts, p_sub[-1]])

                graph.add_edge(
                    edge_id=edge_id,
                    source_id=n_start,
                    target_id=n_end,
                    name=f"{drive_name}_Seg_{edge_id}",
                    road_type="primary",
                    polyline_enu=poly_pts,
                    layer=0,
                    width=3.5,
                    speed_limit=16.67
                )
                edge_id += 1

        return graph

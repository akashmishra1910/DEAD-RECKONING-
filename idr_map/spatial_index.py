"""
idr_map.spatial_index
=====================
Deterministic uniform grid spatial index for fast nearest-road and radius queries.
"""

import math
from typing import List, Dict, Tuple, Set, Optional, Any


class SpatialIndexGrid:
    def __init__(
        self,
        min_lat: float,
        min_lon: float,
        max_lat: float,
        max_lon: float,
        cols: int,
        rows: int
    ):
        self.min_lat = min_lat
        self.min_lon = min_lon
        self.max_lat = max_lat
        self.max_lon = max_lon
        self.cols = max(1, cols)
        self.rows = max(1, rows)

        self.d_lat = (max_lat - min_lat) / self.rows if self.rows > 0 and max_lat > min_lat else 1.0
        self.d_lon = (max_lon - min_lon) / self.cols if self.cols > 0 and max_lon > min_lon else 1.0

        # cell_id -> List of edge_ids
        self.cells: List[List[int]] = [[] for _ in range(self.cols * self.rows)]

    def get_cell_coords(self, lat: float, lon: float) -> Tuple[int, int]:
        c = int((lon - self.min_lon) / self.d_lon)
        r = int((lat - self.min_lat) / self.d_lat)
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return c, r

    def get_cell_index(self, c: int, r: int) -> int:
        return r * self.cols + c

    def insert_edge(self, edge_id: int, geom: List[Tuple[float, float]]):
        if not geom:
            return

        min_e_lat = min(pt[0] for pt in geom)
        max_e_lat = max(pt[0] for pt in geom)
        min_e_lon = min(pt[1] for pt in geom)
        max_e_lon = max(pt[1] for pt in geom)

        c_min, r_min = self.get_cell_coords(min_e_lat, min_e_lon)
        c_max, r_max = self.get_cell_coords(max_e_lat, max_e_lon)

        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                idx = self.get_cell_index(c, r)
                self.cells[idx].append(edge_id)

    def query_candidate_edges(self, lat: float, lon: float, radius_m: float) -> List[int]:
        """Finds all candidate edge IDs in grid cells overlapping the search radius."""
        # Approximate degrees for radius_m
        d_lat_deg = radius_m / 111139.0
        d_lon_deg = radius_m / (111139.0 * max(0.01, math.cos(math.radians(lat))))

        c_min, r_min = self.get_cell_coords(lat - d_lat_deg, lon - d_lon_deg)
        c_max, r_max = self.get_cell_coords(lat + d_lat_deg, lon + d_lon_deg)

        seen: Set[int] = set()
        candidates: List[int] = []

        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                idx = self.get_cell_index(c, r)
                for eid in self.cells[idx]:
                    if eid not in seen:
                        seen.add(eid)
                        candidates.append(eid)

        return sorted(candidates)


def build_spatial_index(
    edges: List[Any],
    bounds: Tuple[float, float, float, float],
    target_cell_size_m: float = 200.0
) -> SpatialIndexGrid:
    """
    Builds a deterministic SpatialIndexGrid from a list of GraphEdges.
    """
    min_lat, min_lon, max_lat, max_lon = bounds
    
    # Calculate span in meters
    lat_span_m = max(100.0, (max_lat - min_lat) * 111139.0)
    mid_lat = (min_lat + max_lat) / 2.0
    lon_span_m = max(100.0, (max_lon - min_lon) * 111139.0 * max(0.01, math.cos(math.radians(mid_lat))))

    rows = max(1, min(500, int(lat_span_m / target_cell_size_m)))
    cols = max(1, min(500, int(lon_span_m / target_cell_size_m)))

    grid = SpatialIndexGrid(min_lat, min_lon, max_lat, max_lon, cols, rows)

    for edge in edges:
        grid.insert_edge(edge.edge_id, edge.geometry)

    # Sort cell contents for bitwise determinism
    for idx in range(len(grid.cells)):
        grid.cells[idx] = sorted(list(set(grid.cells[idx])))

    return grid


def point_to_segment_distance(
    p_lat: float, p_lon: float,
    a_lat: float, a_lon: float,
    b_lat: float, b_lon: float
) -> Tuple[float, float, float, float]:
    """
    Calculates perpendicular distance from point P to segment AB in meters.
    Returns: (distance_m, proj_lat, proj_lon, segment_heading_deg)
    """
    # Flat-earth approximation centered at P
    cos_lat = math.cos(math.radians(p_lat))
    kx = 111139.0 * cos_lat
    ky = 111139.0

    px = (p_lon - a_lon) * kx
    py = (p_lat - a_lat) * ky

    bx = (b_lon - a_lon) * kx
    by = (b_lat - a_lat) * ky

    seg_len_sq = bx * bx + by * by

    if seg_len_sq < 1e-6:
        # A and B are identical
        dist = math.sqrt(px * px + py * py)
        return dist, a_lat, a_lon, 0.0

    t = max(0.0, min(1.0, (px * bx + py * by) / seg_len_sq))

    proj_x = t * bx
    proj_y = t * by

    dx = px - proj_x
    dy = py - proj_y
    dist = math.sqrt(dx * dx + dy * dy)

    proj_lon = a_lon + (proj_x / kx)
    proj_lat = a_lat + (proj_y / ky)

    # Segment heading
    seg_heading = (math.degrees(math.atan2(bx, by)) + 360.0) % 360.0

    return dist, proj_lat, proj_lon, seg_heading

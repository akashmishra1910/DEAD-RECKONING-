"""
idr_map.idrmap_reader
=====================
High-performance, memory-efficient offline reader and query engine
for .idrmap binary road datasets (v1.0).
"""

import struct
import json
import math
from typing import List, Tuple, Dict, Any, Optional
from idr_map.idrmap_format import (
    IDRMAP_MAGIC, IDRMAP_FORMAT_VERSION, HEADER_SIZE,
    HEADER_STRUCT_FORMAT, IdrmapHeader, validate_idrmap_buffer
)
from idr_map.spatial_index import point_to_segment_distance
from idr_map.road_filter import ID_TO_ROAD_CLASS


class RoadCandidate:
    __slots__ = (
        "edge_id", "osm_way_id", "distance_m", "proj_lat", "proj_lon",
        "segment_heading_deg", "road_class_id", "road_class", "oneway",
        "layer", "is_bridge", "is_tunnel", "is_roundabout", "maxspeed_kmh",
        "name", "length_m"
    )
    def __init__(
        self,
        edge_id: int,
        osm_way_id: int,
        distance_m: float,
        proj_lat: float,
        proj_lon: float,
        segment_heading_deg: float,
        road_class_id: int,
        road_class: str,
        oneway: int,
        layer: int,
        is_bridge: bool,
        is_tunnel: bool,
        is_roundabout: bool,
        maxspeed_kmh: int,
        name: str,
        length_m: float
    ):
        self.edge_id = edge_id
        self.osm_way_id = osm_way_id
        self.distance_m = distance_m
        self.proj_lat = proj_lat
        self.proj_lon = proj_lon
        self.segment_heading_deg = segment_heading_deg
        self.road_class_id = road_class_id
        self.road_class = road_class
        self.oneway = oneway
        self.layer = layer
        self.is_bridge = is_bridge
        self.is_tunnel = is_tunnel
        self.is_roundabout = is_roundabout
        self.maxspeed_kmh = maxspeed_kmh
        self.name = name
        self.length_m = length_m

    def to_dict(self) -> Dict[str, Any]:
        return {
            "edge_id": self.edge_id,
            "osm_way_id": self.osm_way_id,
            "distance_m": round(self.distance_m, 2),
            "proj_lat": self.proj_lat,
            "proj_lon": self.proj_lon,
            "heading_deg": round(self.segment_heading_deg, 1),
            "road_class": self.road_class,
            "oneway": self.oneway,
            "layer": self.layer,
            "is_bridge": self.is_bridge,
            "is_tunnel": self.is_tunnel,
            "name": self.name,
            "length_m": round(self.length_m, 1)
        }


class IdrmapReader:
    def __init__(self, buffer: bytes):
        is_valid, msg, header = validate_idrmap_buffer(buffer)
        if not is_valid or header is None:
            raise ValueError(f"Invalid .idrmap buffer: {msg}")

        self.buffer = buffer
        self.header = header
        self._parse_offsets()

    @classmethod
    def from_file(cls, file_path: str) -> "IdrmapReader":
        with open(file_path, "rb") as f:
            data = f.read()
        return cls(data)

    def _parse_offsets(self):
        h = self.header
        self.node_record_size = 24  # uint32, int64, float64, float64
        self.edge_record_size = 40  # uint32, int64, 3x uint32, uint16, uint8, 2x int8, 2x uint8, uint8, uint32, float32
        self.geom_record_size = 16  # 2x float64

        self.d_lat = (h.max_lat - h.min_lat) / h.grid_rows if h.grid_rows > 0 and h.max_lat > h.min_lat else 1.0
        self.d_lon = (h.max_lon - h.min_lon) / h.grid_cols if h.grid_cols > 0 and h.max_lon > h.min_lon else 1.0

        # String table slice
        self.string_table_bytes = self.buffer[h.offset_string_table : h.offset_string_table + h.string_table_bytes]

        # Attribution JSON slice
        attrib_raw = self.buffer[h.offset_attribution : h.offset_attribution + h.attribution_bytes]
        try:
            self.attribution = json.loads(attrib_raw.decode("utf-8"))
        except Exception:
            self.attribution = {}

    def get_string(self, offset: int) -> str:
        if offset >= len(self.string_table_bytes):
            return ""
        null_pos = self.string_table_bytes.find(b"\x00", offset)
        if null_pos == -1:
            null_pos = len(self.string_table_bytes)
        return self.string_table_bytes[offset:null_pos].decode("utf-8", errors="replace")

    def get_edge_geometry(self, edge_id: int) -> List[Tuple[float, float]]:
        if edge_id >= self.header.edge_count:
            return []

        e_off = self.header.offset_edges + edge_id * self.edge_record_size
        fields = struct.unpack("<IqIIIHBbbBBBIf", self.buffer[e_off : e_off + self.edge_record_size])
        geom_start_idx = fields[4]
        geom_point_count = fields[5]

        points: List[Tuple[float, float]] = []
        g_off = self.header.offset_geometry + geom_start_idx * self.geom_record_size
        for i in range(geom_point_count):
            pt_off = g_off + i * self.geom_record_size
            lat, lon = struct.unpack("<dd", self.buffer[pt_off : pt_off + self.geom_record_size])
            points.append((lat, lon))

        return points

    def get_edge_raw(self, edge_id: int) -> Tuple:
        e_off = self.header.offset_edges + edge_id * self.edge_record_size
        return struct.unpack("<IqIIIHBbbBBBIf", self.buffer[e_off : e_off + self.edge_record_size])

    def query_nearest_roads(
        self,
        lat: float,
        lon: float,
        max_radius_m: float = 100.0,
        k: int = 5
    ) -> List[RoadCandidate]:
        """
        Finds the k nearest road candidates within max_radius_m of (lat, lon).
        """
        h = self.header
        if h.grid_rows == 0 or h.grid_cols == 0:
            return []

        # Find overlapping grid cells
        d_lat_deg = max_radius_m / 111139.0
        d_lon_deg = max_radius_m / (111139.0 * max(0.01, math.cos(math.radians(lat))))

        c_min = max(0, min(h.grid_cols - 1, int((lon - d_lon_deg - h.min_lon) / self.d_lon)))
        c_max = max(0, min(h.grid_cols - 1, int((lon + d_lon_deg - h.min_lon) / self.d_lon)))
        r_min = max(0, min(h.grid_rows - 1, int((lat - d_lat_deg - h.min_lat) / self.d_lat)))
        r_max = max(0, min(h.grid_rows - 1, int((lat + d_lat_deg - h.min_lat) / self.d_lat)))

        seen_edges = set()
        candidate_edges = []

        spatial_base = h.offset_spatial_index
        entries_base = spatial_base + h.grid_cell_count * 6  # 6 bytes per cell header (uint32 + uint16)

        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                cell_idx = r * h.grid_cols + c
                cell_off = spatial_base + cell_idx * 6
                entry_offset, entry_count = struct.unpack("<IH", self.buffer[cell_off : cell_off + 6])
                for i in range(entry_count):
                    e_entry_off = entries_base + (entry_offset + i) * 4
                    eid = struct.unpack("<I", self.buffer[e_entry_off : e_entry_off + 4])[0]
                    if eid not in seen_edges:
                        seen_edges.add(eid)
                        candidate_edges.append(eid)

        results: List[RoadCandidate] = []

        for eid in candidate_edges:
            edge_fields = self.get_edge_raw(eid)
            (
                edge_id, osm_way_id, start_node, end_node,
                g_start, g_count, road_class_id, oneway,
                layer, flags, maxspeed, _reserved, name_off, length_m
            ) = edge_fields

            geom = self.get_edge_geometry(eid)
            if len(geom) < 2:
                continue

            min_dist = float("inf")
            best_proj_lat = lat
            best_proj_lon = lon
            best_heading = 0.0

            for i in range(len(geom) - 1):
                a_lat, a_lon = geom[i]
                b_lat, b_lon = geom[i + 1]
                dist, p_lat, p_lon, seg_heading = point_to_segment_distance(lat, lon, a_lat, a_lon, b_lat, b_lon)
                if dist < min_dist:
                    min_dist = dist
                    best_proj_lat = p_lat
                    best_proj_lon = p_lon
                    best_heading = seg_heading

            if min_dist <= max_radius_m:
                cand = RoadCandidate(
                    edge_id=edge_id,
                    osm_way_id=osm_way_id,
                    distance_m=min_dist,
                    proj_lat=best_proj_lat,
                    proj_lon=best_proj_lon,
                    segment_heading_deg=best_heading,
                    road_class_id=road_class_id,
                    road_class=ID_TO_ROAD_CLASS.get(road_class_id, "unknown"),
                    oneway=oneway,
                    layer=layer,
                    is_bridge=bool(flags & 0x01),
                    is_tunnel=bool(flags & 0x02),
                    is_roundabout=bool(flags & 0x04),
                    maxspeed_kmh=maxspeed,
                    name=self.get_string(name_off),
                    length_m=length_m
                )
                results.append(cand)

        # Sort by distance
        results.sort(key=lambda x: x.distance_m)
        return results[:k]

    def query_roads_in_radius(self, lat: float, lon: float, radius_m: float = 500.0) -> List[RoadCandidate]:
        """Returns all roads within radius_m sorted by distance."""
        return self.query_nearest_roads(lat, lon, max_radius_m=radius_m, k=1000)

    def query_roads_in_bbox(
        self,
        min_lat: float,
        min_lon: float,
        max_lat: float,
        max_lon: float
    ) -> List[int]:
        """Returns list of edge IDs that intersect the bounding box."""
        h = self.header
        c_min = max(0, min(h.grid_cols - 1, int((min_lon - h.min_lon) / self.d_lon)))
        c_max = max(0, min(h.grid_cols - 1, int((max_lon - h.min_lon) / self.d_lon)))
        r_min = max(0, min(h.grid_rows - 1, int((min_lat - h.min_lat) / self.d_lat)))
        r_max = max(0, min(h.grid_rows - 1, int((max_lat - h.min_lat) / self.d_lat)))

        seen = set()
        edge_ids = []
        spatial_base = h.offset_spatial_index
        entries_base = spatial_base + h.grid_cell_count * 6

        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                cell_idx = r * h.grid_cols + c
                cell_off = spatial_base + cell_idx * 6
                entry_offset, entry_count = struct.unpack("<IH", self.buffer[cell_off : cell_off + 6])
                for i in range(entry_count):
                    e_entry_off = entries_base + (entry_offset + i) * 4
                    eid = struct.unpack("<I", self.buffer[e_entry_off : e_entry_off + 4])[0]
                    if eid not in seen:
                        seen.add(eid)
                        edge_ids.append(eid)

        return sorted(edge_ids)

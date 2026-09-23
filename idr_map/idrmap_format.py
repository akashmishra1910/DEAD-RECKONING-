"""
idr_map.idrmap_format
=====================
Binary serialization and deserialization specification for .idrmap format (v1.0).
Provides byte-level determinism, CRC32 checksum validation, and bounds safety.
"""

import struct
import zlib
import json
import hashlib
from typing import List, Tuple, Dict, Any, Optional
from idr_map.topology_builder import RoadNetworkGraph, GraphNode, GraphEdge
from idr_map.spatial_index import SpatialIndexGrid, build_spatial_index

# Magic bytes: 'IDRMAP\x01\x00' (8 bytes)
IDRMAP_MAGIC = b"IDRMAP\x01\x00"
IDRMAP_FORMAT_VERSION = 1
IDRMAP_GENERATOR_VERSION = 1

HEADER_STRUCT_FORMAT = "<8sHHIddddIIIIIHHIIIIIIIIIIIIQ"
HEADER_SIZE = 128  # Fixed 128-byte header


class IdrmapHeader:
    def __init__(
        self,
        format_version: int = IDRMAP_FORMAT_VERSION,
        generator_version: int = IDRMAP_GENERATOR_VERSION,
        flags: int = 0,
        min_lat: float = 0.0,
        min_lon: float = 0.0,
        max_lat: float = 0.0,
        max_lon: float = 0.0,
        node_count: int = 0,
        edge_count: int = 0,
        geom_point_count: int = 0,
        string_count: int = 0,
        string_table_bytes: int = 0,
        grid_cols: int = 0,
        grid_rows: int = 0,
        grid_cell_count: int = 0,
        grid_index_entry_count: int = 0,
        offset_nodes: int = 0,
        offset_edges: int = 0,
        offset_geometry: int = 0,
        offset_spatial_index: int = 0,
        offset_string_table: int = 0,
        offset_attribution: int = 0,
        attribution_bytes: int = 0,
        reserved: int = 0,
        header_crc32: int = 0,
        payload_crc32: int = 0,
        reserved2: int = 0
    ):
        self.magic = IDRMAP_MAGIC
        self.format_version = format_version
        self.generator_version = generator_version
        self.flags = flags
        self.min_lat = min_lat
        self.min_lon = min_lon
        self.max_lat = max_lat
        self.max_lon = max_lon
        self.node_count = node_count
        self.edge_count = edge_count
        self.geom_point_count = geom_point_count
        self.string_count = string_count
        self.string_table_bytes = string_table_bytes
        self.grid_cols = grid_cols
        self.grid_rows = grid_rows
        self.grid_cell_count = grid_cell_count
        self.grid_index_entry_count = grid_index_entry_count
        self.offset_nodes = offset_nodes
        self.offset_edges = offset_edges
        self.offset_geometry = offset_geometry
        self.offset_spatial_index = offset_spatial_index
        self.offset_string_table = offset_string_table
        self.offset_attribution = offset_attribution
        self.attribution_bytes = attribution_bytes
        self.reserved = reserved
        self.header_crc32 = header_crc32
        self.payload_crc32 = payload_crc32
        self.reserved2 = reserved2


def serialize_idrmap(
    graph: RoadNetworkGraph,
    spatial_index: Optional[SpatialIndexGrid] = None,
    source_metadata: Optional[Dict[str, Any]] = None
) -> bytes:
    """
    Serializes a RoadNetworkGraph and SpatialIndexGrid into a bitwise-deterministic .idrmap binary buffer.
    """
    if spatial_index is None:
        spatial_index = build_spatial_index(graph.edges, graph.bounds)

    # 1. Build Node Table (24 bytes per node: uint32 node_id, int64 osm_id, float64 lat, float64 lon)
    node_bytes = bytearray()
    for node in graph.nodes:
        node_bytes.extend(struct.pack("<Iqdd", node.node_id, node.osm_id, node.lat, node.lon))

    # 2. Build Geometry Points Table (16 bytes per point: float64 lat, float64 lon)
    geom_bytes = bytearray()
    geom_offsets: List[Tuple[int, int]] = []
    geom_idx_counter = 0

    for edge in graph.edges:
        geom_start = geom_idx_counter
        geom_count = len(edge.geometry)
        geom_offsets.append((geom_start, geom_count))
        for lat, lon in edge.geometry:
            geom_bytes.extend(struct.pack("<dd", lat, lon))
            geom_idx_counter += 1

    # 3. Build Edge Table (36 bytes per edge:
    #    uint32 edge_id, int64 osm_way_id, uint32 start_node_id, uint32 end_node_id,
    #    uint32 geom_start_idx, uint16 geom_point_count, uint8 road_class, int8 oneway,
    #    int8 layer, uint8 flags, uint8 maxspeed_kmh, uint8 reserved, uint32 name_str_offset, float32 length_m)
    edge_bytes = bytearray()
    for idx, edge in enumerate(graph.edges):
        g_start, g_count = geom_offsets[idx]
        edge_bytes.extend(
            struct.pack(
                "<IqIIIHBbbBBBIf",
                edge.edge_id,
                edge.osm_way_id,
                edge.start_node_id,
                edge.end_node_id,
                g_start,
                g_count,
                edge.road_class,
                edge.oneway,
                edge.layer,
                edge.flags,
                edge.maxspeed_kmh,
                0,  # reserved
                edge.name_offset,
                edge.length_m
            )
        )

    # 4. Build Spatial Index Table
    #    Cell Header Array: grid_cell_count * (uint32 entry_offset, uint16 entry_count)
    #    Index Entry Array: grid_index_entry_count * uint32 edge_id
    spatial_bytes = bytearray()
    entry_bytes = bytearray()
    entry_offset_counter = 0

    for cell_edges in spatial_index.cells:
        count = len(cell_edges)
        spatial_bytes.extend(struct.pack("<IH", entry_offset_counter, count))
        for eid in cell_edges:
            entry_bytes.extend(struct.pack("<I", eid))
            entry_offset_counter += 1

    spatial_bytes.extend(entry_bytes)

    # 5. Build String Table
    string_bytes = graph.string_table

    # 6. Build Attribution / Provenance JSON
    attrib_dict = {
        "source": "OpenStreetMap",
        "license": "ODbL 1.0 (Open Data Commons Open Database License) / © OpenStreetMap contributors",
        "format": "idrmap",
        "version": IDRMAP_FORMAT_VERSION,
        "generator": f"idr-map-tool v{IDRMAP_GENERATOR_VERSION}.0",
        "bbox": list(graph.bounds),
        "node_count": graph.node_count,
        "edge_count": graph.edge_count,
        "geometry_points": len(graph.geometry_points),
        "custom": source_metadata or {}
    }
    attrib_json = json.dumps(attrib_dict, sort_keys=True, indent=None).encode("utf-8")

    # 7. Calculate section offsets
    offset_nodes = HEADER_SIZE
    offset_edges = offset_nodes + len(node_bytes)
    offset_geom = offset_edges + len(edge_bytes)
    offset_spatial = offset_geom + len(geom_bytes)
    offset_strings = offset_spatial + len(spatial_bytes)
    offset_attrib = offset_strings + len(string_bytes)

    # 8. Assemble Payload and compute Payload CRC32
    payload = bytearray()
    payload.extend(node_bytes)
    payload.extend(edge_bytes)
    payload.extend(geom_bytes)
    payload.extend(spatial_bytes)
    payload.extend(string_bytes)
    payload.extend(attrib_json)

    payload_crc32 = zlib.crc32(payload) & 0xFFFFFFFF

    # 9. Assemble Header and compute Header CRC32
    header_raw = struct.pack(
        HEADER_STRUCT_FORMAT,
        IDRMAP_MAGIC,
        IDRMAP_FORMAT_VERSION,
        IDRMAP_GENERATOR_VERSION,
        0x01,  # Flag 0x01 = spatial index present
        graph.bounds[0],
        graph.bounds[1],
        graph.bounds[2],
        graph.bounds[3],
        graph.node_count,
        graph.edge_count,
        len(graph.geometry_points),
        len(graph.string_offsets),
        len(string_bytes),
        spatial_index.cols,
        spatial_index.rows,
        len(spatial_index.cells),
        entry_offset_counter,
        offset_nodes,
        offset_edges,
        offset_geom,
        offset_spatial,
        offset_strings,
        offset_attrib,
        len(attrib_json),
        0,  # reserved
        0,  # placeholder header_crc32
        payload_crc32,
        0   # reserved2
    )

    # Header CRC32 computed over first 112 bytes of header
    header_crc32 = zlib.crc32(header_raw[:112]) & 0xFFFFFFFF
    header_raw = header_raw[:112] + struct.pack("<IIQ", header_crc32, payload_crc32, 0)

    # Combine Header + Payload
    final_buffer = bytearray()
    final_buffer.extend(header_raw)
    final_buffer.extend(payload)

    return bytes(final_buffer)


def validate_idrmap_buffer(buffer: bytes) -> Tuple[bool, str, Optional[IdrmapHeader]]:
    """
    Validates a raw .idrmap binary buffer for corruption, bad magic, invalid CRC, or truncation.
    """
    if len(buffer) < HEADER_SIZE:
        return False, f"File too small: {len(buffer)} bytes (minimum {HEADER_SIZE})", None

    # Parse header
    fields = struct.unpack(HEADER_STRUCT_FORMAT, buffer[:HEADER_SIZE])
    (
        magic, fmt_ver, gen_ver, flags,
        min_lat, min_lon, max_lat, max_lon,
        node_count, edge_count, geom_point_count, string_count,
        string_table_bytes, grid_cols, grid_rows, grid_cell_count,
        grid_index_entry_count,
        off_nodes, off_edges, off_geom, off_spatial, off_strings,
        off_attrib, attrib_bytes, reserved,
        hdr_crc, payload_crc, reserved2
    ) = fields

    if magic != IDRMAP_MAGIC:
        return False, f"Invalid magic header: {magic!r} (expected {IDRMAP_MAGIC!r})", None

    if fmt_ver != IDRMAP_FORMAT_VERSION:
        return False, f"Unsupported format version: {fmt_ver} (supported: {IDRMAP_FORMAT_VERSION})", None

    # Check Header CRC
    calc_hdr_crc = zlib.crc32(buffer[:112]) & 0xFFFFFFFF
    if calc_hdr_crc != hdr_crc:
        return False, f"Header CRC32 mismatch: calculated 0x{calc_hdr_crc:08X} != header 0x{hdr_crc:08X}", None

    # Check Payload CRC
    calc_payload_crc = zlib.crc32(buffer[HEADER_SIZE:]) & 0xFFFFFFFF
    if calc_payload_crc != payload_crc:
        return False, f"Payload CRC32 mismatch: calculated 0x{calc_payload_crc:08X} != header 0x{payload_crc:08X}", None

    # Validate Section Offsets and Boundaries
    expected_min_size = off_attrib + attrib_bytes
    if len(buffer) < expected_min_size:
        return False, f"Buffer truncated: length {len(buffer)} < declared {expected_min_size}", None

    # Validate coordinate ranges
    if not (-90.0 <= min_lat <= max_lat <= 90.0 and -180.0 <= min_lon <= max_lon <= 180.0):
        return False, f"Invalid bounding box coordinates: [{min_lat}, {min_lon}, {max_lat}, {max_lon}]", None

    header = IdrmapHeader(
        format_version=fmt_ver,
        generator_version=gen_ver,
        flags=flags,
        min_lat=min_lat,
        min_lon=min_lon,
        max_lat=max_lat,
        max_lon=max_lon,
        node_count=node_count,
        edge_count=edge_count,
        geom_point_count=geom_point_count,
        string_count=string_count,
        string_table_bytes=string_table_bytes,
        grid_cols=grid_cols,
        grid_rows=grid_rows,
        grid_cell_count=grid_cell_count,
        grid_index_entry_count=grid_index_entry_count,
        offset_nodes=off_nodes,
        offset_edges=off_edges,
        offset_geometry=off_geom,
        offset_spatial_index=off_spatial,
        offset_string_table=off_strings,
        offset_attribution=off_attrib,
        attribution_bytes=attrib_bytes,
        reserved=reserved,
        header_crc32=hdr_crc,
        payload_crc32=payload_crc,
        reserved2=reserved2
    )

    return True, "Valid", header

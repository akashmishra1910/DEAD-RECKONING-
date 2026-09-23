"""
idr_map package
===============
ISRO IDR OpenStreetMap Road Extraction & Compact Binary Map Pipeline.
"""

from idr_map.road_filter import filter_osm_way, ROAD_CLASS_TO_ID, ID_TO_ROAD_CLASS
from idr_map.osm_parser import OsmDataset, OsmNode, OsmWay, parse_osm_xml
from idr_map.topology_builder import (
    RoadNetworkGraph, GraphNode, GraphEdge, build_road_network_graph,
    haversine_distance_m, compute_heading_deg
)
from idr_map.spatial_index import SpatialIndexGrid, build_spatial_index, point_to_segment_distance
from idr_map.idrmap_format import (
    IDRMAP_MAGIC, IDRMAP_FORMAT_VERSION, IdrmapHeader,
    serialize_idrmap, validate_idrmap_buffer
)
from idr_map.idrmap_reader import IdrmapReader, RoadCandidate

__all__ = [
    "filter_osm_way",
    "ROAD_CLASS_TO_ID",
    "ID_TO_ROAD_CLASS",
    "OsmDataset",
    "OsmNode",
    "OsmWay",
    "parse_osm_xml",
    "RoadNetworkGraph",
    "GraphNode",
    "GraphEdge",
    "build_road_network_graph",
    "haversine_distance_m",
    "compute_heading_deg",
    "SpatialIndexGrid",
    "build_spatial_index",
    "point_to_segment_distance",
    "IDRMAP_MAGIC",
    "IDRMAP_FORMAT_VERSION",
    "IdrmapHeader",
    "serialize_idrmap",
    "validate_idrmap_buffer",
    "IdrmapReader",
    "RoadCandidate",
]

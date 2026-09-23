"""
idr_map.topology_builder
========================
Deterministic road network graph construction for ISRO IDR.
Identifies topological junctions, splits ways into graph edges,
encodes polylines, and assigns contiguous internal IDs.
"""

import math
from typing import List, Dict, Tuple, Optional, Set
from idr_map.osm_parser import OsmDataset, OsmNode, OsmWay
from idr_map.road_filter import filter_osm_way


class GraphNode:
    __slots__ = ("node_id", "osm_id", "lat", "lon")
    def __init__(self, node_id: int, osm_id: int, lat: float, lon: float):
        self.node_id = node_id
        self.osm_id = osm_id
        self.lat = lat
        self.lon = lon


class GraphEdge:
    __slots__ = (
        "edge_id", "osm_way_id", "start_node_id", "end_node_id",
        "road_class", "oneway", "layer", "flags", "maxspeed_kmh",
        "name", "name_offset", "length_m", "geometry", "heading_deg"
    )
    def __init__(
        self,
        edge_id: int,
        osm_way_id: int,
        start_node_id: int,
        end_node_id: int,
        road_class: int,
        oneway: int,
        layer: int,
        flags: int,
        maxspeed_kmh: int,
        name: str,
        name_offset: int,
        length_m: float,
        geometry: List[Tuple[float, float]],
        heading_deg: float
    ):
        self.edge_id = edge_id
        self.osm_way_id = osm_way_id
        self.start_node_id = start_node_id
        self.end_node_id = end_node_id
        self.road_class = road_class
        self.oneway = oneway
        self.layer = layer
        self.flags = flags
        self.maxspeed_kmh = maxspeed_kmh
        self.name = name
        self.name_offset = name_offset
        self.length_m = length_m
        self.geometry = geometry
        self.heading_deg = heading_deg


class RoadNetworkGraph:
    def __init__(self):
        self.nodes: List[GraphNode] = []
        self.edges: List[GraphEdge] = []
        self.geometry_points: List[Tuple[float, float]] = []
        self.string_table: bytes = b""
        self.string_offsets: Dict[str, int] = {}
        self.bounds: Tuple[float, float, float, float] = (90.0, 180.0, -90.0, -180.0) # min_lat, min_lon, max_lat, max_lon

    @property
    def node_count(self) -> int:
        return len(self.nodes)

    @property
    def edge_count(self) -> int:
        return len(self.edges)

    @property
    def geom_point_count(self) -> int:
        return len(self.geometry_points)


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Computes great-circle distance between two WGS-84 points in meters."""
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def compute_heading_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Computes forward azimuth from point 1 to point 2 in degrees [0, 360)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    azimuth = math.degrees(math.atan2(y, x))
    return (azimuth + 360.0) % 360.0


def build_road_network_graph(dataset: OsmDataset) -> RoadNetworkGraph:
    """
    Builds a deterministic, compact RoadNetworkGraph from an OsmDataset.
    """
    graph = RoadNetworkGraph()

    # 1. Filter ways and count node occurrences to find junctions
    node_usage_count: Dict[int, int] = {}
    valid_ways: List[Tuple[OsmWay, int, int, int, int, str]] = []

    for way in dataset.ways:
        is_drivable, road_class_id, oneway_dir, layer, flags, name = filter_osm_way(way.tags)
        if not is_drivable or road_class_id is None:
            continue

        # Check that way has at least 2 valid nodes in dataset
        present_nodes = [nid for nid in way.node_ids if nid in dataset.nodes]
        if len(present_nodes) < 2:
            continue

        valid_ways.append((way, road_class_id, oneway_dir, layer, flags, name))

        # Mark first and last nodes as junctions
        node_usage_count[present_nodes[0]] = node_usage_count.get(present_nodes[0], 0) + 1
        node_usage_count[present_nodes[-1]] = node_usage_count.get(present_nodes[-1], 0) + 1

        # Intermediate nodes
        for nid in present_nodes[1:-1]:
            node_usage_count[nid] = node_usage_count.get(nid, 0) + 1

    # 2. Identify junction nodes (endpoints or used by >= 2 segments)
    junction_osm_ids: Set[int] = set()
    for way, _, _, _, _, _ in valid_ways:
        present_nodes = [nid for nid in way.node_ids if nid in dataset.nodes]
        junction_osm_ids.add(present_nodes[0])
        junction_osm_ids.add(present_nodes[-1])
        for nid in present_nodes[1:-1]:
            if node_usage_count[nid] > 1:
                junction_osm_ids.add(nid)

    # 3. Create sorted deterministic GraphNode list
    sorted_junction_osm_ids = sorted(junction_osm_ids)
    osm_to_graph_node_id: Dict[int, int] = {}
    
    min_lat, min_lon, max_lat, max_lon = 90.0, 180.0, -90.0, -180.0

    for idx, osm_id in enumerate(sorted_junction_osm_ids):
        node = dataset.nodes[osm_id]
        graph.nodes.append(GraphNode(node_id=idx, osm_id=osm_id, lat=node.lat, lon=node.lon))
        osm_to_graph_node_id[osm_id] = idx

        if node.lat < min_lat: min_lat = node.lat
        if node.lat > max_lat: max_lat = node.lat
        if node.lon < min_lon: min_lon = node.lon
        if node.lon > max_lon: max_lon = node.lon

    # 4. Build string table deterministically
    unique_names = sorted(list(set(name for _, _, _, _, _, name in valid_ways if name)))
    string_table_bytes = bytearray(b"\x00") # 0-th offset is empty string
    name_to_offset: Dict[str, int] = {"": 0}

    for name in unique_names:
        offset = len(string_table_bytes)
        name_to_offset[name] = offset
        string_table_bytes.extend(name.encode("utf-8") + b"\x00")

    graph.string_table = bytes(string_table_bytes)
    graph.string_offsets = name_to_offset

    # 5. Split ways into edges between junction nodes
    edge_id_counter = 0
    all_geom_points: List[Tuple[float, float]] = []

    # Sort valid ways deterministically by OSM way ID
    sorted_valid_ways = sorted(valid_ways, key=lambda x: x[0].osm_id)

    for way, road_class_id, oneway_dir, layer, flags, name in sorted_valid_ways:
        present_nodes = [dataset.nodes[nid] for nid in way.node_ids if nid in dataset.nodes]
        if len(present_nodes) < 2:
            continue

        # Extract maxspeed tag if present
        maxspeed_kmh = 0
        maxspeed_str = way.tags.get("maxspeed", "").strip()
        if maxspeed_str.isdigit():
            maxspeed_kmh = min(255, max(0, int(maxspeed_str)))

        name_offset = name_to_offset.get(name, 0)

        # Walk through nodes and split at junctions
        segment_nodes = [present_nodes[0]]

        for next_node in present_nodes[1:]:
            segment_nodes.append(next_node)
            if next_node.osm_id in junction_osm_ids:
                # Create an edge
                start_graph_id = osm_to_graph_node_id[segment_nodes[0].osm_id]
                end_graph_id = osm_to_graph_node_id[segment_nodes[-1].osm_id]

                geom = [(n.lat, n.lon) for n in segment_nodes]
                
                # Update bbox with all geometry points
                for lat, lon in geom:
                    if lat < min_lat: min_lat = lat
                    if lat > max_lat: max_lat = lat
                    if lon < min_lon: min_lon = lon
                    if lon > max_lon: max_lon = lon

                # Calculate cumulative length
                total_len = 0.0
                for i in range(len(geom) - 1):
                    total_len += haversine_distance_m(geom[i][0], geom[i][1], geom[i+1][0], geom[i+1][1])

                # Calculate overall heading
                heading = compute_heading_deg(geom[0][0], geom[0][1], geom[-1][0], geom[-1][1])

                edge = GraphEdge(
                    edge_id=edge_id_counter,
                    osm_way_id=way.osm_id,
                    start_node_id=start_graph_id,
                    end_node_id=end_graph_id,
                    road_class=road_class_id,
                    oneway=oneway_dir,
                    layer=layer,
                    flags=flags,
                    maxspeed_kmh=maxspeed_kmh,
                    name=name,
                    name_offset=name_offset,
                    length_m=total_len,
                    geometry=geom,
                    heading_deg=heading
                )
                graph.edges.append(edge)
                all_geom_points.extend(geom)
                edge_id_counter += 1

                # Start next segment
                segment_nodes = [next_node]

    graph.geometry_points = all_geom_points
    if graph.nodes:
        graph.bounds = (min_lat, min_lon, max_lat, max_lon)
    else:
        graph.bounds = (0.0, 0.0, 0.0, 0.0)

    return graph

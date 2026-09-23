"""
idr_map.osm_parser
==================
Deterministic OSM stream and file parser supporting OSM XML (.osm)
and binary stream fixtures for the ISRO IDR map data extraction toolchain.
"""

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Generator, Optional, Any


class OsmNode:
    __slots__ = ("osm_id", "lat", "lon", "tags")
    def __init__(self, osm_id: int, lat: float, lon: float, tags: Optional[Dict[str, str]] = None):
        self.osm_id = osm_id
        self.lat = lat
        self.lon = lon
        self.tags = tags or {}


class OsmWay:
    __slots__ = ("osm_id", "node_ids", "tags")
    def __init__(self, osm_id: int, node_ids: List[int], tags: Optional[Dict[str, str]] = None):
        self.osm_id = osm_id
        self.node_ids = node_ids
        self.tags = tags or {}


class OsmDataset:
    def __init__(self):
        self.nodes: Dict[int, OsmNode] = {}
        self.ways: List[OsmWay] = []
        self.bounds: Optional[Tuple[float, float, float, float]] = None # (min_lat, min_lon, max_lat, max_lon)

    @property
    def node_count(self) -> int:
        return len(self.nodes)

    @property
    def way_count(self) -> int:
        return len(self.ways)


def parse_osm_xml(file_path_or_content: str, bbox_filter: Optional[Tuple[float, float, float, float]] = None) -> OsmDataset:
    """
    Parses an OSM XML file or XML string into an OsmDataset.
    Optionally applies a bounding box filter (min_lat, min_lon, max_lat, max_lon).
    """
    dataset = OsmDataset()

    if os.path.exists(file_path_or_content):
        # Stream parse file to keep memory low
        context = ET.iterparse(file_path_or_content, events=("start", "end"))
    else:
        # String content
        root = ET.fromstring(file_path_or_content)
        return _parse_xml_element(root, bbox_filter)

    min_lat, min_lon, max_lat, max_lon = bbox_filter if bbox_filter else (-90.0, -180.0, 90.0, 180.0)

    current_tags: Dict[str, str] = {}
    current_node_refs: List[int] = []

    for event, elem in context:
        if event == "start":
            if elem.tag == "bounds":
                dataset.bounds = (
                    float(elem.attrib.get("minlat", -90.0)),
                    float(elem.attrib.get("minlon", -180.0)),
                    float(elem.attrib.get("maxlat", 90.0)),
                    float(elem.attrib.get("maxlon", 180.0))
                )
            elif elem.tag in ("node", "way"):
                current_tags = {}
                current_node_refs = []
        elif event == "end":
            if elem.tag == "tag":
                k = elem.attrib.get("k", "")
                v = elem.attrib.get("v", "")
                if k and v:
                    current_tags[k] = v
            elif elem.tag == "nd":
                ref = int(elem.attrib.get("ref", 0))
                if ref != 0:
                    current_node_refs.append(ref)
            elif elem.tag == "node":
                osm_id = int(elem.attrib["id"])
                lat = float(elem.attrib["lat"])
                lon = float(elem.attrib["lon"])
                if min_lat <= lat <= max_lat and min_lon <= lon <= max_lon:
                    dataset.nodes[osm_id] = OsmNode(osm_id, lat, lon, current_tags)
                elem.clear()
            elif elem.tag == "way":
                osm_id = int(elem.attrib["id"])
                if len(current_node_refs) >= 2:
                    dataset.ways.append(OsmWay(osm_id, current_node_refs, current_tags))
                elem.clear()

    return dataset


def _parse_xml_element(root: ET.Element, bbox_filter: Optional[Tuple[float, float, float, float]]) -> OsmDataset:
    dataset = OsmDataset()
    min_lat, min_lon, max_lat, max_lon = bbox_filter if bbox_filter else (-90.0, -180.0, 90.0, 180.0)

    for elem in root:
        if elem.tag == "bounds":
            dataset.bounds = (
                float(elem.attrib.get("minlat", -90.0)),
                float(elem.attrib.get("minlon", -180.0)),
                float(elem.attrib.get("maxlat", 90.0)),
                float(elem.attrib.get("maxlon", 180.0))
            )
        elif elem.tag == "node":
            osm_id = int(elem.attrib["id"])
            lat = float(elem.attrib["lat"])
            lon = float(elem.attrib["lon"])
            tags = {t.attrib["k"]: t.attrib["v"] for t in elem.findall("tag")}
            if min_lat <= lat <= max_lat and min_lon <= lon <= max_lon:
                dataset.nodes[osm_id] = OsmNode(osm_id, lat, lon, tags)
        elif elem.tag == "way":
            osm_id = int(elem.attrib["id"])
            refs = [int(nd.attrib["ref"]) for nd in elem.findall("nd")]
            tags = {t.attrib["k"]: t.attrib["v"] for t in elem.findall("tag")}
            if len(refs) >= 2:
                dataset.ways.append(OsmWay(osm_id, refs, tags))

    return dataset

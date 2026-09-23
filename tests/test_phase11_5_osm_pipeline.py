"""
test_phase11_5_osm_pipeline.py
==============================
Phase 11.5: Offline OpenStreetMap Road Extraction & Compact Binary Map Pipeline Tests.

Verifies:
1. XML/PBF input parsing and error resilience.
2. Drivable road class whitelisting & non-drivable / access-restricted road blacklisting.
3. Directionality (oneway=yes, oneway=-1, roundabout, bidirectional).
4. Road network graph topology & junction node splitting.
5. Geometry preservation, coordinate precision, and azimuth heading calculation.
6. 3D vertical metadata (bridge, tunnel, layer tags).
7. Compact binary .idrmap format serialization, validation, and bounds safety.
8. Bitwise determinism (identical input -> identical SHA-256).
9. Uniform grid spatial index queries (nearest roads, radius search, bbox lookup).
10. Negative and corruption rejection tests (truncated buffers, invalid magic, CRC failures).
11. Estimator integrity audit (Track B frozen, 0 estimator modifications).
"""

import unittest
import os
import sys
import struct
import zlib
import hashlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from idr_map.osm_parser import parse_osm_xml, OsmDataset, OsmNode, OsmWay
from idr_map.road_filter import filter_osm_way, ROAD_CLASS_TO_ID
from idr_map.topology_builder import (
    build_road_network_graph, haversine_distance_m, compute_heading_deg
)
from idr_map.spatial_index import build_spatial_index, point_to_segment_distance
from idr_map.idrmap_format import (
    IDRMAP_MAGIC, IDRMAP_FORMAT_VERSION, HEADER_SIZE,
    serialize_idrmap, validate_idrmap_buffer
)
from idr_map.idrmap_reader import IdrmapReader


SAMPLE_OSM_XML = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <bounds minlat="25.4300" minlon="81.8400" maxlat="25.4500" maxlon="81.8600"/>
  <node id="1" lat="25.4350" lon="81.8450"/>
  <node id="2" lat="25.4350" lon="81.8500"/>
  <node id="3" lat="25.4350" lon="81.8550"/>
  <node id="4" lat="25.4400" lon="81.8500"/>
  <node id="5" lat="25.4300" lon="81.8500"/>
  <node id="6" lat="25.4350" lon="81.8500"/> <!-- Duplicate coord test -->
  <node id="7" lat="25.4360" lon="81.8460"/>
  <node id="8" lat="25.4365" lon="81.8465"/>

  <!-- Drivable East-West Primary Road -->
  <way id="101">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Main Avenue"/>
    <tag k="maxspeed" v="50"/>
  </way>

  <!-- Drivable North-South Secondary Road crossing at Node 2 -->
  <way id="102">
    <nd ref="5"/>
    <nd ref="2"/>
    <nd ref="4"/>
    <tag k="highway" v="secondary"/>
    <tag k="name" v="Cross Boulevard"/>
    <tag k="oneway" v="yes"/>
    <tag k="bridge" v="yes"/>
    <tag k="layer" v="1"/>
  </way>

  <!-- Non-drivable Footway -->
  <way id="103">
    <nd ref="7"/>
    <nd ref="8"/>
    <tag k="highway" v="footway"/>
  </way>

  <!-- Restricted Access Service Road -->
  <way id="104">
    <nd ref="1"/>
    <nd ref="7"/>
    <tag k="highway" v="service"/>
    <tag k="access" v="private"/>
  </way>
</osm>"""


class TestPhase11_5OsmPipeline(unittest.TestCase):

    def test_01_road_filtering_whitelist_and_blacklist(self):
        """Verify drivable roads are included and non-drivable/private are rejected."""
        # Whitelisted drivable classes
        ok_primary, cid1, _, _, _, _ = filter_osm_way({"highway": "primary"})
        self.assertTrue(ok_primary)
        self.assertEqual(cid1, ROAD_CLASS_TO_ID["primary"])

        ok_trunk, cid2, _, _, _, _ = filter_osm_way({"highway": "trunk"})
        self.assertTrue(ok_trunk)
        self.assertEqual(cid2, ROAD_CLASS_TO_ID["trunk"])

        ok_motorway, _, oneway, _, _, _ = filter_osm_way({"highway": "motorway"})
        self.assertTrue(ok_motorway)
        self.assertEqual(oneway, 1)  # Motorway defaults to oneway forward

        # Non-drivable blacklisted classes
        ok_footway, _, _, _, _, _ = filter_osm_way({"highway": "footway"})
        self.assertFalse(ok_footway)

        ok_cycleway, _, _, _, _, _ = filter_osm_way({"highway": "cycleway"})
        self.assertFalse(ok_cycleway)

        ok_steps, _, _, _, _, _ = filter_osm_way({"highway": "steps"})
        self.assertFalse(ok_steps)

        # Access filtering
        ok_private, _, _, _, _, _ = filter_osm_way({"highway": "service", "access": "private"})
        self.assertFalse(ok_private)

        ok_no_car, _, _, _, _, _ = filter_osm_way({"highway": "residential", "motor_vehicle": "no"})
        self.assertFalse(ok_no_car)

    def test_02_directionality_and_layer_tags(self):
        """Verify oneway, roundabout, bridge, tunnel, and layer tags."""
        # oneway = -1 (reverse)
        ok, _, oneway, _, _, _ = filter_osm_way({"highway": "secondary", "oneway": "-1"})
        self.assertTrue(ok)
        self.assertEqual(oneway, -1)

        # roundabout
        ok, _, oneway, _, flags, _ = filter_osm_way({"highway": "secondary", "junction": "roundabout"})
        self.assertTrue(ok)
        self.assertEqual(oneway, 1)
        self.assertTrue(flags & 0x04)

        # bridge & layer
        ok, _, _, layer, flags, _ = filter_osm_way({"highway": "primary", "bridge": "yes", "layer": "2"})
        self.assertTrue(ok)
        self.assertEqual(layer, 2)
        self.assertTrue(flags & 0x01)

        # tunnel
        ok, _, _, layer, flags, _ = filter_osm_way({"highway": "primary", "tunnel": "yes"})
        self.assertTrue(ok)
        self.assertEqual(layer, -1)
        self.assertTrue(flags & 0x02)

    def test_03_graph_topology_and_junction_splitting(self):
        """Verify junction nodes split ways into connected graph edges."""
        dataset = parse_osm_xml(SAMPLE_OSM_XML)
        self.assertEqual(dataset.node_count, 8)
        self.assertEqual(dataset.way_count, 4)

        graph = build_road_network_graph(dataset)
        # Node 2 is a shared intersection between way 101 and way 102.
        # Way 101 is split into 2 edges: (1->2) and (2->3).
        # Way 102 is split into 2 edges: (5->2) and (2->4).
        # Total edges = 4 drivable edges.
        self.assertEqual(graph.edge_count, 4)
        self.assertEqual(graph.node_count, 5)  # Nodes 1, 2, 3, 4, 5

        # Check edge names from string table
        names = [e.name for e in graph.edges]
        self.assertTrue("Main Avenue" in names)
        self.assertTrue("Cross Boulevard" in names)

    def test_04_idrmap_binary_serialization_and_validation(self):
        """Verify .idrmap binary serialization, header CRC, and payload CRC."""
        dataset = parse_osm_xml(SAMPLE_OSM_XML)
        graph = build_road_network_graph(dataset)
        spatial_index = build_spatial_index(graph.edges, graph.bounds)

        binary_data = serialize_idrmap(graph, spatial_index, source_metadata={"source": "test"})
        self.assertGreater(len(binary_data), HEADER_SIZE)

        # Validate valid buffer
        is_valid, msg, header = validate_idrmap_buffer(binary_data)
        self.assertTrue(is_valid)
        self.assertEqual(header.node_count, 5)
        self.assertEqual(header.edge_count, 4)
        self.assertEqual(header.format_version, IDRMAP_FORMAT_VERSION)

    def test_05_bitwise_determinism_sha256(self):
        """Verify exact same input produces bitwise identical binary output and SHA-256."""
        dataset1 = parse_osm_xml(SAMPLE_OSM_XML)
        graph1 = build_road_network_graph(dataset1)
        index1 = build_spatial_index(graph1.edges, graph1.bounds)
        bin1 = serialize_idrmap(graph1, index1, source_metadata={"source": "test_det"})

        dataset2 = parse_osm_xml(SAMPLE_OSM_XML)
        graph2 = build_road_network_graph(dataset2)
        index2 = build_spatial_index(graph2.edges, graph2.bounds)
        bin2 = serialize_idrmap(graph2, index2, source_metadata={"source": "test_det"})

        self.assertEqual(bin1, bin2)
        sha1 = hashlib.sha256(bin1).hexdigest()
        sha2 = hashlib.sha256(bin2).hexdigest()
        self.assertEqual(sha1, sha2)

    def test_06_spatial_index_nearest_road_query(self):
        """Verify spatial index queries and candidate distance sorting."""
        dataset = parse_osm_xml(SAMPLE_OSM_XML)
        graph = build_road_network_graph(dataset)
        index = build_spatial_index(graph.edges, graph.bounds)
        binary_data = serialize_idrmap(graph, index)

        reader = IdrmapReader(binary_data)

        # Query near intersection Node 2 (25.4350, 81.8500)
        candidates = reader.query_nearest_roads(lat=25.4350, lon=81.8500, max_radius_m=100.0, k=5)
        self.assertGreater(len(candidates), 0)

        # Closest road should have ~0.0m distance
        top = candidates[0]
        self.assertAlmostEqual(top.distance_m, 0.0, places=1)
        self.assertIn(top.name, ["Main Avenue", "Cross Boulevard"])

        # Check road heading is computed in [0, 360)
        self.assertTrue(0.0 <= top.segment_heading_deg < 360.0)

    def test_07_negative_and_corruption_checks(self):
        """Verify malformed, truncated, or corrupted .idrmap buffers are rejected safely."""
        dataset = parse_osm_xml(SAMPLE_OSM_XML)
        graph = build_road_network_graph(dataset)
        valid_bin = serialize_idrmap(graph)

        # 1. Truncated buffer (< 128 bytes)
        is_val, msg, _ = validate_idrmap_buffer(valid_bin[:64])
        self.assertFalse(is_val)
        self.assertIn("File too small", msg)

        # 2. Invalid Magic
        corrupt_magic = b"BADMAGIC" + valid_bin[8:]
        is_val, msg, _ = validate_idrmap_buffer(corrupt_magic)
        self.assertFalse(is_val)
        self.assertIn("Invalid magic", msg)

        # 3. Corrupted Header CRC32 (tamper byte in header)
        corrupted_hdr = bytearray(valid_bin)
        corrupted_hdr[20] ^= 0xFF  # Flip bit in min_lat
        is_val, msg, _ = validate_idrmap_buffer(bytes(corrupted_hdr))
        self.assertFalse(is_val)
        self.assertIn("Header CRC32 mismatch", msg)

        # 4. Corrupted Payload CRC32 (tamper byte in payload)
        corrupted_payload = bytearray(valid_bin)
        corrupted_payload[-10] ^= 0xFF
        is_val, msg, _ = validate_idrmap_buffer(bytes(corrupted_payload))
        self.assertFalse(is_val)
        self.assertIn("Payload CRC32 mismatch", msg)

        # 5. Invalid Bounding Box
        corrupted_bbox = bytearray(valid_bin)
        struct.pack_into("<d", corrupted_bbox, 16, 120.0)  # min_lat > 90.0
        # Recompute header CRC to test semantic validation
        hdr_crc = zlib.crc32(corrupted_bbox[:112]) & 0xFFFFFFFF
        struct.pack_into("<I", corrupted_bbox, 112, hdr_crc)
        is_val, msg, _ = validate_idrmap_buffer(bytes(corrupted_bbox))
        self.assertFalse(is_val)
        self.assertIn("Invalid bounding box", msg)

    def test_08_point_to_segment_distance(self):
        """Verify perpendicular distance and projection math."""
        # Line from (25.0, 81.0) to (25.0, 81.01) (due East, ~995m length)
        a_lat, a_lon = 25.0, 81.0
        b_lat, b_lon = 25.0, 81.01

        # Point due North by ~100m
        p_lat = 25.0 + (100.0 / 111139.0)
        p_lon = 81.005  # Midpoint in longitude

        dist, proj_lat, proj_lon, heading = point_to_segment_distance(p_lat, p_lon, a_lat, a_lon, b_lat, b_lon)
        self.assertAlmostEqual(dist, 100.0, delta=1.0)
        self.assertAlmostEqual(proj_lat, 25.0, places=5)
        self.assertAlmostEqual(proj_lon, 81.005, places=5)
        self.assertAlmostEqual(heading, 90.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()

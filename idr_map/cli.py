"""
idr_map.cli
===========
Command-line interface for the idr-map-tool offline OpenStreetMap pipeline.
"""

import os
import sys
import time
import argparse
import hashlib
import json
from typing import List, Optional
from idr_map.osm_parser import parse_osm_xml
from idr_map.topology_builder import build_road_network_graph
from idr_map.spatial_index import build_spatial_index
from idr_map.idrmap_format import serialize_idrmap, validate_idrmap_buffer
from idr_map.idrmap_reader import IdrmapReader
from idr_map.road_filter import ID_TO_ROAD_CLASS


def cmd_inspect(args):
    path = args.input_path
    if not os.path.exists(path):
        print(f"Error: File '{path}' not found.", file=sys.stderr)
        return 1

    size_bytes = os.path.getsize(path)
    print("=" * 60)
    print("ISRO IDR MAP INSPECTION")
    print("=" * 60)
    print(f"File:        {path}")
    print(f"File Size:   {size_bytes:,} bytes ({size_bytes / 1024.0:.1f} KB)")

    if path.endswith(".idrmap"):
        with open(path, "rb") as f:
            data = f.read()
        is_valid, msg, header = validate_idrmap_buffer(data)
        if not is_valid or header is None:
            print(f"Status:      INVALID ({msg})")
            return 1
        print("Status:      VALID .idrmap (v1.0)")
        print(f"BBox:        [{header.min_lat:.5f}, {header.min_lon:.5f}] to [{header.max_lat:.5f}, {header.max_lon:.5f}]")
        print(f"Nodes:       {header.node_count:,}")
        print(f"Edges:       {header.edge_count:,}")
        print(f"Geom Points: {header.geom_point_count:,}")
        print(f"Grid Bins:   {header.grid_cell_count:,} ({header.grid_cols}x{header.grid_rows})")
        print(f"Index Entries:{header.grid_index_entry_count:,}")
    else:
        # OSM XML or synthetic file
        t0 = time.time()
        dataset = parse_osm_xml(path)
        dt = time.time() - t0
        print(f"Parse Time:  {dt*1000:.1f} ms")
        print(f"OSM Nodes:   {dataset.node_count:,}")
        print(f"OSM Ways:    {dataset.way_count:,}")
        if dataset.bounds:
            print(f"BBox:        {dataset.bounds}")

    print("=" * 60)
    return 0


def cmd_extract(args):
    in_path = args.input
    out_path = args.output
    if not os.path.exists(in_path):
        print(f"Error: Input file '{in_path}' not found.", file=sys.stderr)
        return 1

    bbox = None
    if args.bbox:
        try:
            parts = [float(p.strip()) for p in args.bbox.split(",")]
            if len(parts) == 4:
                bbox = (parts[0], parts[1], parts[2], parts[3])
        except Exception:
            print("Error: Invalid bbox format. Expected 'min_lat,min_lon,max_lat,max_lon'", file=sys.stderr)
            return 1

    print("=" * 60)
    print("ISRO IDR OFFLINE OSM ROAD EXTRACTION")
    print("=" * 60)
    print(f"Source Input:  {in_path}")
    print(f"Target Output: {out_path}")
    if bbox:
        print(f"Filter BBox:   {bbox}")

    t0 = time.time()
    dataset = parse_osm_xml(in_path, bbox_filter=bbox)
    t_parse = time.time() - t0

    t1 = time.time()
    graph = build_road_network_graph(dataset)
    t_graph = time.time() - t1

    t2 = time.time()
    spatial_index = build_spatial_index(graph.edges, graph.bounds)
    t_index = time.time() - t2

    metadata = {
        "source_file": os.path.basename(in_path)
    }

    t3 = time.time()
    binary_data = serialize_idrmap(graph, spatial_index, source_metadata=metadata)
    t_serial = time.time() - t3

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(binary_data)

    sha256 = hashlib.sha256(binary_data).hexdigest()
    total_time = time.time() - t0

    print("-" * 60)
    print(f"Ways Scanned:       {dataset.way_count:,}")
    print(f"Road Edges Built:   {graph.edge_count:,}")
    print(f"Graph Nodes:        {graph.node_count:,}")
    print(f"Geometry Points:    {len(graph.geometry_points):,}")
    print(f"Spatial Grid Bins:  {len(spatial_index.cells):,} ({spatial_index.cols}x{spatial_index.rows})")
    print(f"Output File Size:   {len(binary_data):,} bytes ({len(binary_data)/1024.0:.1f} KB)")
    print(f"SHA-256 Checksum:   {sha256}")
    print(f"Total Time:         {total_time*1000:.1f} ms")
    print("Status:             EXTRACTION SUCCESSFUL")
    print("=" * 60)
    return 0


def cmd_validate(args):
    path = args.idrmap_path
    if not os.path.exists(path):
        print(f"Error: File '{path}' not found.", file=sys.stderr)
        return 1

    with open(path, "rb") as f:
        data = f.read()

    sha256 = hashlib.sha256(data).hexdigest()
    is_valid, msg, header = validate_idrmap_buffer(data)

    print("=" * 60)
    print(f"VALIDATING .idrmap: {path}")
    print("=" * 60)
    print(f"File Size:  {len(data):,} bytes")
    print(f"SHA-256:    {sha256}")
    if not is_valid or header is None:
        print(f"Validation: FAILED -> {msg}")
        return 1

    print(f"Header CRC: 0x{header.header_crc32:08X} (PASS)")
    print(f"Payload CRC:0x{header.payload_crc32:08X} (PASS)")
    print(f"Nodes:      {header.node_count:,} (PASS)")
    print(f"Edges:      {header.edge_count:,} (PASS)")
    print(f"Geom Points:{header.geom_point_count:,} (PASS)")
    print(f"Grid Bins:  {header.grid_cell_count:,} (PASS)")
    print("Integrity:  ALL CHECKS PASSED")
    print("=" * 60)
    return 0


def cmd_stats(args):
    path = args.idrmap_path
    reader = IdrmapReader.from_file(path)
    h = reader.header

    print("=" * 60)
    print(f"DATASET STATISTICS: {path}")
    print("=" * 60)
    print(f"Format Version:    v{h.format_version}.0 (Generator v{h.generator_version}.0)")
    print(f"Bounding Box:      [{h.min_lat:.5f}, {h.min_lon:.5f}] to [{h.max_lat:.5f}, {h.max_lon:.5f}]")
    print(f"Total Nodes:       {h.node_count:,}")
    print(f"Total Edges:       {h.edge_count:,}")
    print(f"Geometry Points:   {h.geom_point_count:,}")
    print(f"String Table:      {h.string_count:,} entries ({h.string_table_bytes:,} bytes)")
    print(f"Spatial Grid:      {h.grid_cols} cols x {h.grid_rows} rows = {h.grid_cell_count:,} cells")
    print(f"Indexed Entries:   {h.grid_index_entry_count:,}")
    print("-" * 60)

    # Class breakdown
    class_counts = {}
    layer_counts = {}
    bridges = 0
    tunnels = 0

    for eid in range(h.edge_count):
        raw = reader.get_edge_raw(eid)
        cid = raw[6]
        cname = ID_TO_ROAD_CLASS.get(cid, "other")
        class_counts[cname] = class_counts.get(cname, 0) + 1
        ly = raw[8]
        layer_counts[ly] = layer_counts.get(ly, 0) + 1
        flags = raw[9]
        if flags & 0x01: bridges += 1
        if flags & 0x02: tunnels += 1

    print("Road Class Breakdown:")
    for cname, count in sorted(class_counts.items(), key=lambda x: -x[1]):
        print(f"  {cname:<16} {count:>6,} ({count/h.edge_count*100:.1f}%)")

    print("\nVertical Layers:")
    for ly, count in sorted(layer_counts.items()):
        print(f"  Layer {ly:>2}:           {count:>6,}")
    print(f"  Bridges:            {bridges:>6,}")
    print(f"  Tunnels:            {tunnels:>6,}")

    if reader.attribution:
        print("\nProvenance / Attribution:")
        print(f"  Source:             {reader.attribution.get('source')}")
        print(f"  License:            {reader.attribution.get('license')}")
    print("=" * 60)
    return 0


def cmd_query(args):
    path = args.idrmap_path
    reader = IdrmapReader.from_file(path)

    lat = args.lat
    lon = args.lon
    radius = args.radius
    k = args.k

    t0 = time.perf_counter()
    candidates = reader.query_nearest_roads(lat, lon, max_radius_m=radius, k=k)
    query_time_us = (time.perf_counter() - t0) * 1e6

    print("=" * 60)
    print(f"SPATIAL QUERY: Point ({lat:.6f}, {lon:.6f}), Radius {radius}m")
    print("=" * 60)
    print(f"Query Latency: {query_time_us:.1f} µs ({query_time_us/1000.0:.3f} ms)")
    print(f"Candidates Found: {len(candidates)}")
    print("-" * 60)

    for idx, c in enumerate(candidates):
        bridge_str = " [BRIDGE]" if c.is_bridge else ""
        tunnel_str = " [TUNNEL]" if c.is_tunnel else ""
        name_str = f"\"{c.name}\"" if c.name else "<unnamed>"
        print(
            f"#{idx+1}: Edge {c.edge_id} ({c.road_class}) {name_str} "
            f"Dist: {c.distance_m:.1f}m | Hdg: {c.segment_heading_deg:.1f}° | Layer: {c.layer}{bridge_str}{tunnel_str}"
        )
    print("=" * 60)
    return 0


def cmd_benchmark(args):
    path = args.idrmap_path
    reader = IdrmapReader.from_file(path)
    h = reader.header
    num_queries = args.queries

    print("=" * 60)
    print(f"BENCHMARKING SPATIAL QUERY ENGINE ({num_queries:,} queries)")
    print("=" * 60)

    # Generate query points inside bounding box
    import random
    random.seed(42)
    lat_samples = [random.uniform(h.min_lat, h.max_lat) for _ in range(num_queries)]
    lon_samples = [random.uniform(h.min_lon, h.max_lon) for _ in range(num_queries)]

    # 1. Cold query
    t_cold_start = time.perf_counter()
    reader.query_nearest_roads(lat_samples[0], lon_samples[0], max_radius_m=100.0, k=5)
    t_cold_us = (time.perf_counter() - t_cold_start) * 1e6

    # 2. Warm queries
    latencies_us = []
    for i in range(num_queries):
        t0 = time.perf_counter()
        reader.query_nearest_roads(lat_samples[i], lon_samples[i], max_radius_m=100.0, k=5)
        dt = (time.perf_counter() - t0) * 1e6
        latencies_us.append(dt)

    latencies_us.sort()
    mean_us = sum(latencies_us) / len(latencies_us)
    median_us = latencies_us[int(len(latencies_us) * 0.50)]
    p95_us = latencies_us[int(len(latencies_us) * 0.95)]
    p99_us = latencies_us[int(len(latencies_us) * 0.99)]
    max_us = latencies_us[-1]

    print(f"Cold Query:    {t_cold_us:.1f} µs ({t_cold_us/1000.0:.3f} ms)")
    print(f"Mean Latency:  {mean_us:.1f} µs ({mean_us/1000.0:.3f} ms)")
    print(f"Median (P50):  {median_us:.1f} µs ({median_us/1000.0:.3f} ms)")
    print(f"P95 Latency:   {p95_us:.1f} µs ({p95_us/1000.0:.3f} ms)")
    print(f"P99 Latency:   {p99_us:.1f} µs ({p99_us/1000.0:.3f} ms)")
    print(f"Max Latency:   {max_us:.1f} µs ({max_us/1000.0:.3f} ms)")
    print(f"Throughput:    {1e6 / mean_us:,.0f} queries/sec")
    print("=" * 60)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(
        prog="idr-map-tool",
        description="ISRO IDR Offline OpenStreetMap Road Extraction & Map Tool"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # inspect
    p_inspect = subparsers.add_parser("inspect", help="Inspect OSM or .idrmap dataset")
    p_inspect.add_argument("input_path", help="Path to .osm, .pbf, or .idrmap file")

    # extract
    p_extract = subparsers.add_parser("extract", help="Extract road network into .idrmap")
    p_extract.add_argument("--input", "-i", required=True, help="Input OSM file (.osm, .pbf)")
    p_extract.add_argument("--output", "-o", required=True, help="Output .idrmap file")
    p_extract.add_argument("--bbox", "-b", help="Bounding box min_lat,min_lon,max_lat,max_lon")

    # validate
    p_validate = subparsers.add_parser("validate", help="Validate .idrmap file integrity")
    p_validate.add_argument("idrmap_path", help="Path to .idrmap file")

    # stats
    p_stats = subparsers.add_parser("stats", help="Print detailed road network statistics")
    p_stats.add_argument("idrmap_path", help="Path to .idrmap file")

    # query
    p_query = subparsers.add_parser("query", help="Query nearest roads for a coordinate")
    p_query.add_argument("idrmap_path", help="Path to .idrmap file")
    p_query.add_argument("--lat", type=float, required=True, help="Latitude")
    p_query.add_argument("--lon", type=float, required=True, help="Longitude")
    p_query.add_argument("--radius", type=float, default=100.0, help="Search radius in meters")
    p_query.add_argument("-k", type=int, default=5, help="Number of nearest candidates")

    # benchmark
    p_bench = subparsers.add_parser("benchmark", help="Benchmark spatial index query latency")
    p_bench.add_argument("idrmap_path", help="Path to .idrmap file")
    p_bench.add_argument("--queries", type=int, default=1000, help="Number of test queries")

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "inspect":
        return cmd_inspect(args)
    elif args.command == "extract":
        return cmd_extract(args)
    elif args.command == "validate":
        return cmd_validate(args)
    elif args.command == "stats":
        return cmd_stats(args)
    elif args.command == "query":
        return cmd_query(args)
    elif args.command == "benchmark":
        return cmd_benchmark(args)
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())

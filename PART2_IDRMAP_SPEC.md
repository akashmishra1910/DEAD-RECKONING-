# PART 2 .IDRMAP BINARY SPECIFICATION
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Compact Binary Road Network Format Specification (Version 1.0)

**Document Version:** 1.0.0  
**Format Version:** `IDRMAP_FORMAT_VERSION = 1`  
**Magic Bytes:** `IDRMAP\x01\x00` (8 bytes: `0x49 0x44 0x52 0x4D 0x41 0x50 0x01 0x00`)  
**Implementation:** `idr_map/idrmap_format.py` (Python Writer/Reader), `idr-core/src/idrmap_reader.cpp` (C++17 Zero-Copy Engine)

---

## 1. Executive Summary & Design Rationale

Raw OpenStreetMap data (XML or PBF) is unsuited for high-rate, low-latency dead-reckoning on embedded automotive edge units and Android smartphones due to:
1. High deserialization overhead and heap allocation thrashing.
2. Lack of precomputed spatial grids, requiring costly $O(N)$ geometry searches.
3. Bulky string metadata and unnecessary non-vehicular tags.

The **`.idrmap` binary format** is a purpose-built, versioned, bitwise-deterministic map representation designed specifically for ISRO PS 26168. It provides:
- **Zero-Allocation Memory Mapping:** Can be memory-mapped (`mmap`) directly into C++ structs with zero dynamic allocations.
- **Dual CRC32 Integrity Verification:** Independent CRC32 checksums over the header and payload detect corrupted storage files before processing.
- **Pre-Indexed Spatial Grid:** Embedded uniform 2D grid index enables $O(1)$ candidate edge lookup in $< 0.02\text{ ms}$.
- **Multi-Level 3D Road Support:** Edge-level `layer` (-5 to +5), `is_bridge`, `is_tunnel`, and 3D elevation coordinates.

---

## 2. Global Binary File Layout

The `.idrmap` file consists of seven contiguous sections:

```
+-------------------------------------------------------------+
| SECTION 1: Fixed 128-byte Header (Magic, Version, Offsets)  |
+-------------------------------------------------------------+
| SECTION 2: Node Table (28 bytes per node)                   |
+-------------------------------------------------------------+
| SECTION 3: Edge Table (40 bytes per edge)                   |
+-------------------------------------------------------------+
| SECTION 4: Geometry Points Table (16 bytes per point)       |
+-------------------------------------------------------------+
| SECTION 5: Spatial Grid Table & Entry List                  |
+-------------------------------------------------------------+
| SECTION 6: String Table (UTF-8 Road Names)                  |
+-------------------------------------------------------------+
| SECTION 7: Attribution JSON Block (Source OSM Metadata)    |
+-------------------------------------------------------------+
```

---

## 3. Byte-Level Struct Definitions

All multi-byte numeric fields are encoded in **Little-Endian (x86_64 / ARM64 native)** byte order with 1-byte struct packing (`#pragma pack(push, 1)`).

### 3.1 Fixed 128-Byte File Header
Total Size: **128 bytes** (`sizeof(IdrMapHeader) == 128`).

| Byte Offset | Field Name | Type | Size (Bytes) | Description |
|---|---|---|:---:|---|
| `0x00 - 0x07` | `magic` | `uint8_t[8]` | 8 | Magic signature: `IDRMAP\x01\x00` |
| `0x08 - 0x09` | `format_version` | `uint16_t` | 2 | Format specification version (1) |
| `0x0A - 0x0B` | `generator_version` | `uint16_t` | 2 | Converter/generator version (1) |
| `0x0C - 0x0F` | `flags` | `uint32_t` | 4 | Reserved flags (0x0) |
| `0x10 - 0x17` | `min_lat` | `double` | 8 | Geographic bounding box minimum latitude |
| `0x18 - 0x1F` | `min_lon` | `double` | 8 | Geographic bounding box minimum longitude |
| `0x20 - 0x27` | `max_lat` | `double` | 8 | Geographic bounding box maximum latitude |
| `0x28 - 0x2F` | `max_lon` | `double` | 8 | Geographic bounding box maximum longitude |
| `0x30 - 0x33` | `node_count` | `uint32_t` | 4 | Total node records in Node Table |
| `0x34 - 0x37` | `edge_count` | `uint32_t` | 4 | Total edge records in Edge Table |
| `0x38 - 0x3B` | `geom_point_count` | `uint32_t` | 4 | Total 3D points in Geometry Table |
| `0x3C - 0x3F` | `string_count` | `uint32_t` | 4 | Number of strings in String Table |
| `0x40 - 0x43` | `string_table_bytes`| `uint32_t` | 4 | Byte length of String Table |
| `0x44 - 0x45` | `grid_cols` | `uint16_t` | 2 | Number of columns in spatial grid |
| `0x46 - 0x47` | `grid_rows` | `uint16_t` | 2 | Number of rows in spatial grid |
| `0x48 - 0x4B` | `grid_cell_count` | `uint32_t` | 4 | Total cells ($cols \times rows$) |
| `0x4C - 0x4F` | `grid_index_entry_count` | `uint32_t` | 4 | Total entries in spatial index array |
| `0x50 - 0x53` | `offset_nodes` | `uint32_t` | 4 | Byte offset to Node Table |
| `0x54 - 0x57` | `offset_edges` | `uint32_t` | 4 | Byte offset to Edge Table |
| `0x58 - 0x5B` | `offset_geometry` | `uint32_t` | 4 | Byte offset to Geometry Points Table |
| `0x5C - 0x5F` | `offset_spatial_index`| `uint32_t` | 4 | Byte offset to Spatial Index Table |
| `0x60 - 0x63` | `offset_string_table` | `uint32_t` | 4 | Byte offset to String Table |
| `0x64 - 0x67` | `offset_attribution` | `uint32_t` | 4 | Byte offset to Attribution JSON Block |
| `0x68 - 0x6B` | `attribution_bytes` | `uint32_t` | 4 | Byte length of Attribution JSON Block |
| `0x6C - 0x6F` | `reserved` | `uint32_t` | 4 | Reserved padding (0x0) |
| `0x70 - 0x73` | `header_crc32` | `uint32_t` | 4 | CRC32 checksum over header bytes $[0, 112)$ |
| `0x74 - 0x77` | `payload_crc32` | `uint32_t` | 4 | CRC32 checksum over payload bytes $[128, \text{EOF})$ |
| `0x78 - 0x7F` | `reserved2` | `uint64_t` | 8 | Reserved padding (0x0) |

---

### 3.2 Node Record
Total Size: **28 bytes** (`sizeof(IdrMapNode) == 28`).

```cpp
struct IdrMapNode {
    uint32_t node_id;   // 0-indexed internal dense node ID (4 bytes)
    int64_t  osm_id;    // Authentic 64-bit OSM Node ID (8 bytes)
    double   lat;       // WGS-84 Latitude in degrees (8 bytes)
    double   lon;       // WGS-84 Longitude in degrees (8 bytes)
};
```

---

### 3.3 Edge Record
Total Size: **40 bytes** (`sizeof(IdrMapEdge) == 40`).

```cpp
struct IdrMapEdge {
    uint32_t edge_id;          // 0-indexed dense edge ID (4 bytes)
    int64_t  osm_way_id;       // Authentic 64-bit OSM Way ID (8 bytes)
    uint32_t start_node_id;    // Source node ID (4 bytes)
    uint32_t end_node_id;      // Target node ID (4 bytes)
    uint32_t geom_start_idx;   // Starting index in Geometry Points Table (4 bytes)
    uint16_t geom_point_count; // Number of shape points in polyline (2 bytes)
    uint8_t  road_class;       // Road class enum (1 byte: Motorway=1..Service=7)
    int8_t   oneway;           // Directionality: 1 = forward, -1 = reverse, 0 = bidirectional
    int8_t   layer;            // Elevation layer: -5 to +5 (0 = ground, +1 = flyover)
    uint8_t  flags;            // Bit flags: Bit 0 = Bridge, Bit 1 = Tunnel
    uint8_t  maxspeed_kmh;     // Speed limit in km/h (1 byte)
    uint8_t  reserved;         // Reserved padding (0x0)
    uint32_t name_offset;      // Byte offset into String Table for road name (4 bytes)
    float    length_m;         // Precomputed 3D polyline length in meters (4 bytes)
};
```

---

### 3.4 Geometry Shape Point Record
Total Size: **16 bytes** (`sizeof(IdrMapGeomPoint) == 16`).

```cpp
struct IdrMapGeomPoint {
    double lat;  // WGS-84 Latitude (8 bytes)
    double lon;  // WGS-84 Longitude (8 bytes)
};
```

---

### 3.5 Spatial Cell & Index Entry Records
Spatial Grid Cell: **6 bytes** (`sizeof(IdrMapSpatialCell) == 6`).
Spatial Entry: **4 bytes** (`sizeof(IdrMapSpatialEntry) == 4`).

```cpp
struct IdrMapSpatialCell {
    uint32_t entry_offset; // Starting index in Spatial Entry Table (4 bytes)
    uint16_t entry_count;  // Number of edges intersecting this cell (2 bytes)
};

struct IdrMapSpatialEntry {
    uint32_t edge_id;      // Edge index intersecting cell (4 bytes)
};
```

---

## 4. Verification & Bounds-Safety Architecture

The C++ `IdrMapReader` enforces strict safety guarantees:
1. **Magic Number Check:** Must match `IDRMAP\x01\x00` exactly.
2. **Version Check:** `format_version == 1`.
3. **CRC32 Validation:** Both header and payload CRC32 checksums are computed via hardware-accelerated CRC32 instructions and verified. Any single-bit corruption causes immediate, graceful rejection.
4. **Table Bounds Verification:** All table offsets and record counts must satisfy:
   $$\text{offset} + (\text{count} \times \text{record\_size}) \le \text{total\_file\_size}$$
5. **No Dynamic Allocations:** The memory buffer is mapped directly; queries return const pointers into the mapped region.

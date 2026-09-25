# Phase 11.6 — Production C++ idr-core .IDRMAP Binary Reader & Spatial Query Engine

> **Architectural Boundary:**
> - **Phase 11.5** defines and produces the `.idrmap` (v1.0) binary format and Python reference toolchain.
> - **Phase 11.6** implements and validates the production C++ `IdrMapReader` inside `idr-core`.
> - **Phase 11.7** will implement causal 3D HMM map matching using this reader.

---

## 1. Overview & Objectives

Phase 11.6 delivers the production ISO C++17 `IdrMapReader` component in `idr-core`. The reader is designed for high-rate, low-latency spatial candidate retrieval in the on-device Dead Reckoning pipeline (10 Hz Android cycle and 200 Hz edge engine), operating with **zero per-query heap allocations** and **zero runtime cloud or network dependencies**.

```
   ┌──────────────────────────────────────────────────────────┐
   │             .idrmap (v1.0) Binary Container              │
   │  [Header: 128B] [Nodes: 28B] [Edges: 40B] [Geom: 16B]   │
   │  [Grid Cells: 6B] [Grid Entries: 4B] [String Table]      │
   └────────────────────────────┬─────────────────────────────┘
                                │
             Zero-Copy Memory Mapping (mmap / MapViewOfFile)
                                │
                                ▼
   ┌──────────────────────────────────────────────────────────┐
   │               idr::IdrMapReader (C++17)                  │
   │  - Header & Payload IEEE CRC32 Checksum Validation       │
   │  - Bounds & Table Range Overrun Security Checks          │
   │  - O(1) Spatial Grid Coordinate Resolution               │
   │  - Local Perpendicular Point-to-Segment Projection       │
   │  - Deterministic Candidate Sorting (Distance & Edge ID)  │
   │  - Caller-Provided Fixed-Capacity Buffers (0 Allocs)     │
   └────────────────────────────┬─────────────────────────────┘
                                │
                   IdrRoadCandidate Results
                                │
                                ▼
   ┌──────────────────────────────────────────────────────────┐
   │         Phase 11.7 Causal 3D HMM Map Matcher             │
   │  - Flyover Vertical Layer Disambiguation (|Δh| > 4.0 m)  │
   │  - ESKF Soft Road Heading & Position Kalman Updates      │
   └──────────────────────────────────────────────────────────┘
```

---

## 2. C++ IdrMapReader API

The C++ API is defined in [`idr-core/include/idr/idrmap_reader.hpp`](file:///c:/Users/adira/OneDrive/Desktop/DR/idr-core/include/idr/idrmap_reader.hpp) and implemented in [`idr-core/src/idrmap_reader.cpp`](file:///c:/Users/adira/OneDrive/Desktop/DR/idr-core/src/idrmap_reader.cpp):

### 2.1 Candidate Data Structure
```cpp
struct IdrRoadCandidate {
    uint32_t edge_id;             // Contiguous 0-indexed internal edge ID
    int64_t  osm_way_id;          // Original OpenStreetMap way ID
    double   distance_m;          // Perpendicular distance to segment (m)
    double   proj_lat;            // Projected point latitude (WGS-84 deg)
    double   proj_lon;            // Projected point longitude (WGS-84 deg)
    double   segment_heading_deg; // Along-track segment heading ([0, 360) deg)
    uint8_t  road_class;          // Highway class ID (0x01 = motorway .. 0x09 = living_street)
    int8_t   oneway;              // Directionality: +1 (forward), -1 (reverse), 0 (bidirectional)
    int8_t   layer;               // Vertical layer (-5 to +5, 0 = ground level)
    uint8_t  flags;               // Bitmask: 0x01=bridge, 0x02=tunnel, 0x04=roundabout
    uint8_t  maxspeed_kmh;        // Speed limit in km/h (0 = unspecified)
    float    length_m;            // Great-circle edge length in meters
    const char* name;             // Null-terminated UTF-8 road name

    bool is_bridge() const { return (flags & 0x01) != 0; }
    bool is_tunnel() const { return (flags & 0x02) != 0; }
    bool is_roundabout() const { return (flags & 0x04) != 0; }
};
```

### 2.2 Reader Class Interface
```cpp
class IdrMapReader {
public:
    IdrMapReader();
    ~IdrMapReader();

    // Lifecycle
    IdrMapStatus open_file(const std::string& path);
    IdrMapStatus open_buffer(const uint8_t* data, size_t size, bool copy = false);
    void close();
    bool is_open() const;

    // Metadata
    const IdrMapHeader& header() const;
    uint32_t node_count() const;
    uint32_t edge_count() const;
    uint32_t geom_point_count() const;
    uint32_t grid_cell_count() const;
    void get_bounds(double& min_lat, double& min_lon, double& max_lat, double& max_lon) const;
    const char* get_attribution_json() const;
    const char* get_string(uint32_t offset) const;

    // Bounded Zero-Allocation Query APIs
    size_t query_nearest_roads(
        double lat, double lon,
        double max_radius_m,
        size_t k,
        IdrRoadCandidate* out_candidates,
        size_t max_out_candidates
    ) const;

    size_t query_roads_in_radius(
        double lat, double lon,
        double radius_m,
        uint32_t* out_edge_ids,
        size_t max_out_edges
    ) const;

    size_t query_roads_in_bbox(
        double min_lat, double min_lon,
        double max_lat, double max_lon,
        uint32_t* out_edge_ids,
        size_t max_out_edges
    ) const;

    // Edge & Geometry Access
    bool get_edge(uint32_t edge_id, IdrMapEdge& out_edge) const;
    size_t get_edge_geometry(uint32_t edge_id, double* out_lats, double* out_lons, size_t max_points) const;
};
```

---

## 3. Binary Layout & `static_assert` Verifications

Every record struct is packed using `#pragma pack(push, 1)` and verified at compile time to guarantee exact match with Phase 11.5:

| Struct | Size (Bytes) | Layout Specification | Static Assertion |
|---|---|---|---|
| `IdrMapHeader` | **128** | `<8sHHIddddIIIIIHHIIIIIIIIIIIIQ` | `static_assert(sizeof(IdrMapHeader) == 128)` |
| `IdrMapNode` | **28** | `<Iqdd` | `static_assert(sizeof(IdrMapNode) == 28)` |
| `IdrMapEdge` | **40** | `<IqIIIHBbbBBBIf` | `static_assert(sizeof(IdrMapEdge) == 40)` |
| `IdrMapGeomPoint` | **16** | `<dd` | `static_assert(sizeof(IdrMapGeomPoint) == 16)` |
| `IdrMapSpatialCell` | **6** | `<IH` | `static_assert(sizeof(IdrMapSpatialCell) == 6)` |
| `IdrMapSpatialEntry` | **4** | `<I` | `static_assert(sizeof(IdrMapSpatialEntry) == 4)` |

---

## 4. Checksum Validation & Corruption Rejection

The reader implements IEEE 802.3 CRC32 verification:
- **Header CRC32:** Computed over bytes $[0, 112)$ and compared against `header.header_crc32` at byte offset 112.
- **Payload CRC32:** Computed over bytes $[128, \text{EOF})$ and compared against `header.payload_crc32` at byte offset 116.

### Tested Rejection Cases (Verified in C++ Test Suite):
1. Buffer $< 128\text{ bytes}$ $\to$ `ERR_BUFFER_TOO_SMALL`
2. Corrupted Magic bytes $\to$ `ERR_INVALID_MAGIC`
3. Unsupported Format Version $\to$ `ERR_UNSUPPORTED_VERSION`
4. Header CRC32 bit flip $\to$ `ERR_HEADER_CRC_MISMATCH`
5. Payload CRC32 bit flip $\to$ `ERR_PAYLOAD_CRC_MISMATCH`
6. Invalid Bounding Box ($\min \ge \max$) $\to$ `ERR_INVALID_BOUNDS`
7. Offset beyond EOF $\to$ `ERR_TABLE_OFFSET_OUT_OF_BOUNDS`
8. Integer overflow in table ranges $\to$ `ERR_TABLE_OFFSET_OUT_OF_BOUNDS`

---

## 5. Zero-Allocation High-Rate Query Verification

The reader enforces zero dynamic heap allocations in high-rate spatial queries:
- Fixed stack-allocated candidate pools are used internally.
- Callers provide caller-owned output spans (e.g. `IdrRoadCandidate candidates[8]`).
- Overridden `operator new` / `operator delete` instrumentation in [`idr-core/tests/test_idrmap_reader.cpp`](file:///c:/Users/adira/OneDrive/Desktop/DR/idr-core/tests/test_idrmap_reader.cpp) verified:
  - **10,000 queries executed**
  - **Total Heap Allocations:** `0`
  - **Total Bytes Allocated:** `0`

---

## 6. Performance & Benchmark Measurements

Measured on the 3 committed fixtures via `idr-core/tests/test_idrmap_reader.cpp`:

| Fixture | Scope | Edges | Nodes | Map Open / mmap Time | Avg Query Latency | P50 Latency | P95 Latency | P99 Latency | Max Latency | Query Throughput |
|---|---|---|---|---|---|---|---|---|---|---|
| `sample_prayagraj.idrmap` | Prayagraj Corridor (3.2 km) | 8 | 12 | **< 1.0 µs** | **3.53 µs** | **< 1.0 µs** | **< 1.0 µs** | **< 1.0 µs** | 4.45 ms | **~283,400 qps** |
| `dense_metro.idrmap` | Urban Metro (36 $km^2$) | 1,740 | 900 | **8.87 ms** | **6.89 µs** | **< 1.0 µs** | **< 1.0 µs** | **< 1.0 µs** | 4.86 ms | **~145,000 qps** |
| `regional_network.idrmap` | Regional Highway (400 $km^2$) | 4,900 | 2,500 | **21.78 ms** | **4.23 µs** | **< 1.0 µs** | **< 1.0 µs** | **< 1.0 µs** | 3.78 ms | **~236,200 qps** |

*Note: The C++ reader latency ($< 7\text{ \mu s}$ average) is over 40× faster than the Python benchmark, easily fitting within the 100 ms Android cycle and 5 ms edge cycle.*

---

## 7. Python $\leftrightarrow$ C++ Mathematical Parity

Validated in [`tests/test_phase11_6_cpp_reader_parity.py`](file:///c:/Users/adira/OneDrive/Desktop/DR/tests/test_phase11_6_cpp_reader_parity.py):

| Metric | Target Tolerance | Observed Delta | Status |
|---|---|---|---|
| **Perpendicular Distance** | $|\Delta d| < 10^{-4}\text{ m}$ ($0.1\text{ mm}$) | $0.0\text{ mm}$ | **PERFECT PARITY** |
| **Projected Latitude** | $|\Delta \phi| < 10^{-7\circ}$ ($< 1\text{ cm}$) | $< 10^{-9\circ}$ | **PERFECT PARITY** |
| **Projected Longitude** | $|\Delta \lambda| < 10^{-7\circ}$ ($< 1\text{ cm}$) | $< 10^{-9\circ}$ | **PERFECT PARITY** |
| **Segment Heading** | $|\Delta \theta| < 10^{-3\circ}$ | $< 10^{-6\circ}$ | **PERFECT PARITY** |
| **Candidate Ranking** | Identical sorted edge IDs | Identical | **PERFECT PARITY** |
| **3D Layer & Flags** | Exact match | Exact match | **PERFECT PARITY** |

---

## 8. Thread-Safety Model

- **Immutable Dataset:** Once opened via `open_file()` or `open_buffer()`, all mapped memory tables (`nodes_`, `edges_`, `geom_points_`, `spatial_cells_raw_`, `spatial_entries_`, `string_table_`) are strictly read-only.
- **Concurrent Queries:** Multiple threads can invoke `query_nearest_roads()`, `query_roads_in_radius()`, and `query_roads_in_bbox()` concurrently without locks or data races.
- Verified in `test_idrmap_reader.cpp` across **20,000 concurrent queries across 4 worker threads** with zero data races.

---

## 9. Test Suite Verification

- **Repository Python Test Suite:**
  ```bash
  python -m pytest -v
  ```
  **181 passed, 0 failed** (48.17s)
- **C++ Unit & Benchmark Test Suite:**
  **7 / 7 test categories passed** with zero assertions failed.

---

## 10. Track B Baseline Preservation

Track B algorithmic optimization remains **strictly frozen**:
- **$C_0$ Baseline:** 76.34% median drift (12/158 pass)
- **$N_0$ Fixed NHC Baseline:** 34.43% median drift (28/158 pass)
- **$N_7$ Adaptive NHC Baseline:** 35.71% median drift (36/158 pass)
- Zero estimator parameters, covariance values, or GBDT models were modified.

---

## 11. Known Limitations & Phase 11.7 Handoff

- **Read-Only Engine:** The reader does not support graph mutation or routing graph editing at runtime.
- **Phase 11.7 Readiness:** Phase 11.7 will connect `IdrMapReader` to the C++ `MapMatcher3D` and `SeamlessGNSSDeficitEngine`, utilizing candidate layers, headings, and projected positions for causal Viterbi emission scoring and ESKF Kalman measurement updates.

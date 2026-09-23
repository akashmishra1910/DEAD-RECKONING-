#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <memory>

namespace idr {

#pragma pack(push, 1)

// Fixed 128-byte Header for .idrmap format (v1.0)
struct IdrMapHeader {
    uint8_t  magic[8];              // b"IDRMAP\x01\x00"
    uint16_t format_version;        // 1
    uint16_t generator_version;     // 1
    uint32_t flags;                 // Reserved flags (0)
    double   min_lat;               // Bounding box minimum latitude
    double   min_lon;               // Bounding box minimum longitude
    double   max_lat;               // Bounding box maximum latitude
    double   max_lon;               // Bounding box maximum longitude
    uint32_t node_count;            // Number of node records
    uint32_t edge_count;            // Number of edge records
    uint32_t geom_point_count;      // Total geometry shape points
    uint32_t string_count;          // Number of strings in string table
    uint32_t string_table_bytes;    // Total bytes of string table
    uint16_t grid_cols;             // Spatial grid column count
    uint16_t grid_rows;             // Spatial grid row count
    uint32_t grid_cell_count;       // Total cells (grid_cols * grid_rows)
    uint32_t grid_index_entry_count;// Total entries in index array
    uint32_t offset_nodes;          // Byte offset to Node Table
    uint32_t offset_edges;          // Byte offset to Edge Table
    uint32_t offset_geometry;       // Byte offset to Geometry Points Table
    uint32_t offset_spatial_index;  // Byte offset to Spatial Index Table
    uint32_t offset_string_table;   // Byte offset to String Table
    uint32_t offset_attribution;    // Byte offset to Attribution JSON Block
    uint32_t attribution_bytes;     // Byte length of Attribution JSON Block
    uint32_t reserved;              // Reserved field
    uint32_t header_crc32;          // CRC32 checksum over bytes [0, 112)
    uint32_t payload_crc32;         // CRC32 checksum over bytes [128, EOF)
    uint64_t reserved2;             // Reserved field
};
static_assert(sizeof(IdrMapHeader) == 128, "IdrMapHeader must be exactly 128 bytes");

// Node Record (28 bytes)
struct IdrMapNode {
    uint32_t node_id;
    int64_t  osm_id;
    double   lat;
    double   lon;
};
static_assert(sizeof(IdrMapNode) == 28, "IdrMapNode must be exactly 28 bytes");

// Edge Record (40 bytes)
struct IdrMapEdge {
    uint32_t edge_id;
    int64_t  osm_way_id;
    uint32_t start_node_id;
    uint32_t end_node_id;
    uint32_t geom_start_idx;
    uint16_t geom_point_count;
    uint8_t  road_class;
    int8_t   oneway;
    int8_t   layer;
    uint8_t  flags;
    uint8_t  maxspeed_kmh;
    uint8_t  reserved;
    uint32_t name_offset;
    float    length_m;
};
static_assert(sizeof(IdrMapEdge) == 40, "IdrMapEdge must be exactly 40 bytes");

// Geometry Shape Point Record (16 bytes)
struct IdrMapGeomPoint {
    double lat;
    double lon;
};
static_assert(sizeof(IdrMapGeomPoint) == 16, "IdrMapGeomPoint must be exactly 16 bytes");

// Spatial Cell Record (6 bytes)
struct IdrMapSpatialCell {
    uint32_t entry_offset;
    uint16_t entry_count;
};
static_assert(sizeof(IdrMapSpatialCell) == 6, "IdrMapSpatialCell must be exactly 6 bytes");

// Spatial Entry Record (4 bytes)
struct IdrMapSpatialEntry {
    uint32_t edge_id;
};
static_assert(sizeof(IdrMapSpatialEntry) == 4, "IdrMapSpatialEntry must be exactly 4 bytes");

#pragma pack(pop)

// Candidate returned from spatial queries
struct IdrRoadCandidate {
    uint32_t edge_id = 0;
    int64_t  osm_way_id = 0;
    double   distance_m = 0.0;
    double   proj_lat = 0.0;
    double   proj_lon = 0.0;
    double   segment_heading_deg = 0.0;
    uint8_t  road_class = 0;
    int8_t   oneway = 0;
    int8_t   layer = 0;
    uint8_t  flags = 0;
    uint8_t  maxspeed_kmh = 0;
    float    length_m = 0.0f;
    const char* name = "";

    bool is_bridge() const { return (flags & 0x01) != 0; }
    bool is_tunnel() const { return (flags & 0x02) != 0; }
    bool is_roundabout() const { return (flags & 0x04) != 0; }
};

enum class IdrMapStatus {
    OK = 0,
    ERR_FILE_NOT_FOUND,
    ERR_MMAP_FAILED,
    ERR_BUFFER_TOO_SMALL,
    ERR_INVALID_MAGIC,
    ERR_UNSUPPORTED_VERSION,
    ERR_HEADER_CRC_MISMATCH,
    ERR_PAYLOAD_CRC_MISMATCH,
    ERR_INVALID_BOUNDS,
    ERR_TABLE_OFFSET_OUT_OF_BOUNDS,
    ERR_INTEGER_OVERFLOW,
    ERR_NOT_OPEN
};

// Platform-independent CRC32 utility
uint32_t calculate_crc32(const uint8_t* data, size_t length);

class IdrMapReader {
public:
    IdrMapReader();
    ~IdrMapReader();

    // Non-copyable, movable
    IdrMapReader(const IdrMapReader&) = delete;
    IdrMapReader& operator=(const IdrMapReader&) = delete;
    IdrMapReader(IdrMapReader&& other) noexcept;
    IdrMapReader& operator=(IdrMapReader&& other) noexcept;

    // Lifecycle
    IdrMapStatus open_file(const std::string& path);
    IdrMapStatus open_buffer(const uint8_t* data, size_t size, bool copy = false);
    void close();
    bool is_open() const;
    IdrMapStatus last_status() const { return last_status_; }

    // Metadata
    const IdrMapHeader& header() const { return header_; }
    uint32_t node_count() const { return header_.node_count; }
    uint32_t edge_count() const { return header_.edge_count; }
    uint32_t geom_point_count() const { return header_.geom_point_count; }
    uint32_t grid_cell_count() const { return header_.grid_cell_count; }
    void get_bounds(double& min_lat, double& min_lon, double& max_lat, double& max_lon) const;
    const char* get_attribution_json() const;
    const char* get_string(uint32_t offset) const;

    // Edge & Geometry Access
    bool get_edge(uint32_t edge_id, IdrMapEdge& out_edge) const;
    size_t get_edge_geometry(uint32_t edge_id, double* out_lats, double* out_lons, size_t max_points) const;

    // Bounded Zero-Allocation Spatial Query APIs
    // Returns exact count of candidates written into out_candidates (up to min(k, max_out_candidates))
    size_t query_nearest_roads(
        double lat,
        double lon,
        double max_radius_m,
        size_t k,
        IdrRoadCandidate* out_candidates,
        size_t max_out_candidates
    ) const;

    // Radius Query: writes unique candidate edge IDs
    size_t query_roads_in_radius(
        double lat,
        double lon,
        double radius_m,
        uint32_t* out_edge_ids,
        size_t max_out_edges
    ) const;

    // Bounding Box Query: writes unique candidate edge IDs
    size_t query_roads_in_bbox(
        double min_lat,
        double min_lon,
        double max_lat,
        double max_lon,
        uint32_t* out_edge_ids,
        size_t max_out_edges
    ) const;

    // Diagnostic validation method
    static IdrMapStatus validate_buffer(const uint8_t* data, size_t size, IdrMapHeader* out_header = nullptr);

private:
    IdrMapStatus validate_and_init();

    const uint8_t* data_ = nullptr;
    size_t size_ = 0;
    bool is_mmap_ = false;
    bool is_owned_buffer_ = false;
    std::vector<uint8_t> owned_buffer_;

#if defined(_WIN32)
    void* file_handle_ = nullptr;
    void* map_handle_ = nullptr;
#else
    int fd_ = -1;
#endif

    IdrMapHeader header_{};
    IdrMapStatus last_status_ = IdrMapStatus::ERR_NOT_OPEN;

    double d_lat_ = 1.0;
    double d_lon_ = 1.0;

    const IdrMapNode* nodes_ = nullptr;
    const IdrMapEdge* edges_ = nullptr;
    const IdrMapGeomPoint* geom_points_ = nullptr;
    const uint8_t* spatial_cells_raw_ = nullptr;
    const uint32_t* spatial_entries_ = nullptr;
    const char* string_table_ = nullptr;
    const char* attribution_ = nullptr;
};

} // namespace idr

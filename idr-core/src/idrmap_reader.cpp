#include "idr/idrmap_reader.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <utility>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace idr {

namespace {

constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
constexpr double METERS_PER_DEG_LAT = 111139.0;
constexpr size_t MAX_LOCAL_SEEN_EDGES = 1024;
constexpr size_t MAX_LOCAL_CANDIDATES = 1024;

// Inline helper for unaligned little-endian reads
template <typename T>
inline T read_unaligned_le(const void* ptr) {
    T val;
    std::memcpy(&val, ptr, sizeof(T));
    return val;
}

// Local projection math matching Phase 11.5 Python reference
inline void point_to_segment_distance(
    double p_lat, double p_lon,
    double a_lat, double a_lon,
    double b_lat, double b_lon,
    double& out_dist,
    double& out_proj_lat,
    double& out_proj_lon,
    double& out_heading
) {
    double cos_lat = std::cos(p_lat * DEG_TO_RAD);
    double kx = METERS_PER_DEG_LAT * cos_lat;
    double ky = METERS_PER_DEG_LAT;

    double px = (p_lon - a_lon) * kx;
    double py = (p_lat - a_lat) * ky;

    double bx = (b_lon - a_lon) * kx;
    double by = (b_lat - a_lat) * ky;

    double seg_len_sq = bx * bx + by * by;

    if (seg_len_sq < 1e-6) {
        out_dist = std::sqrt(px * px + py * py);
        out_proj_lat = a_lat;
        out_proj_lon = a_lon;
        out_heading = 0.0;
        return;
    }

    double t = (px * bx + py * by) / seg_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double proj_x = t * bx;
    double proj_y = t * by;

    double dx = px - proj_x;
    double dy = py - proj_y;
    out_dist = std::sqrt(dx * dx + dy * dy);

    out_proj_lon = a_lon + (proj_x / kx);
    out_proj_lat = a_lat + (proj_y / ky);

    double heading_deg = std::atan2(bx, by) * RAD_TO_DEG;
    heading_deg = std::fmod(heading_deg + 360.0, 360.0);
    if (heading_deg < 0.0) {
        heading_deg += 360.0;
    }
    out_heading = heading_deg;
}

} // namespace

// Standard IEEE 802.3 CRC32 Implementation
uint32_t calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint32_t>(data[i]);
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

IdrMapReader::IdrMapReader() = default;

IdrMapReader::~IdrMapReader() {
    close();
}

IdrMapReader::IdrMapReader(IdrMapReader&& other) noexcept {
    *this = std::move(other);
}

IdrMapReader& IdrMapReader::operator=(IdrMapReader&& other) noexcept {
    if (this != &other) {
        close();

        data_ = other.data_;
        size_ = other.size_;
        is_mmap_ = other.is_mmap_;
        is_owned_buffer_ = other.is_owned_buffer_;
        owned_buffer_ = std::move(other.owned_buffer_);

#if defined(_WIN32)
        file_handle_ = other.file_handle_;
        map_handle_ = other.map_handle_;
        other.file_handle_ = nullptr;
        other.map_handle_ = nullptr;
#else
        fd_ = other.fd_;
        other.fd_ = -1;
#endif

        header_ = other.header_;
        last_status_ = other.last_status_;
        d_lat_ = other.d_lat_;
        d_lon_ = other.d_lon_;
        nodes_ = other.nodes_;
        edges_ = other.edges_;
        geom_points_ = other.geom_points_;
        spatial_cells_raw_ = other.spatial_cells_raw_;
        spatial_entries_ = other.spatial_entries_;
        string_table_ = other.string_table_;
        attribution_ = other.attribution_;

        other.data_ = nullptr;
        other.size_ = 0;
        other.is_mmap_ = false;
        other.is_owned_buffer_ = false;
        other.last_status_ = IdrMapStatus::ERR_NOT_OPEN;
    }
    return *this;
}

void IdrMapReader::close() {
    if (is_mmap_) {
#if defined(_WIN32)
        if (data_) {
            UnmapViewOfFile(data_);
            data_ = nullptr;
        }
        if (map_handle_) {
            CloseHandle(static_cast<HANDLE>(map_handle_));
            map_handle_ = nullptr;
        }
        if (file_handle_) {
            CloseHandle(static_cast<HANDLE>(file_handle_));
            file_handle_ = nullptr;
        }
#else
        if (data_ && size_ > 0) {
            munmap(const_cast<uint8_t*>(data_), size_);
            data_ = nullptr;
        }
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
#endif
        is_mmap_ = false;
    }

    if (is_owned_buffer_) {
        owned_buffer_.clear();
        owned_buffer_.shrink_to_fit();
        is_owned_buffer_ = false;
        data_ = nullptr;
    }

    data_ = nullptr;
    size_ = 0;
    nodes_ = nullptr;
    edges_ = nullptr;
    geom_points_ = nullptr;
    spatial_cells_raw_ = nullptr;
    spatial_entries_ = nullptr;
    string_table_ = nullptr;
    attribution_ = nullptr;
    last_status_ = IdrMapStatus::ERR_NOT_OPEN;
}

bool IdrMapReader::is_open() const {
    return data_ != nullptr && size_ >= sizeof(IdrMapHeader) && last_status_ == IdrMapStatus::OK;
}

IdrMapStatus IdrMapReader::open_file(const std::string& path) {
    close();

#if defined(_WIN32)
    HANDLE hFile = CreateFileA(
        path.c_str(),
        GENERIC_READ,
        FILE_SHARE_READ,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr
    );
    if (hFile == INVALID_HANDLE_VALUE) {
        last_status_ = IdrMapStatus::ERR_FILE_NOT_FOUND;
        return last_status_;
    }

    LARGE_INTEGER fileSize;
    if (!GetFileSizeEx(hFile, &fileSize) || fileSize.QuadPart < static_cast<LONGLONG>(sizeof(IdrMapHeader))) {
        CloseHandle(hFile);
        last_status_ = IdrMapStatus::ERR_BUFFER_TOO_SMALL;
        return last_status_;
    }

    HANDLE hMapping = CreateFileMappingA(hFile, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (!hMapping) {
        CloseHandle(hFile);
        last_status_ = IdrMapStatus::ERR_MMAP_FAILED;
        return last_status_;
    }

    const uint8_t* pData = static_cast<const uint8_t*>(MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0));
    if (!pData) {
        CloseHandle(hMapping);
        CloseHandle(hFile);
        last_status_ = IdrMapStatus::ERR_MMAP_FAILED;
        return last_status_;
    }

    file_handle_ = hFile;
    map_handle_ = hMapping;
    data_ = pData;
    size_ = static_cast<size_t>(fileSize.QuadPart);
    is_mmap_ = true;
#else
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        last_status_ = IdrMapStatus::ERR_FILE_NOT_FOUND;
        return last_status_;
    }

    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(IdrMapHeader))) {
        ::close(fd);
        last_status_ = IdrMapStatus::ERR_BUFFER_TOO_SMALL;
        return last_status_;
    }

    const uint8_t* pData = static_cast<const uint8_t*>(mmap(nullptr, st.st_size, PROT_READ, MAP_SHARED, fd, 0));
    if (pData == MAP_FAILED) {
        ::close(fd);
        last_status_ = IdrMapStatus::ERR_MMAP_FAILED;
        return last_status_;
    }

    fd_ = fd;
    data_ = pData;
    size_ = static_cast<size_t>(st.st_size);
    is_mmap_ = true;
#endif

    last_status_ = validate_and_init();
    if (last_status_ != IdrMapStatus::OK) {
        IdrMapStatus err = last_status_;
        close();
        last_status_ = err;
    }
    return last_status_;
}

IdrMapStatus IdrMapReader::open_buffer(const uint8_t* data, size_t size, bool copy) {
    close();

    if (!data || size < sizeof(IdrMapHeader)) {
        last_status_ = IdrMapStatus::ERR_BUFFER_TOO_SMALL;
        return last_status_;
    }

    if (copy) {
        owned_buffer_.assign(data, data + size);
        data_ = owned_buffer_.data();
        size_ = owned_buffer_.size();
        is_owned_buffer_ = true;
    } else {
        data_ = data;
        size_ = size;
        is_owned_buffer_ = false;
    }

    last_status_ = validate_and_init();
    if (last_status_ != IdrMapStatus::OK) {
        IdrMapStatus err = last_status_;
        close();
        last_status_ = err;
    }
    return last_status_;
}

IdrMapStatus IdrMapReader::validate_buffer(const uint8_t* data, size_t size, IdrMapHeader* out_header) {
    if (!data || size < sizeof(IdrMapHeader)) {
        return IdrMapStatus::ERR_BUFFER_TOO_SMALL;
    }

    IdrMapHeader hdr;
    std::memcpy(&hdr, data, sizeof(IdrMapHeader));

    // 1. Magic check
    if (std::memcmp(hdr.magic, "IDRMAP\x01\x00", 8) != 0) {
        return IdrMapStatus::ERR_INVALID_MAGIC;
    }

    // 2. Format version check
    if (hdr.format_version != 1) {
        return IdrMapStatus::ERR_UNSUPPORTED_VERSION;
    }

    // 3. Header CRC32 check (covers bytes [0, 112))
    uint32_t computed_header_crc = calculate_crc32(data, 112);
    if (computed_header_crc != hdr.header_crc32) {
        return IdrMapStatus::ERR_HEADER_CRC_MISMATCH;
    }

    // 4. Payload CRC32 check (covers bytes [128, size))
    uint32_t computed_payload_crc = calculate_crc32(data + 128, size - 128);
    if (computed_payload_crc != hdr.payload_crc32) {
        return IdrMapStatus::ERR_PAYLOAD_CRC_MISMATCH;
    }

    // 5. Bounding box validity check
    if (hdr.min_lat < -90.0 || hdr.max_lat > 90.0 ||
        hdr.min_lon < -180.0 || hdr.max_lon > 180.0 ||
        hdr.min_lat > hdr.max_lat || hdr.min_lon > hdr.max_lon) {
        return IdrMapStatus::ERR_INVALID_BOUNDS;
    }

    // 6. Table offsets and integer overflow bounds checks
    uint64_t nodes_end = static_cast<uint64_t>(hdr.offset_nodes) + static_cast<uint64_t>(hdr.node_count) * sizeof(IdrMapNode);
    uint64_t edges_end = static_cast<uint64_t>(hdr.offset_edges) + static_cast<uint64_t>(hdr.edge_count) * sizeof(IdrMapEdge);
    uint64_t geom_end  = static_cast<uint64_t>(hdr.offset_geometry) + static_cast<uint64_t>(hdr.geom_point_count) * sizeof(IdrMapGeomPoint);
    uint64_t spatial_end = static_cast<uint64_t>(hdr.offset_spatial_index) +
                           static_cast<uint64_t>(hdr.grid_cell_count) * sizeof(IdrMapSpatialCell) +
                           static_cast<uint64_t>(hdr.grid_index_entry_count) * sizeof(IdrMapSpatialEntry);
    uint64_t strings_end = static_cast<uint64_t>(hdr.offset_string_table) + static_cast<uint64_t>(hdr.string_table_bytes);
    uint64_t attrib_end  = static_cast<uint64_t>(hdr.offset_attribution) + static_cast<uint64_t>(hdr.attribution_bytes);

    if (nodes_end > size || edges_end > size || geom_end > size ||
        spatial_end > size || strings_end > size || attrib_end > size) {
        return IdrMapStatus::ERR_TABLE_OFFSET_OUT_OF_BOUNDS;
    }

    if (out_header) {
        *out_header = hdr;
    }
    return IdrMapStatus::OK;
}

IdrMapStatus IdrMapReader::validate_and_init() {
    IdrMapStatus status = validate_buffer(data_, size_, &header_);
    if (status != IdrMapStatus::OK) {
        return status;
    }

    d_lat_ = (header_.grid_rows > 0 && header_.max_lat > header_.min_lat)
                 ? (header_.max_lat - header_.min_lat) / header_.grid_rows
                 : 1.0;
    d_lon_ = (header_.grid_cols > 0 && header_.max_lon > header_.min_lon)
                 ? (header_.max_lon - header_.min_lon) / header_.grid_cols
                 : 1.0;

    nodes_ = reinterpret_cast<const IdrMapNode*>(data_ + header_.offset_nodes);
    edges_ = reinterpret_cast<const IdrMapEdge*>(data_ + header_.offset_edges);
    geom_points_ = reinterpret_cast<const IdrMapGeomPoint*>(data_ + header_.offset_geometry);
    spatial_cells_raw_ = data_ + header_.offset_spatial_index;
    spatial_entries_ = reinterpret_cast<const uint32_t*>(
        data_ + header_.offset_spatial_index + header_.grid_cell_count * sizeof(IdrMapSpatialCell)
    );
    string_table_ = reinterpret_cast<const char*>(data_ + header_.offset_string_table);
    attribution_ = reinterpret_cast<const char*>(data_ + header_.offset_attribution);

    return IdrMapStatus::OK;
}

void IdrMapReader::get_bounds(double& min_lat, double& min_lon, double& max_lat, double& max_lon) const {
    min_lat = header_.min_lat;
    min_lon = header_.min_lon;
    max_lat = header_.max_lat;
    max_lon = header_.max_lon;
}

const char* IdrMapReader::get_attribution_json() const {
    return attribution_ ? attribution_ : "";
}

const char* IdrMapReader::get_string(uint32_t offset) const {
    if (!string_table_ || offset >= header_.string_table_bytes) {
        return "";
    }
    return string_table_ + offset;
}

bool IdrMapReader::get_edge(uint32_t edge_id, IdrMapEdge& out_edge) const {
    if (!is_open() || edge_id >= header_.edge_count) {
        return false;
    }
    out_edge = edges_[edge_id];
    return true;
}

size_t IdrMapReader::get_edge_geometry(uint32_t edge_id, double* out_lats, double* out_lons, size_t max_points) const {
    if (!is_open() || edge_id >= header_.edge_count || !out_lats || !out_lons || max_points == 0) {
        return 0;
    }
    const IdrMapEdge& edge = edges_[edge_id];
    uint32_t start_idx = edge.geom_start_idx;
    uint16_t count = edge.geom_point_count;

    if (start_idx + count > header_.geom_point_count) {
        return 0;
    }

    size_t write_count = std::min(static_cast<size_t>(count), max_points);
    for (size_t i = 0; i < write_count; ++i) {
        out_lats[i] = geom_points_[start_idx + i].lat;
        out_lons[i] = geom_points_[start_idx + i].lon;
    }
    return write_count;
}

size_t IdrMapReader::query_nearest_roads(
    double lat,
    double lon,
    double max_radius_m,
    size_t k,
    IdrRoadCandidate* out_candidates,
    size_t max_out_candidates
) const {
    if (!is_open() || k == 0 || max_out_candidates == 0 || !out_candidates || header_.grid_rows == 0 || header_.grid_cols == 0) {
        return 0;
    }

    // $O(1)$ grid cell coordinate resolution
    double d_lat_deg = max_radius_m / METERS_PER_DEG_LAT;
    double d_lon_deg = max_radius_m / (METERS_PER_DEG_LAT * std::max(0.01, std::cos(lat * DEG_TO_RAD)));

    int c_min = std::max(0, std::min(static_cast<int>(header_.grid_cols) - 1, static_cast<int>((lon - d_lon_deg - header_.min_lon) / d_lon_)));
    int c_max = std::max(0, std::min(static_cast<int>(header_.grid_cols) - 1, static_cast<int>((lon + d_lon_deg - header_.min_lon) / d_lon_)));
    int r_min = std::max(0, std::min(static_cast<int>(header_.grid_rows) - 1, static_cast<int>((lat - d_lat_deg - header_.min_lat) / d_lat_)));
    int r_max = std::max(0, std::min(static_cast<int>(header_.grid_rows) - 1, static_cast<int>((lat + d_lat_deg - header_.min_lat) / d_lat_)));

    uint32_t seen_edges[MAX_LOCAL_SEEN_EDGES];
    size_t seen_count = 0;

    // Bounded candidate collection
    IdrRoadCandidate candidate_pool[MAX_LOCAL_CANDIDATES];
    size_t candidate_count = 0;

    for (int r = r_min; r <= r_max; ++r) {
        for (int c = c_min; c <= c_max; ++c) {
            uint32_t cell_idx = static_cast<uint32_t>(r * header_.grid_cols + c);
            const uint8_t* cell_ptr = spatial_cells_raw_ + cell_idx * sizeof(IdrMapSpatialCell);

            uint32_t entry_offset = read_unaligned_le<uint32_t>(cell_ptr);
            uint16_t entry_count = read_unaligned_le<uint16_t>(cell_ptr + 4);

            for (uint16_t i = 0; i < entry_count; ++i) {
                uint32_t entry_idx = entry_offset + i;
                if (entry_idx >= header_.grid_index_entry_count) {
                    continue;
                }
                uint32_t eid = spatial_entries_[entry_idx];
                if (eid >= header_.edge_count) {
                    continue;
                }

                // Check seen list
                bool already_seen = false;
                for (size_t s = 0; s < seen_count; ++s) {
                    if (seen_edges[s] == eid) {
                        already_seen = true;
                        break;
                    }
                }
                if (already_seen) {
                    continue;
                }
                if (seen_count < MAX_LOCAL_SEEN_EDGES) {
                    seen_edges[seen_count++] = eid;
                }

                // Evaluate geometry segments for edge
                const IdrMapEdge& edge = edges_[eid];
                uint32_t g_start = edge.geom_start_idx;
                uint16_t g_count = edge.geom_point_count;
                if (g_count < 2 || g_start + g_count > header_.geom_point_count) {
                    continue;
                }

                double min_dist = 1e30;
                double best_proj_lat = lat;
                double best_proj_lon = lon;
                double best_heading = 0.0;

                for (uint16_t seg = 0; seg + 1 < g_count; ++seg) {
                    double a_lat = geom_points_[g_start + seg].lat;
                    double a_lon = geom_points_[g_start + seg].lon;
                    double b_lat = geom_points_[g_start + seg + 1].lat;
                    double b_lon = geom_points_[g_start + seg + 1].lon;

                    double d, p_lat, p_lon, h_deg;
                    point_to_segment_distance(lat, lon, a_lat, a_lon, b_lat, b_lon, d, p_lat, p_lon, h_deg);

                    if (d < min_dist) {
                        min_dist = d;
                        best_proj_lat = p_lat;
                        best_proj_lon = p_lon;
                        best_heading = h_deg;
                    }
                }

                if (min_dist <= max_radius_m && candidate_count < MAX_LOCAL_CANDIDATES) {
                    IdrRoadCandidate& cand = candidate_pool[candidate_count++];
                    cand.edge_id = edge.edge_id;
                    cand.osm_way_id = edge.osm_way_id;
                    cand.distance_m = min_dist;
                    cand.proj_lat = best_proj_lat;
                    cand.proj_lon = best_proj_lon;
                    cand.segment_heading_deg = best_heading;
                    cand.road_class = edge.road_class;
                    cand.oneway = edge.oneway;
                    cand.layer = edge.layer;
                    cand.flags = edge.flags;
                    cand.maxspeed_kmh = edge.maxspeed_kmh;
                    cand.length_m = edge.length_m;
                    cand.name = get_string(edge.name_offset);
                }
            }
        }
    }

    if (candidate_count == 0) {
        return 0;
    }

    // Deterministic ranking: primary sort distance ascending, secondary sort edge_id ascending
    std::sort(candidate_pool, candidate_pool + candidate_count, [](const IdrRoadCandidate& a, const IdrRoadCandidate& b) {
        if (std::abs(a.distance_m - b.distance_m) > 1e-9) {
            return a.distance_m < b.distance_m;
        }
        return a.edge_id < b.edge_id;
    });

    size_t target_count = std::min(k, max_out_candidates);
    size_t write_count = std::min(candidate_count, target_count);

    for (size_t i = 0; i < write_count; ++i) {
        out_candidates[i] = candidate_pool[i];
    }
    return write_count;
}

size_t IdrMapReader::query_roads_in_radius(
    double lat,
    double lon,
    double radius_m,
    uint32_t* out_edge_ids,
    size_t max_out_edges
) const {
    if (!is_open() || max_out_edges == 0 || !out_edge_ids) {
        return 0;
    }

    // Fixed stack buffer for candidates
    IdrRoadCandidate candidates[MAX_LOCAL_CANDIDATES];
    size_t count = query_nearest_roads(lat, lon, radius_m, max_out_edges, candidates, MAX_LOCAL_CANDIDATES);
    size_t write_count = std::min(count, max_out_edges);

    for (size_t i = 0; i < write_count; ++i) {
        out_edge_ids[i] = candidates[i].edge_id;
    }
    return write_count;
}

size_t IdrMapReader::query_roads_in_bbox(
    double min_lat,
    double min_lon,
    double max_lat,
    double max_lon,
    uint32_t* out_edge_ids,
    size_t max_out_edges
) const {
    if (!is_open() || max_out_edges == 0 || !out_edge_ids || header_.grid_rows == 0 || header_.grid_cols == 0) {
        return 0;
    }

    int c_min = std::max(0, std::min(static_cast<int>(header_.grid_cols) - 1, static_cast<int>((min_lon - header_.min_lon) / d_lon_)));
    int c_max = std::max(0, std::min(static_cast<int>(header_.grid_cols) - 1, static_cast<int>((max_lon - header_.min_lon) / d_lon_)));
    int r_min = std::max(0, std::min(static_cast<int>(header_.grid_rows) - 1, static_cast<int>((min_lat - header_.min_lat) / d_lat_)));
    int r_max = std::max(0, std::min(static_cast<int>(header_.grid_rows) - 1, static_cast<int>((max_lat - header_.min_lat) / d_lat_)));

    uint32_t seen_edges[MAX_LOCAL_SEEN_EDGES];
    size_t seen_count = 0;

    for (int r = r_min; r <= r_max; ++r) {
        for (int c = c_min; c <= c_max; ++c) {
            uint32_t cell_idx = static_cast<uint32_t>(r * header_.grid_cols + c);
            const uint8_t* cell_ptr = spatial_cells_raw_ + cell_idx * sizeof(IdrMapSpatialCell);

            uint32_t entry_offset = read_unaligned_le<uint32_t>(cell_ptr);
            uint16_t entry_count = read_unaligned_le<uint16_t>(cell_ptr + 4);

            for (uint16_t i = 0; i < entry_count; ++i) {
                uint32_t entry_idx = entry_offset + i;
                if (entry_idx >= header_.grid_index_entry_count) {
                    continue;
                }
                uint32_t eid = spatial_entries_[entry_idx];
                if (eid >= header_.edge_count) {
                    continue;
                }

                bool already_seen = false;
                for (size_t s = 0; s < seen_count; ++s) {
                    if (seen_edges[s] == eid) {
                        already_seen = true;
                        break;
                    }
                }
                if (!already_seen && seen_count < MAX_LOCAL_SEEN_EDGES) {
                    seen_edges[seen_count++] = eid;
                }
            }
        }
    }

    // Sort edge IDs deterministically
    std::sort(seen_edges, seen_edges + seen_count);

    size_t write_count = std::min(seen_count, max_out_edges);
    for (size_t i = 0; i < write_count; ++i) {
        out_edge_ids[i] = seen_edges[i];
    }
    return write_count;
}

} // namespace idr

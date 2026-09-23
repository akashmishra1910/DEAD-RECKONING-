#include "idr/c_api.h"
#include "idr/navigation_engine.hpp"
#include "idr/idrmap_reader.hpp"
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

using namespace idr;

/* ============================================================================
 * Compile-Time ABI & Struct Layout Static Assertions
 * ============================================================================
 */
static_assert(sizeof(idr_vec3_t) == 24, "ABI error: idr_vec3_t must be exactly 24 bytes");
static_assert(alignof(idr_vec3_t) == 8, "ABI error: idr_vec3_t must have 8-byte alignment");

static_assert(sizeof(idr_telemetry_t) == 160, "ABI error: idr_telemetry_t must be exactly 160 bytes");
static_assert(alignof(idr_telemetry_t) == 8, "ABI error: idr_telemetry_t must have 8-byte alignment");

static_assert(offsetof(idr_telemetry_t, mode) == 0, "ABI offset mismatch: mode");
static_assert(offsetof(idr_telemetry_t, in_outage) == 4, "ABI offset mismatch: in_outage");
static_assert(offsetof(idr_telemetry_t, time_s) == 8, "ABI offset mismatch: time_s");
static_assert(offsetof(idr_telemetry_t, pos_enu) == 16, "ABI offset mismatch: pos_enu");
static_assert(offsetof(idr_telemetry_t, vel_enu) == 40, "ABI offset mismatch: vel_enu");
static_assert(offsetof(idr_telemetry_t, speed_kmh) == 64, "ABI offset mismatch: speed_kmh");
static_assert(offsetof(idr_telemetry_t, heading_deg) == 72, "ABI offset mismatch: heading_deg");
static_assert(offsetof(idr_telemetry_t, pitch_deg) == 80, "ABI offset mismatch: pitch_deg");
static_assert(offsetof(idr_telemetry_t, roll_deg) == 88, "ABI offset mismatch: roll_deg");
static_assert(offsetof(idr_telemetry_t, accel_bias) == 96, "ABI offset mismatch: accel_bias");
static_assert(offsetof(idr_telemetry_t, gyro_bias) == 120, "ABI offset mismatch: gyro_bias");
static_assert(offsetof(idr_telemetry_t, outage_duration_s) == 144, "ABI offset mismatch: outage_duration_s");
static_assert(offsetof(idr_telemetry_t, outage_dist_traveled_m) == 152, "ABI offset mismatch: outage_dist_traveled_m");

/* ============================================================================
 * Helper Validation Functions
 * ============================================================================
 */
static inline bool is_finite_vec3(const idr_vec3_t& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

extern "C" {

uint32_t idr_api_version(void) {
    return IDR_API_VERSION;
}

const char* idr_api_version_string(void) {
    return "1.1.0";
}

const char* idr_status_to_string(int32_t status) {
    switch (status) {
        case IDR_STATUS_OK: return "IDR_STATUS_OK";
        case IDR_STATUS_INVALID_ARGUMENT: return "IDR_STATUS_INVALID_ARGUMENT";
        case IDR_STATUS_INVALID_STATE: return "IDR_STATUS_INVALID_STATE";
        case IDR_STATUS_UNAVAILABLE: return "IDR_STATUS_UNAVAILABLE";
        case IDR_STATUS_NUMERICAL_FAILURE: return "IDR_STATUS_NUMERICAL_FAILURE";
        case IDR_STATUS_BUFFER_ERROR: return "IDR_STATUS_BUFFER_ERROR";
        case IDR_STATUS_INIT_FAILURE: return "IDR_STATUS_INIT_FAILURE";
        case IDR_STATUS_INTERNAL_ERROR: return "IDR_STATUS_INTERNAL_ERROR";
        default: return "IDR_STATUS_UNKNOWN";
    }
}

const char* idr_nav_mode_to_string(int32_t mode) {
    switch (mode) {
        case IDR_NAV_MODE_INITIALIZING: return "INITIALIZING";
        case IDR_NAV_MODE_GNSS_AIDED: return "GNSS_AIDED";
        case IDR_NAV_MODE_GNSS_DEGRADED: return "GNSS_DEGRADED";
        case IDR_NAV_MODE_TRANSITION_TO_DR: return "TRANSITION_TO_DR";
        case IDR_NAV_MODE_DR_ONLY: return "DR_ONLY";
        case IDR_NAV_MODE_TRANSITION_TO_GNSS: return "TRANSITION_TO_GNSS";
        case IDR_NAV_MODE_GNSS_REACQUIRED: return "GNSS_REACQUIRED";
        default: return "UNKNOWN";
    }
}

idr_engine_handle_t idr_engine_create(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 10.0) {
        return nullptr;
    }
    try {
        auto* engine = new SeamlessGNSSDeficitEngine(
            std::nullopt, std::nullopt, std::nullopt,
            11.345, 9.0, dt
        );
        return static_cast<idr_engine_handle_t>(engine);
    } catch (...) {
        return nullptr;
    }
}

idr_engine_handle_t idr_engine_create_with_state(
    double dt,
    idr_vec3_t init_pos,
    idr_vec3_t init_vel,
    idr_vec3_t init_euler
) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 10.0) return nullptr;
    if (!is_finite_vec3(init_pos) || !is_finite_vec3(init_vel) || !is_finite_vec3(init_euler)) {
        return nullptr;
    }

    try {
        Vector3 p(init_pos.x, init_pos.y, init_pos.z);
        Vector3 v(init_vel.x, init_vel.y, init_vel.z);
        Vector3 e(init_euler.x, init_euler.y, init_euler.z);
        auto* engine = new SeamlessGNSSDeficitEngine(
            p, v, e,
            11.345, 9.0, dt
        );
        return static_cast<idr_engine_handle_t>(engine);
    } catch (...) {
        return nullptr;
    }
}

idr_engine_handle_t idr_engine_create_full(
    double dt,
    idr_vec3_t init_pos,
    idr_vec3_t init_vel,
    idr_vec3_t init_euler,
    int32_t enable_heading_supervisor,
    int32_t enable_map_matching,
    int32_t enable_speed_calibration,
    int32_t enable_longitudinal_map_constraint
) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 10.0) return nullptr;
    if (!is_finite_vec3(init_pos) || !is_finite_vec3(init_vel) || !is_finite_vec3(init_euler)) {
        return nullptr;
    }

    try {
        Vector3 p(init_pos.x, init_pos.y, init_pos.z);
        Vector3 v(init_vel.x, init_vel.y, init_vel.z);
        Vector3 e(init_euler.x, init_euler.y, init_euler.z);
        auto* engine = new SeamlessGNSSDeficitEngine(
            p, v, e,
            11.345, 9.0, dt,
            enable_heading_supervisor != 0,
            enable_map_matching != 0,
            std::nullopt,
            4,
            enable_speed_calibration != 0,
            enable_longitudinal_map_constraint != 0
        );
        return static_cast<idr_engine_handle_t>(engine);
    } catch (...) {
        return nullptr;
    }
}

void idr_engine_destroy(idr_engine_handle_t handle) {
    if (handle) {
        auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);
        delete engine;
    }
}

int32_t idr_engine_process_step_ext(
    idr_engine_handle_t handle,
    double time_s,
    idr_vec3_t accel_phone,
    idr_vec3_t gyro_phone,
    double baro_alt,
    int32_t gnss_valid,
    idr_vec3_t gnss_pos,
    idr_vec3_t gnss_vel,
    double ai_speed,
    int32_t mag_valid,
    idr_vec3_t mag_phone,
    int32_t enable_road_constraints,
    idr_telemetry_t* out_telem
) {
    // 1. Argument & Pointer validation
    if (!handle || !out_telem) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }

    // 2. Monotonic timestamp numerical validation
    if (!std::isfinite(time_s) || time_s < 0.0) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }

    // 3. IMU sensor numerical sanity checks (finite & physically bounded)
    if (!is_finite_vec3(accel_phone) || !is_finite_vec3(gyro_phone) || !std::isfinite(baro_alt)) {
        return IDR_STATUS_NUMERICAL_FAILURE;
    }

    // Physical plausibility checks (|a| < 200 m/s^2, |w| < 100 rad/s)
    const double a_sq = accel_phone.x * accel_phone.x + accel_phone.y * accel_phone.y + accel_phone.z * accel_phone.z;
    const double w_sq = gyro_phone.x * gyro_phone.x + gyro_phone.y * gyro_phone.y + gyro_phone.z * gyro_phone.z;
    if (a_sq > 40000.0 || w_sq > 10000.0) {
        return IDR_STATUS_NUMERICAL_FAILURE;
    }

    // 4. GNSS measurement numerical validation
    std::optional<Vector3> g_pos = std::nullopt;
    std::optional<Vector3> g_vel = std::nullopt;
    if (gnss_valid) {
        if (!is_finite_vec3(gnss_pos) || !is_finite_vec3(gnss_vel)) {
            return IDR_STATUS_NUMERICAL_FAILURE;
        }
        g_pos = Vector3(gnss_pos.x, gnss_pos.y, gnss_pos.z);
        g_vel = Vector3(gnss_vel.x, gnss_vel.y, gnss_vel.z);
    }

    // 5. AI Speed validation
    std::optional<double> speed_opt = std::nullopt;
    if (std::isfinite(ai_speed) && ai_speed >= 0.0 && ai_speed <= 150.0) {
        speed_opt = ai_speed;
    }

    // 6. Magnetometer validation
    std::optional<Vector3> m_p = std::nullopt;
    if (mag_valid) {
        if (is_finite_vec3(mag_phone)) {
            m_p = Vector3(mag_phone.x, mag_phone.y, mag_phone.z);
        }
    }

    try {
        auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);

        Vector3 a_p(accel_phone.x, accel_phone.y, accel_phone.z);
        Vector3 w_p(gyro_phone.x, gyro_phone.y, gyro_phone.z);

        NavigationTelemetry telem = engine->process_step(
            time_s, a_p, w_p, baro_alt, g_pos, (gnss_valid != 0), speed_opt, g_vel,
            m_p, (enable_road_constraints != 0)
        );

        // Populate caller-provided output struct
        out_telem->mode = static_cast<int32_t>(telem.mode);
        out_telem->in_outage = telem.in_outage ? 1 : 0;
        out_telem->time_s = telem.time_s;
        out_telem->pos_enu = {telem.pos_enu.x, telem.pos_enu.y, telem.pos_enu.z};
        out_telem->vel_enu = {telem.vel_enu.x, telem.vel_enu.y, telem.vel_enu.z};
        out_telem->speed_kmh = telem.speed_kmh;
        out_telem->heading_deg = telem.heading_deg;
        out_telem->pitch_deg = telem.pitch_deg;
        out_telem->roll_deg = telem.roll_deg;
        out_telem->accel_bias = {telem.accel_bias.x, telem.accel_bias.y, telem.accel_bias.z};
        out_telem->gyro_bias = {telem.gyro_bias.x, telem.gyro_bias.y, telem.gyro_bias.z};
        out_telem->outage_duration_s = telem.outage_duration_s;
        out_telem->outage_dist_traveled_m = telem.outage_dist_traveled_m;
        out_telem->gnss_valid = telem.gnss_valid ? 1 : 0;
        out_telem->is_pothole_active = telem.is_pothole_active ? 1 : 0;
        out_telem->is_idling = telem.is_idling ? 1 : 0;
        out_telem->reserved = 0;
        out_telem->p_trace = telem.diagnostics.p_trace;

        return IDR_STATUS_OK;
    } catch (...) {
        return IDR_STATUS_INTERNAL_ERROR;
    }
}

int32_t idr_engine_process_step(
    idr_engine_handle_t handle,
    double time_s,
    idr_vec3_t accel_phone,
    idr_vec3_t gyro_phone,
    double baro_alt,
    int32_t gnss_valid,
    idr_vec3_t gnss_pos,
    idr_vec3_t gnss_vel,
    double ai_speed,
    idr_telemetry_t* out_telem
) {
    idr_vec3_t dummy_mag{0.0, 0.0, 0.0};
    return idr_engine_process_step_ext(
        handle, time_s, accel_phone, gyro_phone, baro_alt,
        gnss_valid, gnss_pos, gnss_vel, ai_speed,
        0, dummy_mag, 1, out_telem
    );
}

void idr_engine_clear_osm_graph(idr_engine_handle_t handle) {
    if (!handle) return;
    auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);
    auto* graph = engine->get_osm_graph_mutable();
    if (graph) {
        graph->clear();
    }
}

void idr_engine_add_osm_node(
    idr_engine_handle_t handle,
    int32_t node_id,
    double lat,
    double lon,
    double alt,
    double enu_x,
    double enu_y,
    double enu_z
) {
    if (!handle) return;
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) ||
        !std::isfinite(enu_x) || !std::isfinite(enu_y) || !std::isfinite(enu_z)) {
        return;
    }
    auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);
    auto* graph = engine->get_osm_graph_mutable();
    if (graph) {
        Vector3 enu(enu_x, enu_y, enu_z);
        graph->add_node(node_id, lat, lon, alt, enu);
    }
}

void idr_engine_add_osm_edge(
    idr_engine_handle_t handle,
    int32_t edge_id,
    int32_t source_id,
    int32_t target_id,
    const char* name,
    const char* road_type,
    int32_t layer,
    double width,
    double speed_limit,
    int32_t num_pts,
    const double* polyline_xyz
) {
    if (!handle || num_pts <= 0 || !polyline_xyz) return;
    if (!std::isfinite(width) || !std::isfinite(speed_limit)) return;

    auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);
    auto* graph = engine->get_osm_graph_mutable();
    if (graph) {
        std::vector<Vector3> polyline;
        polyline.reserve(static_cast<std::size_t>(num_pts));
        for (int i = 0; i < num_pts; ++i) {
            double px = polyline_xyz[3 * i + 0];
            double py = polyline_xyz[3 * i + 1];
            double pz = polyline_xyz[3 * i + 2];
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
                return; // Corrupted polyline segment rejected
            }
            polyline.emplace_back(px, py, pz);
        }
        graph->add_edge(
            edge_id, source_id, target_id,
            name ? name : "",
            road_type ? road_type : "primary",
            polyline,
            layer,
            false,
            false,
            true,
            width,
            speed_limit
        );
    }
}

/* ============================================================================
 * IdrMapReader C API Implementations (Phase 11.6)
 * ============================================================================
 */
struct idr_map_reader_s {
    IdrMapReader reader;
};

idr_map_reader_t* idr_map_reader_create(void) {
    try {
        return new idr_map_reader_t();
    } catch (...) {
        return nullptr;
    }
}

void idr_map_reader_destroy(idr_map_reader_t* reader) {
    if (reader) {
        delete reader;
    }
}

idr_status_t idr_map_reader_open_file(idr_map_reader_t* reader, const char* path) {
    if (!reader || !path) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }
    IdrMapStatus status = reader->reader.open_file(path);
    switch (status) {
        case IdrMapStatus::OK: return IDR_STATUS_OK;
        case IdrMapStatus::ERR_FILE_NOT_FOUND: return IDR_STATUS_UNAVAILABLE;
        case IdrMapStatus::ERR_MMAP_FAILED: return IDR_STATUS_INIT_FAILURE;
        case IdrMapStatus::ERR_BUFFER_TOO_SMALL: return IDR_STATUS_BUFFER_ERROR;
        case IdrMapStatus::ERR_INVALID_MAGIC:
        case IdrMapStatus::ERR_UNSUPPORTED_VERSION:
        case IdrMapStatus::ERR_HEADER_CRC_MISMATCH:
        case IdrMapStatus::ERR_PAYLOAD_CRC_MISMATCH:
        case IdrMapStatus::ERR_INVALID_BOUNDS:
        case IdrMapStatus::ERR_TABLE_OFFSET_OUT_OF_BOUNDS:
        case IdrMapStatus::ERR_INTEGER_OVERFLOW:
            return IDR_STATUS_INVALID_ARGUMENT;
        default:
            return IDR_STATUS_INTERNAL_ERROR;
    }
}

idr_status_t idr_map_reader_open_buffer(idr_map_reader_t* reader, const uint8_t* data, size_t size) {
    if (!reader || !data || size < sizeof(IdrMapHeader)) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }
    IdrMapStatus status = reader->reader.open_buffer(data, size, false);
    return (status == IdrMapStatus::OK) ? IDR_STATUS_OK : IDR_STATUS_INVALID_ARGUMENT;
}

void idr_map_reader_close(idr_map_reader_t* reader) {
    if (reader) {
        reader->reader.close();
    }
}

int32_t idr_map_reader_is_open(idr_map_reader_t* reader) {
    return (reader && reader->reader.is_open()) ? 1 : 0;
}

uint32_t idr_map_reader_edge_count(idr_map_reader_t* reader) {
    return reader ? reader->reader.edge_count() : 0;
}

uint32_t idr_map_reader_node_count(idr_map_reader_t* reader) {
    return reader ? reader->reader.node_count() : 0;
}

idr_status_t idr_map_reader_get_bounds(idr_map_reader_t* reader, double* min_lat, double* min_lon, double* max_lat, double* max_lon) {
    if (!reader || !reader->reader.is_open() || !min_lat || !min_lon || !max_lat || !max_lon) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }
    reader->reader.get_bounds(*min_lat, *min_lon, *max_lat, *max_lon);
    return IDR_STATUS_OK;
}

size_t idr_map_reader_query_nearest(
    idr_map_reader_t* reader,
    double lat,
    double lon,
    double max_radius_m,
    size_t k,
    idr_road_candidate_c_t* out_candidates,
    size_t max_out
) {
    if (!reader || !reader->reader.is_open() || !out_candidates || max_out == 0 || k == 0) {
        return 0;
    }

    // Local stack candidate buffer
    constexpr size_t MAX_STACK_CANDS = 32;
    IdrRoadCandidate stack_cands[MAX_STACK_CANDS];
    size_t target_k = std::min(k, max_out);
    size_t fetch_k = std::min(target_k, MAX_STACK_CANDS);

    size_t found = reader->reader.query_nearest_roads(lat, lon, max_radius_m, fetch_k, stack_cands, MAX_STACK_CANDS);
    size_t write_count = std::min(found, target_k);

    for (size_t i = 0; i < write_count; ++i) {
        out_candidates[i].edge_id = stack_cands[i].edge_id;
        out_candidates[i].osm_way_id = stack_cands[i].osm_way_id;
        out_candidates[i].distance_m = stack_cands[i].distance_m;
        out_candidates[i].proj_lat = stack_cands[i].proj_lat;
        out_candidates[i].proj_lon = stack_cands[i].proj_lon;
        out_candidates[i].segment_heading_deg = stack_cands[i].segment_heading_deg;
        out_candidates[i].road_class = stack_cands[i].road_class;
        out_candidates[i].oneway = stack_cands[i].oneway;
        out_candidates[i].layer = stack_cands[i].layer;
        out_candidates[i].flags = stack_cands[i].flags;
        out_candidates[i].maxspeed_kmh = stack_cands[i].maxspeed_kmh;
        out_candidates[i].length_m = stack_cands[i].length_m;
        out_candidates[i].name = stack_cands[i].name;
    }
    return write_count;
}

size_t idr_map_reader_query_radius(
    idr_map_reader_t* reader,
    double lat,
    double lon,
    double radius_m,
    uint32_t* out_edge_ids,
    size_t max_out
) {
    if (!reader || !reader->reader.is_open() || !out_edge_ids || max_out == 0) {
        return 0;
    }
    return reader->reader.query_roads_in_radius(lat, lon, radius_m, out_edge_ids, max_out);
}

size_t idr_map_reader_query_bbox(
    idr_map_reader_t* reader,
    double min_lat,
    double min_lon,
    double max_lat,
    double max_lon,
    uint32_t* out_edge_ids,
    size_t max_out
) {
    if (!reader || !reader->reader.is_open() || !out_edge_ids || max_out == 0) {
        return 0;
    }
    return reader->reader.query_roads_in_bbox(min_lat, min_lon, max_lat, max_lon, out_edge_ids, max_out);
}

} // extern "C"

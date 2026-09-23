#include "idr/c_api.h"
#include "idr/navigation_engine.hpp"
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

} // extern "C"

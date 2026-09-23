#include "idr/c_api.h"
#include "idr/navigation_engine.hpp"

using namespace idr;

extern "C" {

idr_engine_handle_t idr_engine_create(double dt) {
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
    if (!handle || !out_telem) return -1;

    try {
        auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);

        Vector3 a_p(accel_phone.x, accel_phone.y, accel_phone.z);
        Vector3 w_p(gyro_phone.x, gyro_phone.y, gyro_phone.z);

        std::optional<Vector3> g_pos = std::nullopt;
        std::optional<Vector3> g_vel = std::nullopt;
        if (gnss_valid) {
            g_pos = Vector3(gnss_pos.x, gnss_pos.y, gnss_pos.z);
            g_vel = Vector3(gnss_vel.x, gnss_vel.y, gnss_vel.z);
        }

        std::optional<double> speed_opt = (ai_speed >= 0.0) ? std::optional<double>(ai_speed) : std::nullopt;

        std::optional<Vector3> m_p = std::nullopt;
        if (mag_valid) {
            m_p = Vector3(mag_phone.x, mag_phone.y, mag_phone.z);
        }

        NavigationTelemetry telem = engine->process_step(
            time_s, a_p, w_p, baro_alt, g_pos, (gnss_valid != 0), speed_opt, g_vel,
            m_p, (enable_road_constraints != 0)
        );

        out_telem->mode = static_cast<int32_t>(telem.mode);
        out_telem->time_s = telem.time_s;
        out_telem->pos_enu = {telem.pos_enu.x, telem.pos_enu.y, telem.pos_enu.z};
        out_telem->vel_enu = {telem.vel_enu.x, telem.vel_enu.y, telem.vel_enu.z};
        out_telem->speed_kmh = telem.speed_kmh;
        out_telem->heading_deg = telem.heading_deg;
        out_telem->pitch_deg = telem.pitch_deg;
        out_telem->roll_deg = telem.roll_deg;
        out_telem->accel_bias = {telem.accel_bias.x, telem.accel_bias.y, telem.accel_bias.z};
        out_telem->gyro_bias = {telem.gyro_bias.x, telem.gyro_bias.y, telem.gyro_bias.z};
        out_telem->in_outage = telem.in_outage ? 1 : 0;
        out_telem->outage_duration_s = telem.outage_duration_s;
        out_telem->outage_dist_traveled_m = telem.outage_dist_traveled_m;
        out_telem->gnss_valid = telem.gnss_valid ? 1 : 0;
        out_telem->is_pothole_active = telem.is_pothole_active ? 1 : 0;
        out_telem->is_idling = telem.is_idling ? 1 : 0;
        out_telem->p_trace = telem.diagnostics.p_trace;

        return 0;
    } catch (...) {
        return -2;
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
    if (!handle) return;
    auto* engine = static_cast<SeamlessGNSSDeficitEngine*>(handle);
    auto* graph = engine->get_osm_graph_mutable();
    if (graph) {
        std::vector<Vector3> polyline;
        polyline.reserve(num_pts);
        for (int i = 0; i < num_pts; ++i) {
            polyline.emplace_back(
                polyline_xyz[3 * i + 0],
                polyline_xyz[3 * i + 1],
                polyline_xyz[3 * i + 2]
            );
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

}

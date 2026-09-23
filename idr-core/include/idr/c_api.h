#ifndef IDR_C_API_H
#define IDR_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef struct {
    double x;
    double y;
    double z;
} idr_vec3_t;

typedef struct {
    int32_t mode;                    // 0: INITIALIZING, 1: GNSS_AIDED, 2: GNSS_DEGRADED, 3: TRANSITION_TO_DR, 4: DR_ONLY, 5: TRANSITION_TO_GNSS, 6: GNSS_REACQUIRED
    double time_s;
    idr_vec3_t pos_enu;
    idr_vec3_t vel_enu;
    double speed_kmh;
    double heading_deg;
    double pitch_deg;
    double roll_deg;
    idr_vec3_t accel_bias;
    idr_vec3_t gyro_bias;
    int32_t in_outage;
    double outage_duration_s;
    double outage_dist_traveled_m;
    int32_t gnss_valid;
    int32_t is_pothole_active;
    int32_t is_idling;
    double p_trace;
} idr_telemetry_t;

typedef void* idr_engine_handle_t;

// Engine lifecycle
idr_engine_handle_t idr_engine_create(double dt);
idr_engine_handle_t idr_engine_create_with_state(
    double dt,
    idr_vec3_t init_pos,
    idr_vec3_t init_vel,
    idr_vec3_t init_euler
);
idr_engine_handle_t idr_engine_create_full(
    double dt,
    idr_vec3_t init_pos,
    idr_vec3_t init_vel,
    idr_vec3_t init_euler,
    int32_t enable_heading_supervisor,
    int32_t enable_map_matching,
    int32_t enable_speed_calibration,
    int32_t enable_longitudinal_map_constraint
);

void idr_engine_destroy(idr_engine_handle_t handle);

// Step process
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
);

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
);

// OSM Road Network Map Graph API
void idr_engine_clear_osm_graph(idr_engine_handle_t handle);

void idr_engine_add_osm_node(
    idr_engine_handle_t handle,
    int32_t node_id,
    double lat,
    double lon,
    double alt,
    double enu_x,
    double enu_y,
    double enu_z
);

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
);

#ifdef __cplusplus
}
#endif

#endif // IDR_C_API_H

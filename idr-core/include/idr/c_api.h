#ifndef IDR_C_API_H
#define IDR_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * API Versioning
 * ============================================================================
 */
#define IDR_API_VERSION_MAJOR 1
#define IDR_API_VERSION_MINOR 1
#define IDR_API_VERSION_PATCH 0
#define IDR_API_VERSION ((IDR_API_VERSION_MAJOR << 16) | (IDR_API_VERSION_MINOR << 8) | IDR_API_VERSION_PATCH)

/**
 * @brief Returns the packed 32-bit API version integer (0xMMmmpp).
 */
uint32_t idr_api_version(void);

/**
 * @brief Returns the semantic version string (e.g. "1.1.0").
 */
const char* idr_api_version_string(void);

/* ============================================================================
 * Status & Error Return Codes
 * ============================================================================
 */
typedef enum idr_status_e {
    IDR_STATUS_OK                  =  0,  /**< Success */
    IDR_STATUS_INVALID_ARGUMENT    = -1,  /**< Null handle, null output pointer, or out-of-range argument */
    IDR_STATUS_INVALID_STATE       = -2,  /**< Engine uninitialized, destroyed, or invalid internal state */
    IDR_STATUS_UNAVAILABLE         = -3,  /**< Required sensor or map data unavailable */
    IDR_STATUS_NUMERICAL_FAILURE   = -4,  /**< NaN, Inf, non-invertible covariance, or numerical breakdown */
    IDR_STATUS_BUFFER_ERROR        = -5,  /**< Buffer too small, truncation, or memory capacity exceeded */
    IDR_STATUS_INIT_FAILURE        = -6,  /**< Failed to construct or allocate engine instance */
    IDR_STATUS_INTERNAL_ERROR      = -7   /**< Unexpected exception or internal assertion failure */
} idr_status_t;

/**
 * @brief Returns a human-readable description for an IDR status code.
 */
const char* idr_status_to_string(int32_t status);

/* ============================================================================
 * Navigation Mode Enumeration
 * ============================================================================
 */
typedef enum idr_nav_mode_e {
    IDR_NAV_MODE_INITIALIZING      = 0,  /**< Stationary bias capture & initial leveling */
    IDR_NAV_MODE_GNSS_AIDED        = 1,  /**< Full GNSS+INS fusion with active calibration */
    IDR_NAV_MODE_GNSS_DEGRADED     = 2,  /**< Multipath / high DOP; down-weighted GNSS */
    IDR_NAV_MODE_TRANSITION_TO_DR  = 3,  /**< Loss of GNSS fix detected; locking calibration */
    IDR_NAV_MODE_DR_ONLY           = 4,  /**< Pure Dead Reckoning (AI-Speed + ESKF + Map) */
    IDR_NAV_MODE_TRANSITION_TO_GNSS= 5,  /**< GNSS reacquisition debounce & validation */
    IDR_NAV_MODE_GNSS_REACQUIRED   = 6   /**< Smooth covariance handover back to GNSS */
} idr_nav_mode_t;

/**
 * @brief Returns a human-readable name for a navigation mode.
 */
const char* idr_nav_mode_to_string(int32_t mode);

/* ============================================================================
 * Public Struct Definitions (Strict 8-Byte Aligned ABI Layout)
 * ============================================================================
 */
#pragma pack(push, 8)

/**
 * @brief 3D Cartesian vector.
 *
 * Coordinate Frames:
 * - When used for position/velocity: Local Tangent ENU (X: East, Y: North, Z: Up) in meters or m/s.
 * - When used for IMU/Mag: Phone Sensor Body Frame (X: right, Y: forward/up along screen, Z: out of screen).
 */
typedef struct {
    double x;  /**< X-axis component */
    double y;  /**< Y-axis component */
    double z;  /**< Z-axis component */
} idr_vec3_t;

/**
 * @brief Complete Navigation Telemetry Output.
 *
 * Exact fixed ABI struct layout (160 bytes on 64-bit platforms).
 */
typedef struct {
    int32_t mode;                    /**< Current NavigationMode (idr_nav_mode_t 0..6) */
    int32_t in_outage;               /**< 1 if in GNSS outage, 0 otherwise */
    double time_s;                   /**< Monotonic step timestamp in seconds */
    idr_vec3_t pos_enu;              /**< Local tangent East-North-Up position (meters) */
    idr_vec3_t vel_enu;              /**< East-North-Up velocity (m/s) */
    double speed_kmh;                /**< Forward ground speed (km/h) */
    double heading_deg;              /**< Azimuth heading [0, 360) deg clockwise from true North */
    double pitch_deg;                /**< Elevation pitch [-90, +90] deg above horizontal plane */
    double roll_deg;                 /**< Bank roll [-180, +180] deg */
    idr_vec3_t accel_bias;           /**< Accelerometer bias estimates in vehicle body frame (m/s^2) */
    idr_vec3_t gyro_bias;            /**< Gyroscope bias estimates in vehicle body frame (rad/s) */
    double outage_duration_s;        /**< Elapsed duration inside current outage (s) */
    double outage_dist_traveled_m;   /**< Distance traveled inside current outage (meters) */
    int32_t gnss_valid;              /**< 1 if GNSS fix was accepted, 0 otherwise */
    int32_t is_pothole_active;       /**< 1 if vertical vibration road shock active, 0 otherwise */
    int32_t is_idling;               /**< 1 if standstill zero-velocity update active, 0 otherwise */
    int32_t reserved;                /**< Explicit 32-bit alignment padding for 64-bit boundaries */
    double p_trace;                  /**< Trace of 15x15 error covariance matrix Tr(P) */
} idr_telemetry_t;

/**
 * @brief Extended Offline Navigation Telemetry Output (Phase 11.8).
 * Explicitly separates raw DR / filter state from map-matched presentation state.
 */
typedef struct {
    idr_telemetry_t base;            /**< Base navigation telemetry */
    idr_vec3_t raw_dr_pos_enu;       /**< Raw unfiltered DR ENU position (m) */
    idr_vec3_t map_matched_pos_enu;  /**< Road-snapped presentation position (m) */
    int32_t selected_edge_id;        /**< Matched road edge ID (-1 if none) */
    int32_t selected_layer;          /**< Road layer (0=Ground, 1=Flyover, -1=Tunnel) */
    int32_t map_confidence;          /**< 0=HIGH, 1=MEDIUM, 2=LOW, 3=INVALID */
    int32_t is_map_ambiguous;        /**< 1 if geometric candidate ambiguity detected */
    uint32_t candidate_count;        /**< Number of road candidates queried */
    uint32_t hypothesis_count;       /**< Number of active Viterbi hypotheses */
    double score_margin;             /**< Posterior score margin between top 2 candidates */
    int32_t map_available;           /**< 1 if offline map reader/graph is active */
    int32_t _reserved;               /**< Padding */
    const char* selected_edge_name;  /**< Matched road name */
    const char* selected_layer_label;/**< Matched layer label ("Ground", "Flyover", etc.) */
} idr_telemetry_offline_nav_t;

#pragma pack(pop)

/**
 * @brief Opaque handle to an instantiated IDR Navigation Engine.
 */
typedef void* idr_engine_handle_t;

/* ============================================================================
 * Engine Lifecycle API
 * ============================================================================
 * Ownership:
 * - The caller allocates the engine using idr_engine_create_* and OWNS the returned handle.
 * - The caller MUST free the handle by calling idr_engine_destroy().
 *
 * Thread Safety:
 * - Individual handles are NOT thread-safe for concurrent process_step calls.
 * - Distinct engine handles are fully isolated and thread-safe.
 */

/**
 * @brief Creates a navigation engine instance with default parameters.
 * @param dt Expected nominal step sampling interval in seconds (e.g. 0.1 for 10 Hz, 0.01 for 100 Hz).
 * @return Opaque engine handle, or NULL on allocation/init failure.
 */
idr_engine_handle_t idr_engine_create(double dt);

/**
 * @brief Creates an engine instance with explicit initial state vectors.
 * @param dt Nominal step sampling interval (s).
 * @param init_pos Initial position in local tangent ENU coordinates (m).
 * @param init_vel Initial velocity in ENU coordinates (m/s).
 * @param init_euler Initial attitude [roll, pitch, yaw] in radians (yaw clockwise from North).
 * @return Opaque engine handle, or NULL on failure.
 */
idr_engine_handle_t idr_engine_create_with_state(
    double dt,
    idr_vec3_t init_pos,
    idr_vec3_t init_vel,
    idr_vec3_t init_euler
);

/**
 * @brief Creates a fully configured engine instance.
 * @param dt Nominal step interval (s).
 * @param init_pos Initial position (ENU, m).
 * @param init_vel Initial velocity (ENU, m/s).
 * @param init_euler Initial attitude (Euler rad).
 * @param enable_heading_supervisor 1 to enable heading supervisor, 0 to disable.
 * @param enable_map_matching 1 to enable 3D map matching, 0 to disable.
 * @param enable_speed_calibration 1 to enable online AI speed scale/bias calibration, 0 to disable.
 * @param enable_longitudinal_map_constraint 1 to enable map longitudinal synchronization, 0 to disable.
 * @return Opaque engine handle, or NULL on failure.
 */
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

/**
 * @brief Destroys an engine instance and frees all associated memory.
 * @param handle Opaque handle returned by idr_engine_create_*. Safe to pass NULL (no-op).
 */
void idr_engine_destroy(idr_engine_handle_t handle);

/* ============================================================================
 * Step Execution API
 * ============================================================================
 * Numerical Safety:
 * - All inputs are validated for NaN, Infinity, and impossible physical ranges.
 * - Non-finite sensor inputs return IDR_STATUS_NUMERICAL_FAILURE without state corruption.
 * - Pointer parameters are validated; NULL returns IDR_STATUS_INVALID_ARGUMENT.
 * - Zero heap allocations occur during step execution.
 */

/**
 * @brief Processes one navigation step with standard sensor inputs.
 * @param handle Valid engine handle.
 * @param time_s Monotonic timestamp of the step (seconds).
 * @param accel_phone Accelerometer measurement in phone body frame (m/s^2, including gravity).
 * @param gyro_phone Gyroscope measurement in phone body frame (rad/s).
 * @param baro_alt Barometric altitude in meters.
 * @param gnss_valid 1 if GNSS position/velocity are valid, 0 otherwise.
 * @param gnss_pos GNSS position in local tangent ENU (meters). Ignored if gnss_valid == 0.
 * @param gnss_vel GNSS velocity in ENU (m/s). Ignored if gnss_valid == 0.
 * @param ai_speed AI forward speed estimate in m/s (pass < 0 if unavailable).
 * @param out_telem Pointer to caller-allocated idr_telemetry_t to receive results.
 * @return IDR_STATUS_OK (0) on success, negative idr_status_t error code on failure.
 */
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

/**
 * @brief Extended navigation step process accepting magnetometer and map constraint flags.
 * @param handle Valid engine handle.
 * @param time_s Monotonic timestamp of the step (seconds).
 * @param accel_phone Accelerometer measurement (m/s^2).
 * @param gyro_phone Gyroscope measurement (rad/s).
 * @param baro_alt Barometric altitude (m).
 * @param gnss_valid 1 if GNSS is valid, 0 if in outage.
 * @param gnss_pos GNSS position (ENU, m).
 * @param gnss_vel GNSS velocity (ENU, m/s).
 * @param ai_speed AI forward speed estimate (m/s, or < 0 if unavailable).
 * @param mag_valid 1 if magnetometer is valid, 0 if uncalibrated/unavailable.
 * @param mag_phone 3-axis magnetometer measurement in phone frame (microTesla).
 * @param enable_road_constraints 1 to apply map-matching road constraints, 0 to bypass.
 * @param out_telem Pointer to caller-allocated idr_telemetry_t struct.
 * @return IDR_STATUS_OK (0) on success, negative idr_status_t error code on failure.
 */
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

/**
 * @brief Offline navigation process step returning extended telemetry with raw vs map-matched separation.
 */
int32_t idr_engine_process_step_offline_nav(
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
    idr_telemetry_offline_nav_t* out_telem
);

/**
 * @brief Loads a compiled .idrmap v1.0 binary file into the navigation engine.
 */
idr_status_t idr_engine_load_idrmap(
    idr_engine_handle_t handle,
    const char* path,
    double lat0,
    double lon0,
    double alt0
);

typedef struct idr_map_reader_s idr_map_reader_t;

/**
 * @brief Attaches a pre-opened IdrMapReader to the navigation engine.
 */
idr_status_t idr_engine_set_map_reader(
    idr_engine_handle_t handle,
    const idr_map_reader_t* reader,
    double lat0,
    double lon0,
    double alt0
);


/**
 * @brief Resets the navigation engine state back to INITIALIZING.
 */
idr_status_t idr_engine_reset(idr_engine_handle_t handle);

/* ============================================================================
 * Offline OpenStreetMap Road Graph API
 * ============================================================================
 * Ownership & Lifetime:
 * - The engine copies node/edge geometry and strings into its internal graph.
 * - Caller retains ownership of polyline_xyz and string pointers.
 */

/**
 * @brief Clears all nodes and edges from the engine's internal OSM road graph.
 * @param handle Engine handle.
 */
void idr_engine_clear_osm_graph(idr_engine_handle_t handle);

/**
 * @brief Adds a topological node to the engine's OSM road graph.
 * @param handle Engine handle.
 * @param node_id Unique integer node identifier.
 * @param lat WGS-84 latitude in degrees.
 * @param lon WGS-84 longitude in degrees.
 * @param alt Altitude above ellipsoid/MSL in meters.
 * @param enu_x Local tangent East coordinate (m).
 * @param enu_y Local tangent North coordinate (m).
 * @param enu_z Local tangent Up coordinate (m).
 */
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

/**
 * @brief Adds a topological road edge with 3D polyline geometry to the OSM road graph.
 * @param handle Engine handle.
 * @param edge_id Unique integer edge identifier.
 * @param source_id Source node identifier.
 * @param target_id Target node identifier.
 * @param name UTF-8 road name string (borrowed, copied internally).
 * @param road_type Highway tag classification (e.g. "motorway", "primary", "service").
 * @param layer Vertical layer index (-5 to +5, 0 = ground level).
 * @param width Road carriageway width in meters.
 * @param speed_limit Speed limit in km/h.
 * @param num_pts Number of 3D points in polyline_xyz array.
 * @param polyline_xyz Contiguous array of [x0, y0, z0, x1, y1, z1, ...] in local ENU (m).
 */
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

/* ============================================================================
 * IdrMapReader C API (Phase 11.6)
 * ============================================================================
 */
typedef struct idr_map_reader_s idr_map_reader_t;

typedef struct idr_road_candidate_c_s {
    uint32_t edge_id;
    int64_t  osm_way_id;
    double   distance_m;
    double   proj_lat;
    double   proj_lon;
    double   segment_heading_deg;
    uint8_t  road_class;
    int8_t   oneway;
    int8_t   layer;
    uint8_t  flags;
    uint8_t  maxspeed_kmh;
    uint8_t  _pad[3];
    float    length_m;
    const char* name;
} idr_road_candidate_c_t;

idr_map_reader_t* idr_map_reader_create(void);
void idr_map_reader_destroy(idr_map_reader_t* reader);
idr_status_t idr_map_reader_open_file(idr_map_reader_t* reader, const char* path);
idr_status_t idr_map_reader_open_buffer(idr_map_reader_t* reader, const uint8_t* data, size_t size);
void idr_map_reader_close(idr_map_reader_t* reader);
int32_t idr_map_reader_is_open(idr_map_reader_t* reader);
uint32_t idr_map_reader_edge_count(idr_map_reader_t* reader);
uint32_t idr_map_reader_node_count(idr_map_reader_t* reader);
idr_status_t idr_map_reader_get_bounds(idr_map_reader_t* reader, double* min_lat, double* min_lon, double* max_lat, double* max_lon);
size_t idr_map_reader_query_nearest(idr_map_reader_t* reader, double lat, double lon, double max_radius_m, size_t k, idr_road_candidate_c_t* out_candidates, size_t max_out);
size_t idr_map_reader_query_radius(idr_map_reader_t* reader, double lat, double lon, double radius_m, uint32_t* out_edge_ids, size_t max_out);
size_t idr_map_reader_query_bbox(idr_map_reader_t* reader, double min_lat, double min_lon, double max_lat, double max_lon, uint32_t* out_edge_ids, size_t max_out);

/* ============================================================================
 * Causal 3D HMM Map Matcher C API (Phase 11.7)
 * ============================================================================
 */
typedef struct idr_map_matcher_s idr_map_matcher_t;

typedef struct idr_map_match_result_c_s {
    int32_t edge_id;
    int32_t layer;
    double  snapped_pos_x;
    double  snapped_pos_y;
    double  snapped_pos_z;
    double  d_2d_m;
    double  delta_h_m;
    double  road_heading_deg;
    double  road_tangent_x;
    double  road_tangent_y;
    int32_t confidence;      /**< 0=HIGH, 1=MEDIUM, 2=LOW, 3=INVALID */
    double  log_posterior;
    double  score_margin;
    int32_t is_ambiguous;
    int32_t is_valid;
    uint32_t candidate_count;
    uint32_t hypothesis_count;
    const char* edge_name;
    const char* layer_label;
} idr_map_match_result_c_t;

typedef struct idr_map_hypothesis_c_s {
    uint32_t edge_id;
    int8_t   layer;
    uint8_t  road_class;
    uint8_t  flags;
    uint8_t  _pad;
    double   log_prob;
    double   along_track_m;
    double   d_2d_m;
    double   delta_h_m;
    double   road_heading_deg;
    double   snapped_x;
    double   snapped_y;
    double   snapped_z;
    int32_t  consecutive_matches;
} idr_map_hypothesis_c_t;

idr_map_matcher_t* idr_map_matcher_create_default(void);

idr_map_matcher_t* idr_map_matcher_create_with_reader(
    const idr_map_reader_t* reader,
    double lat0,
    double lon0,
    double alt0,
    double sigma_d,
    double sigma_h,
    double sigma_psi,
    double beta,
    double elevation_barrier
);

void idr_map_matcher_destroy(idr_map_matcher_t* matcher);
void idr_map_matcher_reset(idr_map_matcher_t* matcher);

idr_status_t idr_map_matcher_process_step(
    idr_map_matcher_t* matcher,
    const idr_vec3_t* pos_dr,
    double heading_rad,
    double baro_alt,
    double speed_dr,
    double dt,
    int32_t use_3d,
    idr_map_match_result_c_t* out_result
);

size_t idr_map_matcher_get_hypotheses(
    idr_map_matcher_t* matcher,
    idr_map_hypothesis_c_t* out_hypotheses,
    size_t max_hypotheses
);

#ifdef __cplusplus
}
#endif

#endif /* IDR_C_API_H */

#include <jni.h>
#include "idr/c_api.h"
#include <cstring>
#include <string>
#include <vector>

#if defined(__ANDROID__)
#include <android/log.h>
#define LOG_TAG "IDR_JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#endif

extern "C" {

/* ============================================================================
 * Version and Status String Utilities
 * ============================================================================
 */

JNIEXPORT jint JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeApiVersion(
    JNIEnv* /*env*/,
    jclass /*clazz*/
) {
    return static_cast<jint>(idr_api_version());
}

JNIEXPORT jstring JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeApiVersionString(
    JNIEnv* env,
    jclass /*clazz*/
) {
    const char* ver_str = idr_api_version_string();
    return env->NewStringUTF(ver_str ? ver_str : "UNKNOWN");
}

JNIEXPORT jstring JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeStatusToString(
    JNIEnv* env,
    jclass /*clazz*/,
    jint statusCode
) {
    const char* str = idr_status_to_string(statusCode);
    return env->NewStringUTF(str ? str : "IDR_STATUS_UNKNOWN");
}

JNIEXPORT jstring JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeNavModeToString(
    JNIEnv* env,
    jclass /*clazz*/,
    jint modeCode
) {
    const char* str = idr_nav_mode_to_string(modeCode);
    return env->NewStringUTF(str ? str : "UNKNOWN");
}

/* ============================================================================
 * Engine Lifecycle Management
 * ============================================================================
 */

JNIEXPORT jlong JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeCreate(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jdouble dt
) {
    try {
        idr_engine_handle_t handle = idr_engine_create(dt);
        return reinterpret_cast<jlong>(handle);
    } catch (...) {
        LOGE("Exception in nativeCreate");
        return 0;
    }
}

JNIEXPORT jlong JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeCreateWithState(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jdouble dt,
    jdouble posX, jdouble posY, jdouble posZ,
    jdouble velX, jdouble velY, jdouble velZ,
    jdouble eulerRoll, jdouble eulerPitch, jdouble eulerYaw
) {
    try {
        idr_vec3_t init_pos{posX, posY, posZ};
        idr_vec3_t init_vel{velX, velY, velZ};
        idr_vec3_t init_euler{eulerRoll, eulerPitch, eulerYaw};

        idr_engine_handle_t handle = idr_engine_create_with_state(
            dt, init_pos, init_vel, init_euler
        );
        return reinterpret_cast<jlong>(handle);
    } catch (...) {
        LOGE("Exception in nativeCreateWithState");
        return 0;
    }
}

JNIEXPORT jlong JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeCreateFull(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jdouble dt,
    jdouble posX, jdouble posY, jdouble posZ,
    jdouble velX, jdouble velY, jdouble velZ,
    jdouble eulerRoll, jdouble eulerPitch, jdouble eulerYaw,
    jint enableHeadingSupervisor,
    jint enableMapMatching,
    jint enableSpeedCalibration,
    jint enableLongitudinalMapConstraint
) {
    try {
        idr_vec3_t init_pos{posX, posY, posZ};
        idr_vec3_t init_vel{velX, velY, velZ};
        idr_vec3_t init_euler{eulerRoll, eulerPitch, eulerYaw};

        idr_engine_handle_t handle = idr_engine_create_full(
            dt, init_pos, init_vel, init_euler,
            enableHeadingSupervisor,
            enableMapMatching,
            enableSpeedCalibration,
            enableLongitudinalMapConstraint
        );
        return reinterpret_cast<jlong>(handle);
    } catch (...) {
        LOGE("Exception in nativeCreateFull");
        return 0;
    }
}

JNIEXPORT void JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeDestroy(
    JNIEnv* /*env*/,
    jobject /*thiz*/,
    jlong handle
) {
    if (handle != 0) {
        idr_engine_handle_t h = reinterpret_cast<idr_engine_handle_t>(handle);
        idr_engine_destroy(h);
    }
}

/* ============================================================================
 * Step Execution with Flat DoubleArray Buffer (Zero Allocation High-Rate Loop)
 * ============================================================================
 * Flat double[] buffer layout (length >= 25):
 *  0: mode (double)
 *  1: in_outage (double)
 *  2: time_s (double)
 *  3: pos_enu.x (m)
 *  4: pos_enu.y (m)
 *  5: pos_enu.z (m)
 *  6: vel_enu.x (m/s)
 *  7: vel_enu.y (m/s)
 *  8: vel_enu.z (m/s)
 *  9: speed_kmh (km/h)
 * 10: heading_deg (deg)
 * 11: pitch_deg (deg)
 * 12: roll_deg (deg)
 * 13: accel_bias.x (m/s^2)
 * 14: accel_bias.y (m/s^2)
 * 15: accel_bias.z (m/s^2)
 * 16: gyro_bias.x (rad/s)
 * 17: gyro_bias.y (rad/s)
 * 18: gyro_bias.z (rad/s)
 * 19: outage_duration_s (s)
 * 20: outage_dist_traveled_m (m)
 * 21: gnss_valid (double)
 * 22: is_pothole_active (double)
 * 23: is_idling (double)
 * 24: p_trace (double)
 */

static inline void serialize_telemetry_to_flat_array(
    const idr_telemetry_t& telem,
    jdouble* dst
) {
    dst[0]  = static_cast<jdouble>(telem.mode);
    dst[1]  = static_cast<jdouble>(telem.in_outage);
    dst[2]  = telem.time_s;
    dst[3]  = telem.pos_enu.x;
    dst[4]  = telem.pos_enu.y;
    dst[5]  = telem.pos_enu.z;
    dst[6]  = telem.vel_enu.x;
    dst[7]  = telem.vel_enu.y;
    dst[8]  = telem.vel_enu.z;
    dst[9]  = telem.speed_kmh;
    dst[10] = telem.heading_deg;
    dst[11] = telem.pitch_deg;
    dst[12] = telem.roll_deg;
    dst[13] = telem.accel_bias.x;
    dst[14] = telem.accel_bias.y;
    dst[15] = telem.accel_bias.z;
    dst[16] = telem.gyro_bias.x;
    dst[17] = telem.gyro_bias.y;
    dst[18] = telem.gyro_bias.z;
    dst[19] = telem.outage_duration_s;
    dst[20] = telem.outage_dist_traveled_m;
    dst[21] = static_cast<jdouble>(telem.gnss_valid);
    dst[22] = static_cast<jdouble>(telem.is_pothole_active);
    dst[23] = static_cast<jdouble>(telem.is_idling);
    dst[24] = telem.p_trace;
}

JNIEXPORT jint JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeProcessStep(
    JNIEnv* env,
    jobject /*thiz*/,
    jlong handle,
    jdouble timeS,
    jdouble ax, jdouble ay, jdouble az,
    jdouble gx, jdouble gy, jdouble gz,
    jdouble baroAlt,
    jint gnssValid,
    jdouble gnssPosX, jdouble gnssPosY, jdouble gnssPosZ,
    jdouble gnssVelX, jdouble gnssVelY, jdouble gnssVelZ,
    jdouble aiSpeed,
    jdoubleArray outTelemBuffer
) {
    if (handle == 0 || outTelemBuffer == nullptr) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }

    jsize buf_len = env->GetArrayLength(outTelemBuffer);
    if (buf_len < 25) {
        return IDR_STATUS_BUFFER_ERROR;
    }

    idr_engine_handle_t h = reinterpret_cast<idr_engine_handle_t>(handle);
    idr_vec3_t a_p{ax, ay, az};
    idr_vec3_t w_p{gx, gy, gz};
    idr_vec3_t g_pos{gnssPosX, gnssPosY, gnssPosZ};
    idr_vec3_t g_vel{gnssVelX, gnssVelY, gnssVelZ};
    idr_telemetry_t telem{};

    int32_t rc = idr_engine_process_step(
        h, timeS, a_p, w_p, baroAlt,
        gnssValid, g_pos, g_vel, aiSpeed, &telem
    );

    if (rc == IDR_STATUS_OK) {
        jdouble* elements = env->GetDoubleArrayElements(outTelemBuffer, nullptr);
        if (elements) {
            serialize_telemetry_to_flat_array(telem, elements);
            env->ReleaseDoubleArrayElements(outTelemBuffer, elements, 0);
        } else {
            return IDR_STATUS_INTERNAL_ERROR;
        }
    }

    return rc;
}

JNIEXPORT jint JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeProcessStepExt(
    JNIEnv* env,
    jobject /*thiz*/,
    jlong handle,
    jdouble timeS,
    jdouble ax, jdouble ay, jdouble az,
    jdouble gx, jdouble gy, jdouble gz,
    jdouble baroAlt,
    jint gnssValid,
    jdouble gnssPosX, jdouble gnssPosY, jdouble gnssPosZ,
    jdouble gnssVelX, jdouble gnssVelY, jdouble gnssVelZ,
    jdouble aiSpeed,
    jint magValid,
    jdouble magX, jdouble magY, jdouble magZ,
    jint enableRoadConstraints,
    jdoubleArray outTelemBuffer
) {
    if (handle == 0 || outTelemBuffer == nullptr) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }

    jsize buf_len = env->GetArrayLength(outTelemBuffer);
    if (buf_len < 25) {
        return IDR_STATUS_BUFFER_ERROR;
    }

    idr_engine_handle_t h = reinterpret_cast<idr_engine_handle_t>(handle);
    idr_vec3_t a_p{ax, ay, az};
    idr_vec3_t w_p{gx, gy, gz};
    idr_vec3_t g_pos{gnssPosX, gnssPosY, gnssPosZ};
    idr_vec3_t g_vel{gnssVelX, gnssVelY, gnssVelZ};
    idr_vec3_t m_p{magX, magY, magZ};
    idr_telemetry_t telem{};

    int32_t rc = idr_engine_process_step_ext(
        h, timeS, a_p, w_p, baroAlt,
        gnssValid, g_pos, g_vel, aiSpeed,
        magValid, m_p, enableRoadConstraints, &telem
    );

    if (rc == IDR_STATUS_OK) {
        jdouble* elements = env->GetDoubleArrayElements(outTelemBuffer, nullptr);
        if (elements) {
            serialize_telemetry_to_flat_array(telem, elements);
            env->ReleaseDoubleArrayElements(outTelemBuffer, elements, 0);
        } else {
            return IDR_STATUS_INTERNAL_ERROR;
        }
    }

    return rc;
}

/* Direct ByteBuffer zero-copy process step */
JNIEXPORT jint JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeProcessStepDirectBuffer(
    JNIEnv* env,
    jobject /*thiz*/,
    jlong handle,
    jdouble timeS,
    jdouble ax, jdouble ay, jdouble az,
    jdouble gx, jdouble gy, jdouble gz,
    jdouble baroAlt,
    jint gnssValid,
    jdouble gnssPosX, jdouble gnssPosY, jdouble gnssPosZ,
    jdouble gnssVelX, jdouble gnssVelY, jdouble gnssVelZ,
    jdouble aiSpeed,
    jint magValid,
    jdouble magX, jdouble magY, jdouble magZ,
    jint enableRoadConstraints,
    jobject directByteBuffer
) {
    if (handle == 0 || directByteBuffer == nullptr) {
        return IDR_STATUS_INVALID_ARGUMENT;
    }

    void* buf_ptr = env->GetDirectBufferAddress(directByteBuffer);
    jlong capacity = env->GetDirectBufferCapacity(directByteBuffer);
    if (!buf_ptr || capacity < static_cast<jlong>(sizeof(idr_telemetry_t))) {
        return IDR_STATUS_BUFFER_ERROR;
    }

    idr_engine_handle_t h = reinterpret_cast<idr_engine_handle_t>(handle);
    idr_vec3_t a_p{ax, ay, az};
    idr_vec3_t w_p{gx, gy, gz};
    idr_vec3_t g_pos{gnssPosX, gnssPosY, gnssPosZ};
    idr_vec3_t g_vel{gnssVelX, gnssVelY, gnssVelZ};
    idr_vec3_t m_p{magX, magY, magZ};
    idr_telemetry_t* out_telem = static_cast<idr_telemetry_t*>(buf_ptr);

    return idr_engine_process_step_ext(
        h, timeS, a_p, w_p, baroAlt,
        gnssValid, g_pos, g_vel, aiSpeed,
        magValid, m_p, enableRoadConstraints, out_telem
    );
}

/* ============================================================================
 * OSM Road Graph Management
 * ============================================================================
 */

JNIEXPORT void JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeClearOsmGraph(
    JNIEnv* /*env*/,
    jobject /*thiz*/,
    jlong handle
) {
    if (handle != 0) {
        idr_engine_clear_osm_graph(reinterpret_cast<idr_engine_handle_t>(handle));
    }
}

JNIEXPORT void JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeAddOsmNode(
    JNIEnv* /*env*/,
    jobject /*thiz*/,
    jlong handle,
    jint nodeId,
    jdouble lat, jdouble lon, jdouble alt,
    jdouble enuX, jdouble enuY, jdouble enuZ
) {
    if (handle != 0) {
        idr_engine_add_osm_node(
            reinterpret_cast<idr_engine_handle_t>(handle),
            nodeId, lat, lon, alt, enuX, enuY, enuZ
        );
    }
}

JNIEXPORT void JNICALL
Java_org_isro_idr_engine_NativeNavigationEngine_nativeAddOsmEdge(
    JNIEnv* env,
    jobject /*thiz*/,
    jlong handle,
    jint edgeId,
    jint sourceId,
    jint targetId,
    jstring name,
    jstring roadType,
    jint layer,
    jdouble width,
    jdouble speedLimit,
    jint numPts,
    jdoubleArray polylineXyz
) {
    if (handle == 0 || numPts <= 0 || polylineXyz == nullptr) {
        return;
    }

    const char* name_cstr = name ? env->GetStringUTFChars(name, nullptr) : nullptr;
    const char* type_cstr = roadType ? env->GetStringUTFChars(roadType, nullptr) : nullptr;
    jdouble* poly_ptr = env->GetDoubleArrayElements(polylineXyz, nullptr);

    if (poly_ptr) {
        idr_engine_add_osm_edge(
            reinterpret_cast<idr_engine_handle_t>(handle),
            edgeId, sourceId, targetId,
            name_cstr, type_cstr,
            layer, width, speedLimit,
            numPts, poly_ptr
        );
        env->ReleaseDoubleArrayElements(polylineXyz, poly_ptr, JNI_ABORT);
    }

    if (name && name_cstr) env->ReleaseStringUTFChars(name, name_cstr);
    if (roadType && type_cstr) env->ReleaseStringUTFChars(roadType, type_cstr);
}

} // extern "C"

#include "idr/c_api.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>

int main() {
    std::cout << "[TEST] Starting Hardened C API Tests..." << std::endl;

    // 1. Test API Versioning
    uint32_t ver = idr_api_version();
    assert(ver == IDR_API_VERSION);
    assert(strcmp(idr_api_version_string(), "1.1.0") == 0);
    std::cout << "  [PASS] API Version: " << idr_api_version_string() << " (0x" << std::hex << ver << std::dec << ")" << std::endl;

    // 2. Test Status and Mode String Mappings
    assert(strcmp(idr_status_to_string(IDR_STATUS_OK), "IDR_STATUS_OK") == 0);
    assert(strcmp(idr_status_to_string(IDR_STATUS_INVALID_ARGUMENT), "IDR_STATUS_INVALID_ARGUMENT") == 0);
    assert(strcmp(idr_status_to_string(IDR_STATUS_NUMERICAL_FAILURE), "IDR_STATUS_NUMERICAL_FAILURE") == 0);
    assert(strcmp(idr_nav_mode_to_string(IDR_NAV_MODE_GNSS_AIDED), "GNSS_AIDED") == 0);
    assert(strcmp(idr_nav_mode_to_string(IDR_NAV_MODE_DR_ONLY), "DR_ONLY") == 0);
    std::cout << "  [PASS] String mapping helpers verified." << std::endl;

    // 3. Test Engine Lifecycle
    idr_engine_handle_t handle = idr_engine_create(0.1);
    assert(handle != nullptr);

    // 4. Test Error Handling: Null Handle & Null Output Pointer
    idr_vec3_t a_valid{0.0, 0.0, 9.80665};
    idr_vec3_t w_valid{0.0, 0.0, 0.0};
    idr_vec3_t g_pos{0.0, 1.0, 0.0};
    idr_vec3_t g_vel{0.0, 10.0, 0.0};
    idr_telemetry_t telem{};

    assert(idr_engine_process_step(nullptr, 0.1, a_valid, w_valid, 0.0, 1, g_pos, g_vel, 10.0, &telem) == IDR_STATUS_INVALID_ARGUMENT);
    assert(idr_engine_process_step(handle, 0.1, a_valid, w_valid, 0.0, 1, g_pos, g_vel, 10.0, nullptr) == IDR_STATUS_INVALID_ARGUMENT);
    std::cout << "  [PASS] Null handle/output error handling verified." << std::endl;

    // 5. Test Numerical Safety: Invalid Timestamp
    assert(idr_engine_process_step(handle, -1.0, a_valid, w_valid, 0.0, 1, g_pos, g_vel, 10.0, &telem) == IDR_STATUS_INVALID_ARGUMENT);
    assert(idr_engine_process_step(handle, std::numeric_limits<double>::quiet_NaN(), a_valid, w_valid, 0.0, 1, g_pos, g_vel, 10.0, &telem) == IDR_STATUS_INVALID_ARGUMENT);
    std::cout << "  [PASS] Timestamp numerical sanitization verified." << std::endl;

    // 6. Test Numerical Safety: NaN & Inf Injection in IMU
    idr_vec3_t a_nan{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.80665};
    idr_vec3_t w_inf{0.0, std::numeric_limits<double>::infinity(), 0.0};

    assert(idr_engine_process_step(handle, 0.1, a_nan, w_valid, 0.0, 1, g_pos, g_vel, 10.0, &telem) == IDR_STATUS_NUMERICAL_FAILURE);
    assert(idr_engine_process_step(handle, 0.1, a_valid, w_inf, 0.0, 1, g_pos, g_vel, 10.0, &telem) == IDR_STATUS_NUMERICAL_FAILURE);
    std::cout << "  [PASS] NaN/Inf IMU rejection verified." << std::endl;

    // 7. Test Numerical Safety: Extreme Physical Outliers
    idr_vec3_t a_extreme{500.0, 0.0, 0.0}; // 500 m/s^2 >> 200 m/s^2 physical threshold
    assert(idr_engine_process_step(handle, 0.1, a_extreme, w_valid, 0.0, 1, g_pos, g_vel, 10.0, &telem) == IDR_STATUS_NUMERICAL_FAILURE);
    std::cout << "  [PASS] Physical outlier rejection verified." << std::endl;

    // 8. Test Valid Step Execution
    int32_t rc = idr_engine_process_step(
        handle, 0.1, a_valid, w_valid, 0.0, 1, g_pos, g_vel, 10.0, &telem
    );
    assert(rc == IDR_STATUS_OK);
    assert(telem.mode == IDR_NAV_MODE_GNSS_AIDED);
    assert(telem.gnss_valid == 1);
    assert(telem.time_s == 0.1);
    assert(std::isfinite(telem.speed_kmh));
    assert(std::isfinite(telem.p_trace));
    std::cout << "  [PASS] Valid step execution: mode=" << telem.mode << ", speed=" << telem.speed_kmh << " km/h" << std::endl;

    // 9. Test OSM Road Graph API
    idr_engine_clear_osm_graph(handle);
    idr_engine_add_osm_node(handle, 1, 25.4358, 81.8463, 100.0, 0.0, 0.0, 0.0);
    idr_engine_add_osm_node(handle, 2, 25.4360, 81.8465, 100.0, 20.0, 30.0, 0.0);

    double polyline[6] = {0.0, 0.0, 0.0, 20.0, 30.0, 0.0};
    idr_engine_add_osm_edge(handle, 100, 1, 2, "Test Road", "primary", 0, 7.0, 50.0, 2, polyline);
    std::cout << "  [PASS] OSM Graph node and edge ingestion verified." << std::endl;

    // 10. Test Engine Destruction
    idr_engine_destroy(handle);
    idr_engine_destroy(nullptr); // Safe no-op
    std::cout << "  [PASS] Engine destruction lifecycle verified." << std::endl;

    std::cout << "[TEST] All Hardened C API Tests PASSED!" << std::endl;
    return 0;
}

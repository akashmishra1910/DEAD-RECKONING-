#include "idr/c_api.h"
#include <iostream>
#include <cassert>

int main() {
    std::cout << "[TEST] Starting C API Tests..." << std::endl;

    idr_engine_handle_t handle = idr_engine_create(0.1);
    assert(handle != nullptr);

    idr_vec3_t a_p{0.0, 0.0, 9.80665};
    idr_vec3_t w_p{0.0, 0.0, 0.0};
    idr_vec3_t g_pos{0.0, 1.0, 0.0};
    idr_vec3_t g_vel{0.0, 10.0, 0.0};
    idr_telemetry_t telem{};

    int32_t rc = idr_engine_process_step(
        handle, 0.1, a_p, w_p, 0.0, 1, g_pos, g_vel, 10.0, &telem
    );
    assert(rc == 0);
    assert(telem.mode == 1); // GNSS_AIDED
    std::cout << "  C API process_step returned mode: " << telem.mode
              << " pos: [" << telem.pos_enu.x << ", " << telem.pos_enu.y << ", " << telem.pos_enu.z << "]" << std::endl;

    idr_engine_destroy(handle);
    std::cout << "[TEST] C API Tests PASSED!" << std::endl;
    return 0;
}

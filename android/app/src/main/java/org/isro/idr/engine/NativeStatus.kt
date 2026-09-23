package org.isro.idr.engine

/**
 * Status and error codes returned by the native IDR C++ navigation engine.
 * Maps directly to C API idr_status_t (idr/c_api.h).
 */
enum class NativeStatus(val code: Int, val description: String) {
    OK(0, "Success"),
    INVALID_ARGUMENT(-1, "Invalid argument, null handle, or out-of-range value"),
    INVALID_STATE(-2, "Engine in uninitialized or invalid state"),
    UNAVAILABLE(-3, "Required sensor or map data unavailable"),
    NUMERICAL_FAILURE(-4, "Numerical failure, NaN, Inf, or physical outlier"),
    BUFFER_ERROR(-5, "Buffer capacity error or truncation"),
    INIT_FAILURE(-6, "Engine initialization or memory allocation failed"),
    INTERNAL_ERROR(-7, "Unexpected internal engine exception"),
    UNKNOWN(-999, "Unknown native status code");

    val isSuccess: Boolean
        get() = this == OK

    companion object {
        fun fromCode(code: Int): NativeStatus =
            values().firstOrNull { it.code == code } ?: UNKNOWN
    }
}

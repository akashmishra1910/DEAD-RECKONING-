package org.isro.idr.engine

/**
 * 7-State Navigation Finite State Machine modes.
 * Maps directly to C API idr_nav_mode_t (idr/c_api.h).
 */
enum class NavigationMode(val code: Int, val displayName: String) {
    INITIALIZING(0, "Initializing (Alignment)"),
    GNSS_AIDED(1, "GNSS Fixed"),
    GNSS_DEGRADED(2, "GNSS Degraded"),
    TRANSITION_TO_DR(3, "Transitioning to DR"),
    DR_ONLY(4, "Dead Reckoning Active"),
    TRANSITION_TO_GNSS(5, "GNSS Reacquisition"),
    GNSS_REACQUIRED(6, "GNSS Restored"),
    UNKNOWN(-1, "Unknown State");

    val isDeadReckoning: Boolean
        get() = this == DR_ONLY || this == TRANSITION_TO_DR

    val isGnssHealthy: Boolean
        get() = this == GNSS_AIDED || this == GNSS_REACQUIRED

    companion object {
        fun fromCode(code: Int): NavigationMode =
            values().firstOrNull { it.code == code } ?: UNKNOWN
    }
}

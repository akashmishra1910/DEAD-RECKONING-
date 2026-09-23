package org.isro.idr.engine

/**
 * High-Rate Navigation Telemetry Output (Target: 10 Hz to 100 Hz).
 * Mapped from C ABI idr_telemetry_t without memory allocations.
 */
data class NavigationTelemetry(
    val mode: NavigationMode,
    val inOutage: Boolean,
    val timeSeconds: Double,
    val posEnu: Vector3D,
    val velEnu: Vector3D,
    val speedKmh: Double,
    val headingDeg: Double,
    val pitchDeg: Double,
    val rollDeg: Double,
    val accelBias: Vector3D,
    val gyroBias: Vector3D,
    val outageDurationSeconds: Double,
    val outageDistanceTraveledMeters: Double,
    val gnssValid: Boolean,
    val isPotholeActive: Boolean,
    val isIdling: Boolean,
    val pTrace: Double
) {
    companion object {
        const val FLAT_ARRAY_SIZE = 25

        fun fromFlatArray(arr: DoubleArray): NavigationTelemetry {
            require(arr.size >= FLAT_ARRAY_SIZE) {
                "Telemetry flat buffer requires at least $FLAT_ARRAY_SIZE elements, got ${arr.size}"
            }
            return NavigationTelemetry(
                mode = NavigationMode.fromCode(arr[0].toInt()),
                inOutage = arr[1].toInt() != 0,
                timeSeconds = arr[2],
                posEnu = Vector3D(arr[3], arr[4], arr[5]),
                velEnu = Vector3D(arr[6], arr[7], arr[8]),
                speedKmh = arr[9],
                headingDeg = arr[10],
                pitchDeg = arr[11],
                rollDeg = arr[12],
                accelBias = Vector3D(arr[13], arr[14], arr[15]),
                gyroBias = Vector3D(arr[16], arr[17], arr[18]),
                outageDurationSeconds = arr[19],
                outageDistanceTraveledMeters = arr[20],
                gnssValid = arr[21].toInt() != 0,
                isPotholeActive = arr[22].toInt() != 0,
                isIdling = arr[23].toInt() != 0,
                pTrace = arr[24]
            )
        }
    }
}

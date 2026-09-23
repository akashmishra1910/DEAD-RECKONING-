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
    val pTrace: Double,
    val mapMatchedPosEnu: Vector3D = posEnu,
    val selectedEdgeId: Int = -1,
    val selectedLayer: Int = 0,
    val mapConfidence: Int = 3, // 0=HIGH, 1=MEDIUM, 2=LOW, 3=INVALID
    val isMapAmbiguous: Boolean = false,
    val candidateCount: Int = 0,
    val hypothesisCount: Int = 0,
    val scoreMargin: Double = 0.0,
    val mapAvailable: Boolean = false
) {
    // Explicit Separation of Raw DR and Map-Matched Presentation State (Phase 11.8)
    val rawDrPosEnu: Vector3D get() = posEnu
    val posX: Double get() = posEnu.x
    val posY: Double get() = posEnu.y
    val posZ: Double get() = posEnu.z
    val timestampSec: Double get() = timeSeconds

    val isMapConfidenceHigh: Boolean get() = mapConfidence == 0
    val isMapConfidenceMedium: Boolean get() = mapConfidence == 1
    val isMapConfidenceLow: Boolean get() = mapConfidence == 2
    val isMapConfidenceValid: Boolean get() = mapConfidence in 0..2

    companion object {
        const val FLAT_ARRAY_SIZE = 35
        const val MIN_FLAT_ARRAY_SIZE = 25

        fun fromFlatArray(arr: DoubleArray): NavigationTelemetry {
            require(arr.size >= MIN_FLAT_ARRAY_SIZE) {
                "Telemetry flat buffer requires at least $MIN_FLAT_ARRAY_SIZE elements, got ${arr.size}"
            }
            val basePos = Vector3D(arr[3], arr[4], arr[5])
            val hasExt = arr.size >= FLAT_ARRAY_SIZE
            val mapPos = if (hasExt) Vector3D(arr[25], arr[26], arr[27]) else basePos
            val edgeId = if (hasExt) arr[28].toInt() else -1
            val layer = if (hasExt) arr[29].toInt() else 0
            val conf = if (hasExt) arr[30].toInt() else 3
            val ambig = if (hasExt) arr[31].toInt() != 0 else false
            val candCnt = if (hasExt) arr[32].toInt() else 0
            val hypCnt = if (hasExt) arr[33].toInt() else 0
            val margin = if (hasExt) arr[34] else 0.0
            val mapAvail = if (hasExt && arr.size > 35) arr[35].toInt() != 0 else (conf in 0..2)

            return NavigationTelemetry(
                mode = NavigationMode.fromCode(arr[0].toInt()),
                inOutage = arr[1].toInt() != 0,
                timeSeconds = arr[2],
                posEnu = basePos,
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
                pTrace = arr[24],
                mapMatchedPosEnu = mapPos,
                selectedEdgeId = edgeId,
                selectedLayer = layer,
                mapConfidence = conf,
                isMapAmbiguous = ambig,
                candidateCount = candCnt,
                hypothesisCount = hypCnt,
                scoreMargin = margin,
                mapAvailable = mapAvail
            )
        }
    }
}


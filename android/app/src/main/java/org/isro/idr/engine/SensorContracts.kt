package org.isro.idr.engine

/**
 * Explicit 3D Cartesian vector.
 */
data class Vector3D(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0
)

/**
 * IMU Sensor Input Sample (Target: ~100 Hz).
 * Coordinates: Phone sensor frame (m/s^2 for accel, rad/s for gyro).
 * Timestamp: Monotonic nanoseconds from SystemClock.elapsedRealtimeNanos().
 */
data class ImuSample(
    val timestampNanos: Long,
    val timeSeconds: Double,
    val accel: Vector3D,
    val gyro: Vector3D,
    val baroAltMeters: Double = 0.0
)

/**
 * Raw Platform GNSS Measurement Fix (Target: ~1 Hz).
 * Coordinates: Local Tangent ENU (meters) and WGS-84 Geodetic.
 * Note: Uses deterministic platform GNSS fixes, preserving measurement provenance.
 */
data class GnssSample(
    val timestampNanos: Long,
    val timeSeconds: Double,
    val isValid: Boolean,
    val posEnu: Vector3D,
    val velEnu: Vector3D,
    val latitudeDeg: Double = 0.0,
    val longitudeDeg: Double = 0.0,
    val altitudeM: Double = 0.0,
    val groundSpeedMps: Double = 0.0,
    val courseDeg: Double = 0.0,
    val horizontalAccuracyMeters: Float = 0.0f,
    val speedAccuracyMps: Float = 0.0f,
    val bearingAccuracyDeg: Float = 0.0f,
    val navicSatelliteCount: Int = 0,
    val totalSatelliteCount: Int = 0
)

/**
 * Magnetometer Input Sample (Target: ~20-50 Hz).
 * Units: microTesla (uT).
 * Monitored by the ferromagnetic disturbance rejection shield.
 */
data class MagnetometerSample(
    val timestampNanos: Long,
    val timeSeconds: Double,
    val isValid: Boolean,
    val magUteslas: Vector3D
)

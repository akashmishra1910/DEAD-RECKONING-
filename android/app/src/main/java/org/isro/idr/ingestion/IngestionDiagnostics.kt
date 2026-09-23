package org.isro.idr.ingestion

/**
 * Diagnostics snapshot representing live Android hardware ingestion health.
 * Exposes sensor rates, queue occupancy, drop counters, and GNSS status.
 */
data class IngestionDiagnostics(
    // Sensor Availability
    val isAccelerometerAvailable: Boolean = false,
    val isGyroscopeAvailable: Boolean = false,
    val isMagnetometerAvailable: Boolean = false,
    val isBarometerAvailable: Boolean = false,

    // IMU Measured Rates & Metrics
    val accelMeasuredHz: Double = 0.0,
    val gyroMeasuredHz: Double = 0.0,
    val accelSampleCount: Long = 0,
    val gyroSampleCount: Long = 0,
    val accelDroppedCount: Long = 0,
    val gyroDroppedCount: Long = 0,
    val accelLargeGapCount: Long = 0,
    val gyroLargeGapCount: Long = 0,
    val accelMeanDtMs: Double = 0.0,
    val gyroMeanDtMs: Double = 0.0,

    // GNSS Status
    val isGnssAvailable: Boolean = false,
    val gnssFixAgeMs: Long = -1L,
    val latitudeDeg: Double = 0.0,
    val longitudeDeg: Double = 0.0,
    val altitudeM: Double = 0.0,
    val horizontalAccuracyM: Float = 0.0f,
    val groundSpeedMps: Double = 0.0,
    val navicSatCount: Int = 0,
    val totalSatCount: Int = 0,

    // Buffer & Pipeline Health
    val queueOccupancy: Int = 0,
    val queueCapacity: Int = 500,
    val queueOverflowDrops: Long = 0,
    val totalStepsProcessed: Long = 0,
    val lastStepDurationMicros: Long = 0,
    val pipelineRunning: Boolean = false
)

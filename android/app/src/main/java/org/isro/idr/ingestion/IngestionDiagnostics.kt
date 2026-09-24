package org.isro.idr.ingestion

/**
 * Diagnostics snapshot representing live Android hardware ingestion health.
 * Exposes raw sensor callback rates, navigation consumption rates, queue occupancy,
 * drop counters, GNSS tracking, and native engine step latencies.
 */
data class IngestionDiagnostics(
    // Sensor Availability
    val isAccelerometerAvailable: Boolean = false,
    val isGyroscopeAvailable: Boolean = false,
    val isMagnetometerAvailable: Boolean = false,
    val isBarometerAvailable: Boolean = false,

    // IMU Callback Rates (Raw Hardware Delivery Frequency)
    val accelCallbackHz: Double = 0.0,
    val gyroCallbackHz: Double = 0.0,
    val accelCallbackCount: Long = 0,
    val gyroCallbackCount: Long = 0,

    // IMU Consumed Rates (Navigation Filter Step Frequency, Target ~100 Hz)
    val accelConsumedHz: Double = 0.0,
    val gyroConsumedHz: Double = 0.0,
    val accelConsumedCount: Long = 0,
    val gyroConsumedCount: Long = 0,

    // Backward-compatible aliases
    val accelMeasuredHz: Double = accelCallbackHz,
    val gyroMeasuredHz: Double = gyroCallbackHz,
    val accelSampleCount: Long = accelCallbackCount,
    val gyroSampleCount: Long = gyroCallbackCount,

    // Monotonic & Timestamp Anomalies
    val accelDroppedCount: Long = 0,
    val gyroDroppedCount: Long = 0,
    val accelLargeGapCount: Long = 0,
    val gyroLargeGapCount: Long = 0,
    val accelMeanDtMs: Double = 0.0,
    val gyroMeanDtMs: Double = 0.0,
    val accelMinDtMs: Double = 0.0,
    val accelMaxDtMs: Double = 0.0,
    val accelP95DtMs: Double = 0.0,

    // GNSS Status
    val isGnssAvailable: Boolean = false,
    val gnssFixState: String = "SEARCHING", // SEARCHING, FIXED, STALE, OUTAGE
    val gnssFixAgeMs: Long = -1L,
    val latitudeDeg: Double = 0.0,
    val longitudeDeg: Double = 0.0,
    val altitudeM: Double = 0.0,
    val horizontalAccuracyM: Float = 0.0f,
    val groundSpeedMps: Double = 0.0,
    val navicSatCount: Int = 0,
    val navicInViewCount: Int = 0,
    val totalSatCount: Int = 0,
    val totalInViewCount: Int = 0,

    // Buffer & Pipeline Health
    val queueOccupancy: Int = 0,
    val queueCapacity: Int = 500,
    val queueOverflowDrops: Long = 0,
    val totalStepsProcessed: Long = 0,
    val lastStepDurationMicros: Long = 0,
    val pipelineRunning: Boolean = false
)

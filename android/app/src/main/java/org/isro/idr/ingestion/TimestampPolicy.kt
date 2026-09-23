package org.isro.idr.ingestion

/**
 * High-precision monotonic timestamp policy and rate monitoring engine.
 * Ensures strict monotonicity and records sample delivery statistics.
 */
class TimestampPolicy(
    val sensorName: String,
    val nominalRateHz: Double = 100.0,
    val maxInterSampleGapNanos: Long = 100_000_000L // 100 ms max gap before anomaly alert
) {
    private val lock = Any()

    var sampleCount: Long = 0
        private set
    var firstTimestampNanos: Long = -1L
        private set
    var lastTimestampNanos: Long = -1L
        private set
    var duplicateTimestampCount: Long = 0
        private set
    var backwardsTimestampCount: Long = 0
        private set
    var largeGapCount: Long = 0
        private set
    var minDtNanos: Long = Long.MAX_VALUE
        private set
    var maxDtNanos: Long = 0L
        private set
    private var totalDtNanos: Long = 0L

    val measuredHz: Double
        get() = synchronized(lock) {
            if (sampleCount < 2 || firstTimestampNanos < 0 || lastTimestampNanos <= firstTimestampNanos) {
                0.0
            } else {
                val elapsedSec = (lastTimestampNanos - firstTimestampNanos).toDouble() / 1_000_000_000.0
                (sampleCount - 1).toDouble() / elapsedSec
            }
        }

    val meanDtMs: Double
        get() = synchronized(lock) {
            if (sampleCount < 2) 0.0
            else (totalDtNanos.toDouble() / (sampleCount - 1).toDouble()) / 1_000_000.0
        }

    /**
     * Validates a new incoming sensor monotonic timestamp in nanoseconds.
     * Returns true if valid and monotonic, false if duplicate or backwards.
     */
    fun ingestTimestamp(timestampNanos: Long): Boolean = synchronized(lock) {
        if (firstTimestampNanos < 0) {
            firstTimestampNanos = timestampNanos
            lastTimestampNanos = timestampNanos
            sampleCount = 1
            return true
        }

        val dt = timestampNanos - lastTimestampNanos
        if (dt == 0L) {
            duplicateTimestampCount++
            return false
        }
        if (dt < 0L) {
            backwardsTimestampCount++
            return false
        }

        if (dt > maxInterSampleGapNanos) {
            largeGapCount++
        }

        if (dt < minDtNanos) minDtNanos = dt
        if (dt > maxDtNanos) maxDtNanos = dt
        totalDtNanos += dt

        lastTimestampNanos = timestampNanos
        sampleCount++
        return true
    }

    /**
     * Converts a monotonic timestamp to relative engine seconds from first sample.
     */
    fun toEngineTimeSeconds(timestampNanos: Long, baseReferenceNanos: Long): Double {
        return (timestampNanos - baseReferenceNanos).toDouble() / 1_000_000_000.0
    }

    fun reset() = synchronized(lock) {
        sampleCount = 0
        firstTimestampNanos = -1L
        lastTimestampNanos = -1L
        duplicateTimestampCount = 0
        backwardsTimestampCount = 0
        largeGapCount = 0
        minDtNanos = Long.MAX_VALUE
        maxDtNanos = 0L
        totalDtNanos = 0L
    }
}

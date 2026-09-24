package org.isro.idr.ingestion

/**
 * High-precision monotonic timestamp policy and rate monitoring engine.
 * Distinguishes raw hardware callback frequency from navigation consumption frequency,
 * enforces strict monotonicity, and records inter-sample timing statistics.
 */
class TimestampPolicy(
    val sensorName: String,
    val nominalRateHz: Double = 100.0,
    val maxInterSampleGapNanos: Long = 100_000_000L // 100 ms max gap before anomaly alert
) {
    private val lock = Any()

    // Raw Callback Tracking (Hardware Interrupt Delivery)
    var callbackCount: Long = 0
        private set
    var firstCallbackTimestampNanos: Long = -1L
        private set
    var lastCallbackTimestampNanos: Long = -1L
        private set

    // Navigation Consumed Tracking (Forwarded to Estimator)
    var consumedCount: Long = 0
        private set
    var firstConsumedTimestampNanos: Long = -1L
        private set
    var lastConsumedTimestampNanos: Long = -1L
        private set

    // Anomaly Counters
    var duplicateTimestampCount: Long = 0
        private set
    var backwardsTimestampCount: Long = 0
        private set
    var largeGapCount: Long = 0
        private set

    // Inter-sample delta statistics (nanoseconds)
    var minDtNanos: Long = Long.MAX_VALUE
        private set
    var maxDtNanos: Long = 0L
        private set
    private var totalDtNanos: Long = 0L

    // Sliding circular buffer for P95 dt calculation (last 100 deltas)
    private val recentDts = LongArray(100)
    private var recentDtIndex = 0
    private var recentDtCount = 0

    // Backward-compatible aliases
    val sampleCount: Long
        get() = callbackCount
    val firstTimestampNanos: Long
        get() = firstCallbackTimestampNanos
    val lastTimestampNanos: Long
        get() = lastCallbackTimestampNanos

    val callbackHz: Double
        get() = synchronized(lock) {
            if (callbackCount < 2 || firstCallbackTimestampNanos < 0 || lastCallbackTimestampNanos <= firstCallbackTimestampNanos) {
                0.0
            } else {
                val elapsedSec = (lastCallbackTimestampNanos - firstCallbackTimestampNanos).toDouble() / 1_000_000_000.0
                (callbackCount - 1).toDouble() / elapsedSec
            }
        }

    val consumedHz: Double
        get() = synchronized(lock) {
            if (consumedCount < 2 || firstConsumedTimestampNanos < 0 || lastConsumedTimestampNanos <= firstConsumedTimestampNanos) {
                0.0
            } else {
                val elapsedSec = (lastConsumedTimestampNanos - firstConsumedTimestampNanos).toDouble() / 1_000_000_000.0
                (consumedCount - 1).toDouble() / elapsedSec
            }
        }

    val measuredHz: Double
        get() = callbackHz

    val meanDtMs: Double
        get() = synchronized(lock) {
            if (callbackCount < 2) 0.0
            else (totalDtNanos.toDouble() / (callbackCount - 1).toDouble()) / 1_000_000.0
        }

    val minDtMs: Double
        get() = synchronized(lock) {
            if (minDtNanos == Long.MAX_VALUE) 0.0 else minDtNanos / 1_000_000.0
        }

    val maxDtMs: Double
        get() = synchronized(lock) {
            maxDtNanos / 1_000_000.0
        }

    val p95DtMs: Double
        get() = synchronized(lock) {
            if (recentDtCount == 0) return@synchronized meanDtMs
            val copy = recentDts.copyOf(recentDtCount).apply { sort() }
            val p95Index = ((copy.size - 1) * 0.95).toInt().coerceIn(0, copy.size - 1)
            copy[p95Index] / 1_000_000.0
        }

    /**
     * Ingests a raw sensor callback timestamp.
     * Returns true if valid and monotonic, false if duplicate or backwards.
     */
    fun ingestCallback(timestampNanos: Long): Boolean = synchronized(lock) {
        if (firstCallbackTimestampNanos < 0) {
            firstCallbackTimestampNanos = timestampNanos
            lastCallbackTimestampNanos = timestampNanos
            callbackCount = 1
            return true
        }

        val dt = timestampNanos - lastCallbackTimestampNanos
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

        recentDts[recentDtIndex] = dt
        recentDtIndex = (recentDtIndex + 1) % recentDts.size
        if (recentDtCount < recentDts.size) recentDtCount++

        lastCallbackTimestampNanos = timestampNanos
        callbackCount++
        return true
    }

    /**
     * Backward-compatible alias for ingestCallback.
     */
    fun ingestTimestamp(timestampNanos: Long): Boolean = ingestCallback(timestampNanos)

    /**
     * Records that a sample at timestampNanos was forwarded and consumed by the navigation filter.
     */
    fun recordConsumed(timestampNanos: Long) = synchronized(lock) {
        if (firstConsumedTimestampNanos < 0) {
            firstConsumedTimestampNanos = timestampNanos
        }
        lastConsumedTimestampNanos = timestampNanos
        consumedCount++
    }

    /**
     * Converts a monotonic timestamp to relative engine seconds from base reference.
     */
    fun toEngineTimeSeconds(timestampNanos: Long, baseReferenceNanos: Long): Double {
        return (timestampNanos - baseReferenceNanos).toDouble() / 1_000_000_000.0
    }

    fun reset() = synchronized(lock) {
        callbackCount = 0
        firstCallbackTimestampNanos = -1L
        lastCallbackTimestampNanos = -1L
        consumedCount = 0
        firstConsumedTimestampNanos = -1L
        lastConsumedTimestampNanos = -1L
        duplicateTimestampCount = 0
        backwardsTimestampCount = 0
        largeGapCount = 0
        minDtNanos = Long.MAX_VALUE
        maxDtNanos = 0L
        totalDtNanos = 0L
        recentDtIndex = 0
        recentDtCount = 0
    }
}

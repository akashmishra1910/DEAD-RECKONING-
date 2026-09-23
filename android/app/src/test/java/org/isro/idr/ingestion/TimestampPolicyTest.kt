package org.isro.idr.ingestion

import org.junit.Assert.*
import org.junit.Test

class TimestampPolicyTest {

    @Test
    fun testMonotonicSequenceAndMeasuredRate() {
        val policy = TimestampPolicy("TestAccel", nominalRateHz = 100.0)

        // Feed 101 samples spaced exactly 10 ms (10,000,000 ns) apart -> 1.0 second total
        val startNanos = 1_000_000_000L
        val intervalNanos = 10_000_000L

        for (i in 0..100) {
            val t = startNanos + i * intervalNanos
            val accepted = policy.ingestTimestamp(t)
            assertTrue(accepted)
        }

        assertEquals(101L, policy.sampleCount)
        assertEquals(0L, policy.duplicateTimestampCount)
        assertEquals(0L, policy.backwardsTimestampCount)
        assertEquals(0L, policy.largeGapCount)
        assertEquals(100.0, policy.measuredHz, 0.1)
        assertEquals(10.0, policy.meanDtMs, 0.1)
    }

    @Test
    fun testDuplicateAndBackwardsTimestampsRejected() {
        val policy = TimestampPolicy("TestGyro")

        policy.ingestTimestamp(100_000_000L)
        policy.ingestTimestamp(110_000_000L)

        // Duplicate
        val dupAccepted = policy.ingestTimestamp(110_000_000L)
        assertFalse(dupAccepted)
        assertEquals(1L, policy.duplicateTimestampCount)

        // Backwards
        val backAccepted = policy.ingestTimestamp(105_000_000L)
        assertFalse(backAccepted)
        assertEquals(1L, policy.backwardsTimestampCount)

        // Valid next
        val validAccepted = policy.ingestTimestamp(120_000_000L)
        assertTrue(validAccepted)
        assertEquals(3L, policy.sampleCount)
    }

    @Test
    fun testLargeGapDetection() {
        val policy = TimestampPolicy("TestAccel", maxInterSampleGapNanos = 50_000_000L) // 50 ms max gap

        policy.ingestTimestamp(100_000_000L)
        policy.ingestTimestamp(110_000_000L) // 10 ms gap (OK)
        assertEquals(0L, policy.largeGapCount)

        policy.ingestTimestamp(200_000_000L) // 90 ms gap (> 50 ms)
        assertEquals(1L, policy.largeGapCount)
    }
}

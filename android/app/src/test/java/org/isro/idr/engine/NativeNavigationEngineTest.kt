package org.isro.idr.engine

import org.junit.Assert.*
import org.junit.Test

class NativeNavigationEngineTest {

    @Test
    fun testNativeStatusEnumMapping() {
        assertEquals(NativeStatus.OK, NativeStatus.fromCode(0))
        assertEquals(NativeStatus.INVALID_ARGUMENT, NativeStatus.fromCode(-1))
        assertEquals(NativeStatus.INVALID_STATE, NativeStatus.fromCode(-2))
        assertEquals(NativeStatus.UNAVAILABLE, NativeStatus.fromCode(-3))
        assertEquals(NativeStatus.NUMERICAL_FAILURE, NativeStatus.fromCode(-4))
        assertEquals(NativeStatus.BUFFER_ERROR, NativeStatus.fromCode(-5))
        assertEquals(NativeStatus.INIT_FAILURE, NativeStatus.fromCode(-6))
        assertEquals(NativeStatus.INTERNAL_ERROR, NativeStatus.fromCode(-7))
        assertEquals(NativeStatus.UNKNOWN, NativeStatus.fromCode(-999))
        assertTrue(NativeStatus.OK.isSuccess)
        assertFalse(NativeStatus.NUMERICAL_FAILURE.isSuccess)
    }

    @Test
    fun testNavigationModeEnumMapping() {
        assertEquals(NavigationMode.INITIALIZING, NavigationMode.fromCode(0))
        assertEquals(NavigationMode.GNSS_AIDED, NavigationMode.fromCode(1))
        assertEquals(NavigationMode.GNSS_DEGRADED, NavigationMode.fromCode(2))
        assertEquals(NavigationMode.TRANSITION_TO_DR, NavigationMode.fromCode(3))
        assertEquals(NavigationMode.DR_ONLY, NavigationMode.fromCode(4))
        assertEquals(NavigationMode.TRANSITION_TO_GNSS, NavigationMode.fromCode(5))
        assertEquals(NavigationMode.GNSS_REACQUIRED, NavigationMode.fromCode(6))
        assertEquals(NavigationMode.UNKNOWN, NavigationMode.fromCode(-1))

        assertTrue(NavigationMode.DR_ONLY.isDeadReckoning)
        assertTrue(NavigationMode.GNSS_AIDED.isGnssHealthy)
    }

    @Test
    fun testTelemetryDeserializationFromFlatArray() {
        val flatBuffer = DoubleArray(25) { 0.0 }
        flatBuffer[0] = 1.0  // GNSS_AIDED
        flatBuffer[1] = 0.0  // not in outage
        flatBuffer[2] = 12.5 // time_s
        flatBuffer[3] = 100.0 // pos_enu.x
        flatBuffer[4] = 200.0 // pos_enu.y
        flatBuffer[5] = 15.0  // pos_enu.z
        flatBuffer[6] = 5.0   // vel_enu.x
        flatBuffer[7] = 12.0  // vel_enu.y
        flatBuffer[8] = 0.0   // vel_enu.z
        flatBuffer[9] = 46.8  // speed_kmh
        flatBuffer[10] = 67.4 // heading_deg
        flatBuffer[11] = 1.2  // pitch_deg
        flatBuffer[12] = -0.5 // roll_deg
        flatBuffer[13] = 0.01 // accel_bias.x
        flatBuffer[14] = -0.02// accel_bias.y
        flatBuffer[15] = 0.03 // accel_bias.z
        flatBuffer[16] = 0.001// gyro_bias.x
        flatBuffer[17] = 0.002// gyro_bias.y
        flatBuffer[18] = -0.001// gyro_bias.z
        flatBuffer[19] = 0.0  // outage_duration_s
        flatBuffer[20] = 0.0  // outage_dist_m
        flatBuffer[21] = 1.0  // gnss_valid
        flatBuffer[22] = 0.0  // is_pothole_active
        flatBuffer[23] = 0.0  // is_idling
        flatBuffer[24] = 0.045// p_trace

        val telem = NavigationTelemetry.fromFlatArray(flatBuffer)

        assertEquals(NavigationMode.GNSS_AIDED, telem.mode)
        assertFalse(telem.inOutage)
        assertEquals(12.5, telem.timeSeconds, 1e-6)
        assertEquals(100.0, telem.posEnu.x, 1e-6)
        assertEquals(200.0, telem.posEnu.y, 1e-6)
        assertEquals(15.0, telem.posEnu.z, 1e-6)
        assertEquals(46.8, telem.speedKmh, 1e-6)
        assertEquals(67.4, telem.headingDeg, 1e-6)
        assertTrue(telem.gnssValid)
        assertFalse(telem.isPotholeActive)
        assertEquals(0.045, telem.pTrace, 1e-6)
    }

    @Test(expected = IllegalArgumentException::class)
    fun testTelemetryShortBufferThrows() {
        val shortBuffer = DoubleArray(10)
        NavigationTelemetry.fromFlatArray(shortBuffer)
    }
}

package org.isro.idr.ingestion

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class GeoCoordinatesTest {

    @Test
    fun testAnchorOriginProducesZeroEnu() {
        val anchorLat = 25.4358
        val anchorLon = 81.8463
        val anchorAlt = 100.0

        val geo = GeoCoordinates(anchorLat, anchorLon, anchorAlt)
        val enu = geo.geodeticToEnu(anchorLat, anchorLon, anchorAlt)

        assertEquals(0.0, enu.x, 1e-6)
        assertEquals(0.0, enu.y, 1e-6)
        assertEquals(0.0, enu.z, 1e-6)
    }

    @Test
    fun testNorthDisplacement() {
        // ~1 degree latitude is approximately 111.139 km
        val anchorLat = 0.0
        val anchorLon = 0.0
        val anchorAlt = 0.0

        val geo = GeoCoordinates(anchorLat, anchorLon, anchorAlt)
        val enu = geo.geodeticToEnu(0.001, 0.0, 0.0) // ~111 meters North

        assertEquals(0.0, enu.x, 0.1)
        assertTrue(enu.y > 110.0 && enu.y < 112.0)
    }

    @Test
    fun testEastDisplacement() {
        val anchorLat = 0.0
        val anchorLon = 0.0
        val anchorAlt = 0.0

        val geo = GeoCoordinates(anchorLat, anchorLon, anchorAlt)
        val enu = geo.geodeticToEnu(0.0, 0.001, 0.0) // ~111 meters East

        assertTrue(enu.x > 110.0 && enu.x < 112.0)
        assertEquals(0.0, enu.y, 0.1)
    }

    @Test
    fun testVelocityToEnu() {
        val geo = GeoCoordinates(25.4358, 81.8463, 100.0)

        // Pure North (0 deg) at 10 m/s
        val velNorth = geo.velocityToEnu(10.0, 0.0, 0.0)
        assertEquals(0.0, velNorth.x, 1e-6)
        assertEquals(10.0, velNorth.y, 1e-6)
        assertEquals(0.0, velNorth.z, 1e-6)

        // Pure East (90 deg) at 10 m/s
        val velEast = geo.velocityToEnu(10.0, 90.0, 0.0)
        assertEquals(10.0, velEast.x, 1e-6)
        assertEquals(0.0, velEast.y, 1e-6)

        // North-East (45 deg) at 10 m/s
        val velNE = geo.velocityToEnu(10.0, 45.0, 0.0)
        val expected = 10.0 / sqrt(2.0)
        assertEquals(expected, velNE.x, 1e-6)
        assertEquals(expected, velNE.y, 1e-6)
    }
}

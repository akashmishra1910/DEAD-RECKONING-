package org.isro.idr.ingestion

import org.isro.idr.engine.Vector3D
import kotlin.math.*

/**
 * High-precision WGS-84 Geodetic to Local Tangent East-North-Up (ENU) Cartesian Projector.
 * Strictly offline, closed-form geodesy without network dependencies.
 */
class GeoCoordinates(
    val anchorLatDeg: Double,
    val anchorLonDeg: Double,
    val anchorAltM: Double
) {
    companion object {
        // WGS-84 standard ellipsoid constants
        const val WGS84_A = 6378137.0                // Semi-major axis (meters)
        const val WGS84_F = 1.0 / 298.257223563       // Flattening
        const val WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F // First eccentricity squared (~0.00669437999014)

        /**
         * Converts geodetic coordinates (lat, lon in degrees, alt in meters) to ECEF (meters).
         */
        fun geodeticToEcef(latDeg: Double, lonDeg: Double, altM: Double): Vector3D {
            val phi = Math.toRadians(latDeg)
            val lambda = Math.toRadians(lonDeg)
            val sinPhi = sin(phi)
            val cosPhi = cos(phi)
            val sinLambda = sin(lambda)
            val cosLambda = cos(lambda)

            val n = WGS84_A / sqrt(1.0 - WGS84_E2 * sinPhi * sinPhi)
            val x = (n + altM) * cosPhi * cosLambda
            val y = (n + altM) * cosPhi * sinLambda
            val z = (n * (1.0 - WGS84_E2) + altM) * sinPhi

            return Vector3D(x, y, z)
        }
    }

    private val anchorPhi = Math.toRadians(anchorLatDeg)
    private val anchorLambda = Math.toRadians(anchorLonDeg)
    private val sinPhi0 = sin(anchorPhi)
    private val cosPhi0 = cos(anchorPhi)
    private val sinLambda0 = sin(anchorLambda)
    private val cosLambda0 = cos(anchorLambda)
    private val anchorEcef = geodeticToEcef(anchorLatDeg, anchorLonDeg, anchorAltM)

    /**
     * Converts a WGS-84 geodetic point to local tangent ENU coordinates relative to the anchor origin.
     */
    fun geodeticToEnu(latDeg: Double, lonDeg: Double, altM: Double): Vector3D {
        val pEcef = geodeticToEcef(latDeg, lonDeg, altM)
        val dx = pEcef.x - anchorEcef.x
        val dy = pEcef.y - anchorEcef.y
        val dz = pEcef.z - anchorEcef.z

        val east  = -sinLambda0 * dx + cosLambda0 * dy
        val north = -sinPhi0 * cosLambda0 * dx - sinPhi0 * sinLambda0 * dy + cosPhi0 * dz
        val up    =  cosPhi0 * cosLambda0 * dx + cosPhi0 * sinLambda0 * dy + sinPhi0 * dz

        return Vector3D(east, north, up)
    }

    /**
     * Converts horizontal ground speed (m/s) and track bearing (deg clockwise from North)
     * to East-North-Up velocity vector.
     */
    fun velocityToEnu(groundSpeedMps: Double, courseDeg: Double, climbRateMps: Double = 0.0): Vector3D {
        val courseRad = Math.toRadians(courseDeg)
        val east = groundSpeedMps * sin(courseRad)
        val north = groundSpeedMps * cos(courseRad)
        return Vector3D(east, north, climbRateMps)
    }
}

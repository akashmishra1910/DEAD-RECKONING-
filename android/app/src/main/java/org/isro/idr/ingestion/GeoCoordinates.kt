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
    data class Wgs84Coordinate(
        val latitude: Double,
        val longitude: Double,
        val altitude: Double = 0.0
    )

    companion object {
        // WGS-84 standard ellipsoid constants
        const val WGS84_A = 6378137.0                // Semi-major axis (meters)
        const val WGS84_B = 6356752.314245           // Semi-minor axis (meters)
        const val WGS84_F = 1.0 / 298.257223563       // Flattening
        const val WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F // First eccentricity squared (~0.00669437999014)
        const val WGS84_E_PRIME2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B)

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

        /**
         * Converts ECEF coordinates (meters) to geodetic WGS-84 (lat, lon in degrees, alt in meters)
         * using Bowring's closed-form algorithm.
         */
        fun ecefToGeodetic(ecef: Vector3D): Wgs84Coordinate {
            val p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y)
            if (p < 1e-6) {
                val lat = if (ecef.z > 0) 90.0 else -90.0
                return Wgs84Coordinate(lat, 0.0, abs(ecef.z) - WGS84_B)
            }

            val theta = atan2(ecef.z * WGS84_A, p * WGS84_B)
            val sinTheta = sin(theta)
            val cosTheta = cos(theta)

            val phi = atan2(
                ecef.z + WGS84_E_PRIME2 * WGS84_B * sinTheta * sinTheta * sinTheta,
                p - WGS84_E2 * WGS84_A * cosTheta * cosTheta * cosTheta
            )
            val lambda = atan2(ecef.y, ecef.x)

            val sinPhi = sin(phi)
            val n = WGS84_A / sqrt(1.0 - WGS84_E2 * sinPhi * sinPhi)
            val alt = p / cos(phi) - n

            return Wgs84Coordinate(
                Math.toDegrees(phi),
                Math.toDegrees(lambda),
                alt
            )
        }

        /**
         * Converts local ENU coordinates to geodetic WGS-84 given an anchor.
         */
        fun enuToWgs84(east: Double, north: Double, up: Double, anchor: Wgs84Coordinate): Wgs84Coordinate {
            val phi0 = Math.toRadians(anchor.latitude)
            val lam0 = Math.toRadians(anchor.longitude)
            val sinPhi0 = sin(phi0)
            val cosPhi0 = cos(phi0)
            val sinLam0 = sin(lam0)
            val cosLam0 = cos(lam0)

            val anchorEcef = geodeticToEcef(anchor.latitude, anchor.longitude, anchor.altitude)

            val dx = -sinLam0 * east - sinPhi0 * cosLam0 * north + cosPhi0 * cosLam0 * up
            val dy =  cosLam0 * east - sinPhi0 * sinLam0 * north + cosPhi0 * sinLam0 * up
            val dz =  cosPhi0 * north + sinPhi0 * up

            val pEcef = Vector3D(anchorEcef.x + dx, anchorEcef.y + dy, anchorEcef.z + dz)
            return ecefToGeodetic(pEcef)
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
     * Converts local ENU coordinates to geodetic WGS-84 relative to this anchor instance.
     */
    fun enuToGeodetic(east: Double, north: Double, up: Double): Wgs84Coordinate {
        val dx = -sinLambda0 * east - sinPhi0 * cosLambda0 * north + cosPhi0 * cosLambda0 * up
        val dy =  cosLambda0 * east - sinPhi0 * sinLambda0 * north + cosPhi0 * sinLambda0 * up
        val dz =  cosPhi0 * north + sinPhi0 * up

        val pEcef = Vector3D(anchorEcef.x + dx, anchorEcef.y + dy, anchorEcef.z + dz)
        return ecefToGeodetic(pEcef)
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

package org.isro.idr.ingestion

import android.annotation.SuppressLint
import android.content.Context
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import org.isro.idr.engine.GnssSample
import org.isro.idr.engine.Vector3D

/**
 * Android GNSS / Location Ingestion Manager.
 * Preserves platform GNSS measurement provenance, tracks NavIC/IRNSS satellites,
 * and projects geodetic fixes into local tangent ENU coordinates.
 */
class GnssIngestionManager(
    private val context: Context,
    val minUpdateIntervalMs: Long = 1000L, // 1 Hz target
    val minUpdateDistanceM: Float = 0.0f
) : LocationListener {

    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as? LocationManager
    private var gnssStatusCallback: GnssStatus.Callback? = null

    @Volatile var isRunning: Boolean = false
        private set

    @Volatile var latestGnssSample: GnssSample? = null
        private set

    @Volatile var anchorGeo: GeoCoordinates? = null
        private set

    @Volatile var navicSatCount: Int = 0
        private set

    @Volatile var totalSatCount: Int = 0
        private set

    var onGnssSampleListener: ((GnssSample) -> Unit)? = null

    val isGnssAvailable: Boolean
        get() = latestGnssSample?.let {
            it.isValid && (SystemClock.elapsedRealtime() - (it.timestampNanos / 1_000_000L) < 2000L)
        } ?: false

    @SuppressLint("MissingPermission")
    fun start(): Boolean {
        if (isRunning || locationManager == null) return false

        try {
            locationManager.requestLocationUpdates(
                LocationManager.GPS_PROVIDER,
                minUpdateIntervalMs,
                minUpdateDistanceM,
                this
            )

            // Register GnssStatus callback for satellite tracking and NavIC detection
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                gnssStatusCallback = object : GnssStatus.Callback() {
                    override fun onSatelliteStatusChanged(status: GnssStatus) {
                        var total = 0
                        var navic = 0
                        val count = status.satelliteCount
                        for (i in 0 until count) {
                            if (status.usedInFix(i)) {
                                total++
                                if (status.getConstellationType(i) == GnssStatus.CONSTELLATION_IRNSS) {
                                    navic++
                                }
                            }
                        }
                        totalSatCount = total
                        navicSatCount = navic
                    }
                }
                locationManager.registerGnssStatusCallback(gnssStatusCallback!!, null)
            }

            isRunning = true
            return true
        } catch (e: SecurityException) {
            isRunning = false
            return false
        }
    }

    fun stop() {
        if (!isRunning) return
        locationManager?.removeUpdates(this)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && gnssStatusCallback != null) {
            locationManager?.unregisterGnssStatusCallback(gnssStatusCallback!!)
            gnssStatusCallback = null
        }
        isRunning = false
    }

    override fun onLocationChanged(location: Location) {
        if (!isRunning) return

        val timestampNanos = location.elapsedRealtimeNanos
        val lat = location.latitude
        val lon = location.longitude
        val alt = if (location.hasAltitude()) location.altitude else 0.0
        val speed = if (location.hasSpeed()) location.speed.toDouble() else 0.0
        val bearing = if (location.hasBearing()) location.bearing.toDouble() else 0.0
        val accuracy = if (location.hasAccuracy()) location.accuracy else 999.0f
        val speedAcc = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && location.hasSpeedAccuracy()) {
            location.speedAccuracyMetersPerSecond
        } else 0.0f
        val bearingAcc = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && location.hasBearingAccuracy()) {
            location.bearingAccuracyDegrees
        } else 0.0f

        // Validation gate
        val isValid = (accuracy <= 25.0f) && (lat >= -90.0 && lat <= 90.0) &&
                      (lon >= -180.0 && lon <= 180.0) && (speed >= 0.0 && speed <= 150.0)

        // Establish or query local tangent ENU anchor
        var anchor = anchorGeo
        if (anchor == null && isValid) {
            anchor = GeoCoordinates(lat, lon, alt)
            anchorGeo = anchor
        }

        val posEnu = anchor?.geodeticToEnu(lat, lon, alt) ?: Vector3D(0.0, 0.0, 0.0)
        val velEnu = anchor?.velocityToEnu(speed, bearing, 0.0) ?: Vector3D(0.0, 0.0, 0.0)

        val sample = GnssSample(
            timestampNanos = timestampNanos,
            timeSeconds = timestampNanos.toDouble() / 1_000_000_000.0,
            isValid = isValid,
            posEnu = posEnu,
            velEnu = velEnu,
            latitudeDeg = lat,
            longitudeDeg = lon,
            altitudeM = alt,
            groundSpeedMps = speed,
            courseDeg = bearing,
            horizontalAccuracyMeters = accuracy,
            speedAccuracyMps = speedAcc,
            bearingAccuracyDeg = bearingAcc,
            navicSatelliteCount = navicSatCount,
            totalSatelliteCount = totalSatCount
        )

        latestGnssSample = sample
        onGnssSampleListener?.invoke(sample)
    }

    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}
    override fun onProviderEnabled(provider: String) {}
    override fun onProviderDisabled(provider: String) {
        latestGnssSample = null
    }

    fun setExplicitAnchor(latDeg: Double, lonDeg: Double, altM: Double) {
        anchorGeo = GeoCoordinates(latDeg, lonDeg, altM)
    }
}

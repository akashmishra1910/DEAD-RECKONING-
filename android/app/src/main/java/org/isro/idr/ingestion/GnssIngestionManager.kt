package org.isro.idr.ingestion

import android.annotation.SuppressLint
import android.content.Context
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import org.isro.idr.engine.GnssSample
import org.isro.idr.engine.Vector3D

/**
 * Android GNSS / Location Ingestion Manager.
 * Preserves platform GNSS measurement provenance, tracks NavIC/IRNSS satellites,
 * coordinates geodetic fixes, and projects geodetic fixes into local tangent ENU coordinates.
 */
class GnssIngestionManager(
    private val context: Context,
    val minUpdateIntervalMs: Long = 1000L, // 1 Hz target
    val minUpdateDistanceM: Float = 0.0f
) : LocationListener {

    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as? LocationManager
    private var gnssStatusCallback: GnssStatus.Callback? = null
    private val mainHandler = Handler(Looper.getMainLooper())

    @Volatile var isRunning: Boolean = false
        private set

    @Volatile var isPermissionGranted: Boolean = false

    @Volatile var latestGnssSample: GnssSample? = null
        private set

    @Volatile var anchorGeo: GeoCoordinates? = null
        private set

    @Volatile var navicSatCount: Int = 0
        private set
    @Volatile var navicInViewCount: Int = 0
        private set

    @Volatile var totalSatCount: Int = 0
        private set
    @Volatile var totalInViewCount: Int = 0
        private set

    var onGnssSampleListener: ((GnssSample) -> Unit)? = null
    var onSatelliteStatusListener: ((List<org.isro.idr.ui.state.SatelliteInfo>) -> Unit)? = null

    val isGnssAvailable: Boolean
        get() = latestGnssSample?.let {
            it.isValid && (SystemClock.elapsedRealtime() - (it.timestampNanos / 1_000_000L) < 2000L)
        } ?: false

    val isSearchingFix: Boolean
        get() = isRunning && !isGnssAvailable

    @SuppressLint("MissingPermission")
    fun start(): Boolean {
        if (isRunning || locationManager == null) return false

        try {
            isPermissionGranted = true

            // 1. Request updates from GPS Provider on Main Looper
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    minUpdateIntervalMs,
                    minUpdateDistanceM,
                    this,
                    Looper.getMainLooper()
                )
            }

            // 2. Fallback request from Network Provider for coarse fix while GPS locks
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                try {
                    locationManager.requestLocationUpdates(
                        LocationManager.NETWORK_PROVIDER,
                        minUpdateIntervalMs,
                        minUpdateDistanceM,
                        this,
                        Looper.getMainLooper()
                    )
                } catch (_: Exception) {}
            }

            // 3. Register GnssStatus callback for satellite tracking and NavIC / IRNSS detection
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                gnssStatusCallback = object : GnssStatus.Callback() {
                    override fun onSatelliteStatusChanged(status: GnssStatus) {
                        var totalUsed = 0
                        var navicUsed = 0
                        var totalInView = 0
                        var navicInView = 0
                        val satList = mutableListOf<org.isro.idr.ui.state.SatelliteInfo>()

                        val count = status.satelliteCount
                        for (i in 0 until count) {
                            totalInView++
                            val constTypeInt = status.getConstellationType(i)
                            val constType = when (constTypeInt) {
                                GnssStatus.CONSTELLATION_GPS -> org.isro.idr.ui.state.ConstellationType.GPS
                                GnssStatus.CONSTELLATION_SBAS -> org.isro.idr.ui.state.ConstellationType.GPS
                                GnssStatus.CONSTELLATION_GLONASS -> org.isro.idr.ui.state.ConstellationType.GLONASS
                                GnssStatus.CONSTELLATION_QZSS -> org.isro.idr.ui.state.ConstellationType.GPS
                                GnssStatus.CONSTELLATION_BEIDOU -> org.isro.idr.ui.state.ConstellationType.BEIDOU
                                GnssStatus.CONSTELLATION_GALILEO -> org.isro.idr.ui.state.ConstellationType.GALILEO
                                GnssStatus.CONSTELLATION_IRNSS -> org.isro.idr.ui.state.ConstellationType.NAVIC
                                else -> if (constTypeInt == 7) org.isro.idr.ui.state.ConstellationType.NAVIC else org.isro.idr.ui.state.ConstellationType.GPS
                            }
                            val isNavic = (constType == org.isro.idr.ui.state.ConstellationType.NAVIC)
                            if (isNavic) {
                                navicInView++
                            }
                            val inFix = status.usedInFix(i)
                            if (inFix) {
                                totalUsed++
                                if (isNavic) {
                                    navicUsed++
                                }
                            }
                            satList.add(
                                org.isro.idr.ui.state.SatelliteInfo(
                                    svid = status.getSvid(i),
                                    constellation = constType,
                                    elevationDeg = status.getElevationDegrees(i),
                                    azimuthDeg = status.getAzimuthDegrees(i),
                                    snrDbHz = status.getCn0DbHz(i),
                                    usedInFix = inFix
                                )
                            )
                        }
                        totalSatCount = totalUsed
                        totalInViewCount = totalInView
                        navicSatCount = navicUsed
                        navicInViewCount = navicInView
                        android.util.Log.d("IDR_GNSS", "GnssStatus: TotalInView=$totalInView, Used=$totalUsed, NavIC_InView=$navicInView, NavIC_Used=$navicUsed")
                        onSatelliteStatusListener?.invoke(satList)
                    }
                }
                locationManager.registerGnssStatusCallback(gnssStatusCallback!!, mainHandler)
            }

            // 4. Poll last known location for instant anchoring
            val lastGps = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER)
            val lastNet = locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER)
            val bestLast = when {
                lastGps != null && lastNet != null -> if (lastGps.time >= lastNet.time) lastGps else lastNet
                lastGps != null -> lastGps
                else -> lastNet
            }
            if (bestLast != null && latestGnssSample == null) {
                onLocationChanged(bestLast)
            }

            isRunning = true
            return true
        } catch (e: SecurityException) {
            isPermissionGranted = false
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
        val isValid = (accuracy <= 40.0f) && (lat in -90.0..90.0) &&
                      (lon in -180.0..180.0) && (speed in 0.0..150.0)

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

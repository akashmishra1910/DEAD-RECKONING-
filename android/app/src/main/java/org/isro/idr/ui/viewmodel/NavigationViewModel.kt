package org.isro.idr.ui.viewmodel

import android.content.Context
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.isro.idr.engine.GnssSample
import org.isro.idr.engine.NativeNavigationEngine
import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GeoCoordinates
import org.isro.idr.ingestion.GnssIngestionManager
import org.isro.idr.ingestion.IngestionDiagnostics
import org.isro.idr.ingestion.NavigationPipelineSynchronizer
import org.isro.idr.map.IdrMapBinaryReader
import org.isro.idr.map.RoadMapGraph
import org.isro.idr.ui.state.AppFlowState
import org.isro.idr.ui.state.GnssStatusState
import org.isro.idr.ui.state.HealthStatus
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.MapStyleType
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.ScreenTab
import org.isro.idr.ui.state.SensorPlotTab
import org.isro.idr.ui.state.SubScreen
import org.isro.idr.ui.state.SystemHealthState
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint
import java.io.File
import java.io.FileOutputStream

/**
 * Production ViewModel orchestrating UI state for the 12-Screen ISRO IDR Navigation Product.
 * Decouples high-rate native estimator execution (~100 Hz) from UI recomposition (~10 Hz).
 * Manages global offline vector datasets, sensor telemetry, and Track-B inference state.
 */
class NavigationViewModel(
    private val synchronizer: NavigationPipelineSynchronizer? = null,
    private val gnssManager: GnssIngestionManager? = null,
    private val engine: NativeNavigationEngine? = null
) : ViewModel() {

    private val _uiState = MutableStateFlow(NavigationUiState())
    val uiState: StateFlow<NavigationUiState> = _uiState.asStateFlow()

    private val maxTrajectoryPoints = 1500
    private val maxWaveformSamples = 100
    private var lastUiUpdateTimestampNanos = 0L
    private val minUiUpdateIntervalNanos = 100_000_000L // 100ms = 10 Hz UI rate

    private var outageStartSec: Double? = null

    private val idrPoints = ArrayDeque<TrajectoryPoint>(maxTrajectoryPoints)
    private val gnssPoints = ArrayDeque<TrajectoryPoint>(maxTrajectoryPoints)
    private val mapAidedPoints = ArrayDeque<TrajectoryPoint>(maxTrajectoryPoints)

    private val accelWaveforms = ArrayDeque<org.isro.idr.ui.state.SensorWaveformSample>(maxWaveformSamples)
    private val gyroWaveforms = ArrayDeque<org.isro.idr.ui.state.SensorWaveformSample>(maxWaveformSamples)
    private val magWaveforms = ArrayDeque<org.isro.idr.ui.state.SensorWaveformSample>(maxWaveformSamples)
    private val baroWaveforms = ArrayDeque<org.isro.idr.ui.state.SensorWaveformSample>(maxWaveformSamples)

    private var tripStartTimestampMs: Long = System.currentTimeMillis()
    private var tripDistanceMetersTotal: Double = 0.0
    private var lastEnuX: Double? = null
    private var lastEnuY: Double? = null
    private var tripMaxSpeedKmhVal: Double = 0.0
    private var prevNavMode: NavigationMode? = null
    private val tripEventsList = mutableListOf<org.isro.idr.ui.state.TripEvent>()
    private val tripDriftList = mutableListOf<org.isro.idr.ui.state.DriftPoint>()

    private var rawRoadGraph: RoadMapGraph? = null

    init {
        synchronizer?.onTelemetryListener = { telemetry ->
            onTelemetryReceived(telemetry)
        }
        synchronizer?.onImuSampleListener = { imu ->
            onImuSample(imu)
        }
        synchronizer?.onMagSampleListener = { mag ->
            onMagSample(mag)
        }
        gnssManager?.onGnssSampleListener = { sample ->
            onGnssSample(sample)
        }
        gnssManager?.onSatelliteStatusListener = { satellites ->
            onSatelliteStatus(satellites)
        }

        // Periodic 10 Hz Diagnostics Ticker
        viewModelScope.launch {
            while (true) {
                delay(100)
                try {
                    synchronizer?.let { sync ->
                        val diag = sync.getDiagnosticsSnapshot()
                        updateDiagnostics(diag)
                    }
                } catch (_: Exception) {}
            }
        }
    }

    /**
     * App Flow Handlers: Splash -> Calibration -> Navigation.
     */
    fun startCalibrationFlow() {
        _uiState.update { it.copy(appFlow = AppFlowState.CALIBRATION, calibrationProgress = 0.25f) }
        viewModelScope.launch {
            delay(800)
            _uiState.update { it.copy(calibrationProgress = 0.65f, isAccelCalibrated = true, isGyroCalibrated = true) }
            delay(800)
            _uiState.update { it.copy(calibrationProgress = 1.0f, isMagCalibrated = true, isBaroCalibrated = true) }
        }
    }

    fun completeCalibration() {
        _uiState.update { it.copy(appFlow = AppFlowState.MAIN_APP, selectedScreen = ScreenTab.NAVIGATION) }
    }

    fun openSubScreen(sub: SubScreen) {
        _uiState.update { it.copy(activeSubScreen = sub) }
    }

    fun closeSubScreen() {
        _uiState.update { it.copy(activeSubScreen = SubScreen.NONE) }
    }

    /**
     * Loads an offline .idrmap dataset from assets into memory and the C++ native engine.
     */
    fun loadOfflineMap(context: Context, fileName: String = "prayagraj_corridor.idrmap") {
        viewModelScope.launch(Dispatchers.IO) {
            _uiState.update { it.copy(mapDataState = MapDataState.LOADING) }
            try {
                // 1. Try to load companion geographic vector basemap if available
                val basemapJsonName = when {
                    fileName.contains("prayagraj") -> "prayagraj_basemap.json"
                    fileName.contains("dense_metro") -> "dense_metro_basemap.json"
                    fileName.contains("regional") -> "regional_network_basemap.json"
                    else -> null
                }
                val companionBasemap = basemapJsonName?.let { bName ->
                    try {
                        context.assets.open("maps/$bName").use { bStream ->
                            IdrMapBinaryReader.readBasemapFromJsonStream(bStream)
                        }
                    } catch (_: Exception) { null }
                }

                // 2. Read binary road graph from assets
                val assetPath = "maps/$fileName"
                val inputStream = context.assets.open(assetPath)
                val graph = IdrMapBinaryReader.readFromStream(
                    stream = inputStream,
                    mapName = fileName.removeSuffix(".idrmap"),
                    basemap = companionBasemap
                )

                if (graph != null) {
                    rawRoadGraph = graph

                    // 3. Extract asset to internal cache for C++ IdrMapReader mmap access
                    val cacheFile = File(context.cacheDir, fileName)
                    context.assets.open(assetPath).use { input ->
                        FileOutputStream(cacheFile).use { output ->
                            input.copyTo(output)
                        }
                    }

                    // 4. Determine anchor (use current GNSS anchor or map center)
                    val currentAnchor = gnssManager?.anchorGeo ?: GeoCoordinates(graph.centerLat, graph.centerLon, 0.0)
                    if (gnssManager?.anchorGeo == null) {
                        gnssManager?.setExplicitAnchor(graph.centerLat, graph.centerLon, 0.0)
                    }

                    // 5. Project roads and basemap to ENU
                    val projectedGraph = graph.withEnuProjection(currentAnchor)

                    // 6. Load into C++ native engine
                    engine?.loadIdrmap(
                        path = cacheFile.absolutePath,
                        lat0 = currentAnchor.anchorLatDeg,
                        lon0 = currentAnchor.anchorLonDeg,
                        alt0 = currentAnchor.anchorAltM
                    )

                    withContext(Dispatchers.Main) {
                        _uiState.update { current ->
                            current.copy(
                                roadMapGraph = projectedGraph,
                                selectedMapFileName = fileName,
                                mapDataState = MapDataState.AVAILABLE,
                                enuAnchor = GeoCoordinates.Wgs84Coordinate(
                                    currentAnchor.anchorLatDeg,
                                    currentAnchor.anchorLonDeg,
                                    currentAnchor.anchorAltM
                                )
                            )
                        }
                    }
                } else {
                    withContext(Dispatchers.Main) {
                        _uiState.update { it.copy(mapDataState = MapDataState.OFFLINE) }
                    }
                }
            } catch (e: Exception) {
                withContext(Dispatchers.Main) {
                    _uiState.update { it.copy(mapDataState = MapDataState.OFFLINE) }
                }
            }
        }
    }

    /**
     * Consumes native telemetry at ~100 Hz, updates trajectories, and throttles UI state to ~10 Hz.
     */
    fun onTelemetryReceived(telemetry: NavigationTelemetry) {
        val nowNanos = System.nanoTime()

        val currentEnuX = telemetry.posX
        val currentEnuY = telemetry.posY
        val currentEnuZ = telemetry.posZ
        val currentSec = telemetry.timestampSec

        // Accumulate IDR trajectory point
        synchronized(idrPoints) {
            val last = idrPoints.lastOrNull()
            if (last == null || distanceSq(last.enuX, last.enuY, currentEnuX, currentEnuY) > 0.16) {
                if (idrPoints.size >= maxTrajectoryPoints) {
                    idrPoints.removeFirst()
                }
                idrPoints.addLast(
                    TrajectoryPoint(
                        enuX = currentEnuX,
                        enuY = currentEnuY,
                        enuZ = currentEnuZ,
                        timestampSec = currentSec,
                        isGnss = false
                    )
                )
            }
        }

        // Accumulate Map-Aided trajectory point if confidence is valid
        if (telemetry.isMapConfidenceValid) {
            val mapX = telemetry.mapMatchedPosEnu.x
            val mapY = telemetry.mapMatchedPosEnu.y
            val mapZ = telemetry.mapMatchedPosEnu.z
            synchronized(mapAidedPoints) {
                val last = mapAidedPoints.lastOrNull()
                if (last == null || distanceSq(last.enuX, last.enuY, mapX, mapY) > 0.16) {
                    if (mapAidedPoints.size >= maxTrajectoryPoints) {
                        mapAidedPoints.removeFirst()
                    }
                    mapAidedPoints.addLast(
                        TrajectoryPoint(
                            enuX = mapX,
                            enuY = mapY,
                            enuZ = mapZ,
                            timestampSec = currentSec,
                            isGnss = false,
                            isMapAided = true
                        )
                    )
                }
            }
        }

        // Check if we should update UI state (throttled to ~10 Hz)
        if (nowNanos - lastUiUpdateTimestampNanos < minUiUpdateIntervalNanos) {
            return
        }
        lastUiUpdateTimestampNanos = nowNanos

        val isDr = telemetry.mode.isDeadReckoning ||
                   telemetry.mode == NavigationMode.TRANSITION_TO_DR
        
        val outageDuration = if (isDr) {
            if (outageStartSec == null) {
                outageStartSec = currentSec
            }
            (currentSec - (outageStartSec ?: currentSec)).coerceAtLeast(0.0)
        } else {
            outageStartSec = null
            0.0
        }

        val currentDiag = _uiState.value.diagnostics
        val health = deriveHealthState(telemetry, currentDiag)
        val anchor = gnssManager?.anchorGeo?.let {
            GeoCoordinates.Wgs84Coordinate(it.anchorLatDeg, it.anchorLonDeg, it.anchorAltM)
        }

        // Trip distance accumulation
        if (lastEnuX != null && lastEnuY != null) {
            val d = Math.hypot(currentEnuX - lastEnuX!!, currentEnuY - lastEnuY!!)
            if (d in 0.001..50.0) {
                tripDistanceMetersTotal += d
            }
        }
        lastEnuX = currentEnuX
        lastEnuY = currentEnuY

        // Speed & Duration
        val speedKmh = telemetry.speedKmh
        if (speedKmh > tripMaxSpeedKmhVal) {
            tripMaxSpeedKmhVal = speedKmh
        }
        val elapsedSec = ((System.currentTimeMillis() - tripStartTimestampMs) / 1000L).coerceAtLeast(1L)
        val avgSpeedKmh = (tripDistanceMetersTotal / elapsedSec) * 3.6

        // Mode transition events
        if (prevNavMode != telemetry.mode) {
            val timeFmt = java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.US).format(java.util.Date())
            when (telemetry.mode) {
                NavigationMode.GNSS_AIDED, NavigationMode.GNSS_REACQUIRED -> {
                    if (prevNavMode == NavigationMode.DR_ONLY || prevNavMode == NavigationMode.TRANSITION_TO_DR) {
                        tripEventsList.add(0, org.isro.idr.ui.state.TripEvent(timeFmt, "GNSS reacquired", "GNSS_REACQUIRED"))
                    } else {
                        tripEventsList.add(0, org.isro.idr.ui.state.TripEvent(timeFmt, "GNSS Acquired", "GNSS_ACQUIRED"))
                    }
                }
                NavigationMode.DR_ONLY, NavigationMode.TRANSITION_TO_DR -> {
                    tripEventsList.add(0, org.isro.idr.ui.state.TripEvent(timeFmt, "GNSS lost - DR active", "DR_ACTIVE"))
                }
                else -> {}
            }
            if (telemetry.isMapConfidenceValid) {
                tripEventsList.add(0, org.isro.idr.ui.state.TripEvent(timeFmt, "Map aided DR", "MAP_AIDED"))
            }
            prevNavMode = telemetry.mode
        }

        // Drift tracking
        val gnssSample = gnssManager?.latestGnssSample
        if (gnssSample != null && gnssSample.isValid) {
            val drErr = Math.hypot(telemetry.posX - gnssSample.posEnu.x, telemetry.posY - gnssSample.posEnu.y).toFloat()
            val mapErr = if (telemetry.isMapConfidenceValid) {
                Math.hypot(telemetry.mapMatchedPosEnu.x - gnssSample.posEnu.x, telemetry.mapMatchedPosEnu.y - gnssSample.posEnu.y).toFloat()
            } else drErr
            val distKm = (tripDistanceMetersTotal / 1000.0).toFloat()
            if (tripDriftList.isEmpty() || (distKm - tripDriftList.last().distanceKm) >= 0.05f) {
                if (tripDriftList.size >= 50) tripDriftList.removeAt(0)
                tripDriftList.add(
                    org.isro.idr.ui.state.DriftPoint(
                        distanceKm = distKm,
                        gnssErrorM = 0.5f,
                        rawDrErrorM = drErr,
                        mapMatchedErrorM = mapErr
                    )
                )
            }
        }

        val accelSnap = synchronized(accelWaveforms) { accelWaveforms.toList() }
        val gyroSnap = synchronized(gyroWaveforms) { gyroWaveforms.toList() }
        val magSnap = synchronized(magWaveforms) { magWaveforms.toList() }
        val baroSnap = synchronized(baroWaveforms) { baroWaveforms.toList() }

        _uiState.update { current ->
            current.copy(
                telemetry = telemetry,
                navigationMode = telemetry.mode,
                nativeStatus = NativeStatus.OK,
                outageDurationSec = outageDuration,
                isDrActive = isDr,
                isGnssOutage = isDr || (current.gnssStatus.ageSeconds > 1.5),
                health = health,
                idrTrajectory = synchronized(idrPoints) { idrPoints.toList() },
                gnssTrajectory = synchronized(gnssPoints) { gnssPoints.toList() },
                mapAidedTrajectory = synchronized(mapAidedPoints) { mapAidedPoints.toList() },
                mapMatchedPositionEnu = if (telemetry.isMapConfidenceValid) telemetry.mapMatchedPosEnu else null,
                mapConfidence = when (telemetry.mapConfidence) {
                    0 -> "HIGH"
                    1 -> "MEDIUM"
                    2 -> "LOW"
                    else -> "STANDBY"
                },
                isMapAmbiguous = telemetry.isMapAmbiguous,
                candidateCount = telemetry.candidateCount,
                hypothesisCount = telemetry.hypothesisCount,
                scoreMargin = telemetry.scoreMargin,
                accelHistory = accelSnap,
                gyroHistory = gyroSnap,
                magHistory = magSnap,
                baroHistory = baroSnap,
                tripDistanceMeters = tripDistanceMetersTotal,
                tripDurationSec = elapsedSec,
                tripAvgSpeedKmh = avgSpeedKmh,
                tripMaxSpeedKmh = tripMaxSpeedKmhVal,
                tripEvents = tripEventsList.toList(),
                tripDriftData = tripDriftList.toList(),
                enuAnchor = anchor
            )
        }
    }

    fun onImuSample(imu: org.isro.idr.engine.ImuSample) {
        val nowMs = System.currentTimeMillis()
        synchronized(accelWaveforms) {
            if (accelWaveforms.size >= maxWaveformSamples) accelWaveforms.removeFirst()
            accelWaveforms.addLast(
                org.isro.idr.ui.state.SensorWaveformSample(
                    nowMs,
                    imu.accel.x.toFloat(),
                    imu.accel.y.toFloat(),
                    imu.accel.z.toFloat()
                )
            )
        }
        synchronized(gyroWaveforms) {
            if (gyroWaveforms.size >= maxWaveformSamples) gyroWaveforms.removeFirst()
            gyroWaveforms.addLast(
                org.isro.idr.ui.state.SensorWaveformSample(
                    nowMs,
                    imu.gyro.x.toFloat(),
                    imu.gyro.y.toFloat(),
                    imu.gyro.z.toFloat()
                )
            )
        }
        if (imu.baroAltMeters != null) {
            val alt = imu.baroAltMeters.toFloat()
            synchronized(baroWaveforms) {
                if (baroWaveforms.size >= maxWaveformSamples) baroWaveforms.removeFirst()
                baroWaveforms.addLast(
                    org.isro.idr.ui.state.SensorWaveformSample(nowMs, alt, alt, alt)
                )
            }
        }
    }

    fun onMagSample(mag: org.isro.idr.engine.MagnetometerSample) {
        val nowMs = System.currentTimeMillis()
        synchronized(magWaveforms) {
            if (magWaveforms.size >= maxWaveformSamples) magWaveforms.removeFirst()
            magWaveforms.addLast(
                org.isro.idr.ui.state.SensorWaveformSample(
                    nowMs,
                    mag.magUteslas.x.toFloat(),
                    mag.magUteslas.y.toFloat(),
                    mag.magUteslas.z.toFloat()
                )
            )
        }
    }

    fun onSatelliteStatus(satellites: List<org.isro.idr.ui.state.SatelliteInfo>) {
        _uiState.update { current ->
            current.copy(
                gnssStatus = current.gnssStatus.copy(
                    satellites = satellites,
                    totalSatellitesInView = satellites.size,
                    totalSatellites = satellites.count { it.usedInFix },
                    navicSatellitesInView = satellites.count { it.constellation == org.isro.idr.ui.state.ConstellationType.NAVIC },
                    navicSatellitesUsed = satellites.count { it.constellation == org.isro.idr.ui.state.ConstellationType.NAVIC && it.usedInFix }
                )
            )
        }
    }

    fun onGnssSample(sample: GnssSample) {
        val anchor = gnssManager?.anchorGeo?.let {
            GeoCoordinates.Wgs84Coordinate(it.anchorLatDeg, it.anchorLonDeg, it.anchorAltM)
        }

        val currentGeo = gnssManager?.anchorGeo
        if (currentGeo != null && rawRoadGraph != null && _uiState.value.roadMapGraph?.currentAnchor != currentGeo) {
            val reprojected = rawRoadGraph?.withEnuProjection(currentGeo)
            _uiState.update { it.copy(roadMapGraph = reprojected) }
        }

        if (sample.isValid) {
            synchronized(gnssPoints) {
                val last = gnssPoints.lastOrNull()
                if (last == null || distanceSq(last.enuX, last.enuY, sample.posEnu.x, sample.posEnu.y) > 0.16) {
                    if (gnssPoints.size >= maxTrajectoryPoints) {
                        gnssPoints.removeFirst()
                    }
                    gnssPoints.addLast(
                        TrajectoryPoint(
                            enuX = sample.posEnu.x,
                            enuY = sample.posEnu.y,
                            enuZ = sample.posEnu.z,
                            timestampSec = sample.timeSeconds,
                            isGnss = true
                        )
                    )
                }
            }

            _uiState.update { current ->
                val currentSec = current.telemetry?.timestampSec ?: sample.timeSeconds
                val age = (currentSec - sample.timeSeconds).coerceAtLeast(0.0)
                current.copy(
                    gnssStatus = GnssStatusState(
                        fixValid = true,
                        isSearching = false,
                        isPermissionGranted = true,
                        fixState = if (age <= 1.5) "FIXED" else "STALE",
                        latitude = sample.latitudeDeg,
                        longitude = sample.longitudeDeg,
                        altitude = sample.altitudeM,
                        speedMps = sample.groundSpeedMps,
                        bearingDeg = sample.courseDeg,
                        accuracyM = sample.horizontalAccuracyMeters.toDouble(),
                        ageSeconds = age,
                        totalSatellites = sample.totalSatelliteCount,
                        totalSatellitesInView = gnssManager?.totalInViewCount ?: sample.totalSatelliteCount,
                        navicSatellitesInView = gnssManager?.navicInViewCount ?: sample.navicSatelliteCount,
                        navicSatellitesUsed = sample.navicSatelliteCount
                    ),
                    gnssTrajectory = synchronized(gnssPoints) { gnssPoints.toList() },
                    enuAnchor = anchor
                )
            }
        } else {
            _uiState.update { current ->
                current.copy(
                    gnssStatus = current.gnssStatus.copy(
                        fixValid = false,
                        isSearching = gnssManager?.isRunning == true,
                        isPermissionGranted = gnssManager?.isPermissionGranted ?: true,
                        fixState = if (gnssManager?.isRunning == true) "SEARCHING" else "STANDBY",
                        totalSatellites = sample.totalSatelliteCount,
                        totalSatellitesInView = gnssManager?.totalInViewCount ?: sample.totalSatelliteCount,
                        navicSatellitesInView = gnssManager?.navicInViewCount ?: sample.navicSatelliteCount,
                        navicSatellitesUsed = sample.navicSatelliteCount
                    ),
                    enuAnchor = anchor
                )
            }
        }
    }

    fun updateDiagnostics(snapshot: IngestionDiagnostics) {
        val health = deriveHealthState(_uiState.value.telemetry, snapshot)
        _uiState.update { current ->
            current.copy(
                diagnostics = snapshot,
                health = health,
                sensorConsumedRateHz = snapshot.accelConsumedHz.coerceAtLeast(0.0),
                sensorCallbackRateHz = snapshot.accelCallbackHz.coerceAtLeast(0.0)
            )
        }
    }

    // Map & Controls
    fun toggleNorthUp() {
        _uiState.update { it.copy(isNorthUp = !it.isNorthUp) }
    }

    fun zoomIn() {
        _uiState.update { it.copy(mapZoomLevel = (it.mapZoomLevel * 1.35f).coerceAtMost(8.0f)) }
    }

    fun zoomOut() {
        _uiState.update { it.copy(mapZoomLevel = (it.mapZoomLevel / 1.35f).coerceAtLeast(0.25f)) }
    }

    fun resetMap() {
        _uiState.update { it.copy(mapZoomLevel = 1.0f, isNorthUp = true) }
    }

    fun setTrajectoryMode(mode: TrajectoryDisplayMode) {
        _uiState.update { it.copy(trajectoryMode = mode) }
    }

    fun setSelectedScreen(tab: ScreenTab) {
        _uiState.update { it.copy(selectedScreen = tab, activeSubScreen = SubScreen.NONE) }
    }

    // Map Explorer Actions
    fun setExplorerSearchQuery(query: String) {
        _uiState.update { it.copy(explorerSearchQuery = query) }
    }

    fun setMapStyle(style: MapStyleType) {
        _uiState.update { it.copy(mapStyle = style) }
    }

    fun toggleRoadsLayer(show: Boolean) {
        _uiState.update { it.copy(showRoadsLayer = show) }
    }

    fun toggle3dBuildingsLayer(show: Boolean) {
        _uiState.update { it.copy(show3dBuildingsLayer = show) }
    }

    fun toggleLandmarksLayer(show: Boolean) {
        _uiState.update { it.copy(showLandmarksLayer = show) }
    }

    fun toggleRiversWaterLayer(show: Boolean) {
        _uiState.update { it.copy(showRiversWaterLayer = show) }
    }

    // Sensor Monitor
    fun setSensorPlotTab(tab: SensorPlotTab) {
        _uiState.update { it.copy(selectedSensorPlotTab = tab) }
    }

    // Offline Maps
    fun downloadRegionPack(id: String) {
        _uiState.update { current ->
            val updated = current.offlineRegionPacks.map {
                if (it.id == id) it.copy(isInstalled = true) else it
            }
            current.copy(offlineRegionPacks = updated)
        }
    }

    fun activateRegionPack(id: String) {
        _uiState.update { current ->
            val target = current.offlineRegionPacks.find { it.id == id }
            if (target != null) {
                val fileName = "${target.id}_corridor.idrmap"
                current.copy(selectedMapFileName = fileName)
            } else current
        }
    }

    // Settings
    fun toggleFollowVehicle(enabled: Boolean) {
        _uiState.update { it.copy(followVehicle = enabled) }
    }

    fun toggleHeadingUp(enabled: Boolean) {
        _uiState.update { it.copy(headingUpMode = enabled, isNorthUp = !enabled) }
    }

    fun toggle3dBuildingsSetting(enabled: Boolean) {
        _uiState.update { it.copy(show3dBuildings = enabled, show3dBuildingsLayer = enabled) }
    }

    fun toggleAutoRecenter(enabled: Boolean) {
        _uiState.update { it.copy(autoRecenter = enabled) }
    }

    fun clearTrajectories() {
        synchronized(idrPoints) { idrPoints.clear() }
        synchronized(gnssPoints) { gnssPoints.clear() }
        synchronized(mapAidedPoints) { mapAidedPoints.clear() }
        _uiState.update {
            it.copy(
                idrTrajectory = emptyList(),
                gnssTrajectory = emptyList(),
                mapAidedTrajectory = emptyList()
            )
        }
    }

    fun setRunning(running: Boolean) {
        _uiState.update { it.copy(isRunning = running) }
    }

    private fun deriveHealthState(
        telemetry: NavigationTelemetry?,
        diag: IngestionDiagnostics?
    ): SystemHealthState {
        val imuHealth = when {
            diag != null && diag.accelConsumedHz > 40.0 && diag.gyroConsumedHz > 40.0 -> HealthStatus.HEALTHY
            diag != null && (diag.isAccelerometerAvailable || diag.isGyroscopeAvailable) -> HealthStatus.SEARCHING
            diag == null && telemetry != null -> HealthStatus.HEALTHY
            else -> HealthStatus.SEARCHING
        }

        val gnssHealth = when {
            telemetry?.gnssValid == true || _uiState.value.gnssStatus.fixValid -> HealthStatus.HEALTHY
            gnssManager?.isPermissionGranted == false -> HealthStatus.UNAVAILABLE
            gnssManager?.isRunning == true -> HealthStatus.SEARCHING
            else -> HealthStatus.SEARCHING
        }

        val navHealth = when {
            telemetry != null && telemetry.pTrace.isFinite() && telemetry.mode != NavigationMode.INITIALIZING -> HealthStatus.HEALTHY
            telemetry?.mode == NavigationMode.INITIALIZING || _uiState.value.isRunning -> HealthStatus.SEARCHING
            else -> HealthStatus.SEARCHING
        }

        val navicUsed = gnssManager?.navicSatCount ?: _uiState.value.gnssStatus.navicSatellitesUsed
        val navicInView = gnssManager?.navicInViewCount ?: _uiState.value.gnssStatus.navicSatellitesInView
        val navicHealth = when {
            navicUsed > 0 -> HealthStatus.HEALTHY
            navicInView > 0 -> HealthStatus.SEARCHING
            gnssManager?.isRunning == true -> HealthStatus.SEARCHING
            else -> HealthStatus.SEARCHING
        }

        val mapHealth = when {
            _uiState.value.roadMapGraph != null -> HealthStatus.HEALTHY
            telemetry?.mapAvailable == true -> HealthStatus.HEALTHY
            _uiState.value.mapDataState == MapDataState.AVAILABLE || _uiState.value.mapDataState == MapDataState.READY -> HealthStatus.HEALTHY
            else -> HealthStatus.OFFLINE
        }

        return SystemHealthState(
            imuStatus = imuHealth,
            gnssStatus = gnssHealth,
            navStatus = navHealth,
            mapStatus = mapHealth,
            navicStatus = navicHealth,
            jniStatus = HealthStatus.HEALTHY
        )
    }

    private fun distanceSq(x1: Double, y1: Double, x2: Double, y2: Double): Double {
        val dx = x2 - x1
        val dy = y2 - y1
        return dx * dx + dy * dy
    }

    override fun onCleared() {
        super.onCleared()
        synchronizer?.onTelemetryListener = null
        gnssManager?.onGnssSampleListener = null
    }
}

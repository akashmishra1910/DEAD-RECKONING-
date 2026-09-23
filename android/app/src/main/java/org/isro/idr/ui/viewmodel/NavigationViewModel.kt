package org.isro.idr.ui.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GnssIngestionManager
import org.isro.idr.ingestion.GnssSample
import org.isro.idr.ingestion.IngestionHealthSnapshot
import org.isro.idr.ingestion.NavigationPipelineSynchronizer
import org.isro.idr.ui.state.GnssStatusState
import org.isro.idr.ui.state.HealthStatus
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.ScreenTab
import org.isro.idr.ui.state.SystemHealthState
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint

/**
 * ViewModel orchestrating UI state for the ISRO IDR Navigation application.
 * Decouples high-rate native estimator execution (~100 Hz) from UI recomposition (~10 Hz).
 * Strictly consumes immutable NavigationTelemetry without modifying navigation mathematics.
 */
class NavigationViewModel(
    private val synchronizer: NavigationPipelineSynchronizer? = null,
    private val gnssManager: GnssIngestionManager? = null
) : ViewModel(), NavigationPipelineSynchronizer.TelemetryListener, GnssIngestionManager.GnssListener {

    private val _uiState = MutableStateFlow(NavigationUiState())
    val uiState: StateFlow<NavigationUiState> = _uiState.asStateFlow()

    private val maxTrajectoryPoints = 1000
    private var lastUiUpdateTimestampNanos = 0L
    private val minUiUpdateIntervalNanos = 100_000_000L // 100ms = 10 Hz UI rate

    // Outage duration tracking
    private var outageStartSec: Double? = null

    // Bounded mutable lists for trajectories (synced on copy)
    private val idrPoints = ArrayDeque<TrajectoryPoint>(maxTrajectoryPoints)
    private val gnssPoints = ArrayDeque<TrajectoryPoint>(maxTrajectoryPoints)
    private val mapAidedPoints = ArrayDeque<TrajectoryPoint>(maxTrajectoryPoints)

    init {
        synchronizer?.setTelemetryListener(this)
        gnssManager?.addListener(this)
    }

    /**
     * Consumes native telemetry at native rate (~100 Hz), buffers trajectory,
     * and publishes throttled UI updates at ~10 Hz.
     */
    override fun onTelemetryReceived(telemetry: NavigationTelemetry) {
        val nowNanos = System.nanoTime()

        // Accumulate IDR trajectory point (with distance decimation to prevent clutter)
        val currentEnuX = telemetry.posX
        val currentEnuY = telemetry.posY
        val currentEnuZ = telemetry.posZ
        val currentSec = telemetry.timestampSec

        synchronized(idrPoints) {
            val last = idrPoints.lastOrNull()
            if (last == null || distanceSq(last.enuX, last.enuY, currentEnuX, currentEnuY) > 0.25) { // >= 0.5m
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
                if (last == null || distanceSq(last.enuX, last.enuY, mapX, mapY) > 0.25) {
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

        // Determine DR state and outage duration
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

        // Derive health states
        val health = deriveHealthState(telemetry)

        _uiState.update { current ->
            current.copy(
                telemetry = telemetry,
                navigationMode = telemetry.mode,
                nativeStatus = NativeStatus.SUCCESS,
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
                    else -> "INVALID"
                },
                isMapAmbiguous = telemetry.isMapAmbiguous,
                candidateCount = telemetry.candidateCount,
                hypothesisCount = telemetry.hypothesisCount,
                scoreMargin = telemetry.scoreMargin,
                enuAnchor = gnssManager?.getAnchorCoordinate()
            )
        }
    }


    override fun onGnssSample(sample: GnssSample) {
        if (sample.isValid) {
            synchronized(gnssPoints) {
                val last = gnssPoints.lastOrNull()
                if (last == null || distanceSq(last.enuX, last.enuY, sample.enuX, sample.enuY) > 0.25) {
                    if (gnssPoints.size >= maxTrajectoryPoints) {
                        gnssPoints.removeFirst()
                    }
                    gnssPoints.addLast(
                        TrajectoryPoint(
                            enuX = sample.enuX,
                            enuY = sample.enuY,
                            enuZ = sample.enuZ,
                            timestampSec = sample.timestampSec,
                            isGnss = true
                        )
                    )
                }
            }

            _uiState.update { current ->
                val currentSec = current.telemetry?.timestampSec ?: sample.timestampSec
                val age = (currentSec - sample.timestampSec).coerceAtLeast(0.0)
                current.copy(
                    gnssStatus = GnssStatusState(
                        fixValid = true,
                        latitude = sample.latitude,
                        longitude = sample.longitude,
                        altitude = sample.altitude,
                        speedMps = sample.speedMps,
                        bearingDeg = sample.bearingDeg,
                        accuracyM = sample.accuracyM,
                        ageSeconds = age,
                        totalSatellites = sample.totalSatellites,
                        navicSatellitesInView = sample.navicSatellitesInView,
                        navicSatellitesUsed = sample.navicSatellitesUsed
                    ),
                    enuAnchor = gnssManager?.getAnchorCoordinate()
                )
            }
        } else {
            _uiState.update { current ->
                current.copy(
                    gnssStatus = current.gnssStatus.copy(
                        fixValid = false,
                        totalSatellites = sample.totalSatellites,
                        navicSatellitesInView = sample.navicSatellitesInView,
                        navicSatellitesUsed = sample.navicSatellitesUsed
                    )
                )
            }
        }
    }

    fun updateDiagnostics(snapshot: IngestionHealthSnapshot) {
        _uiState.update { current ->
            current.copy(diagnostics = snapshot)
        }
    }

    fun setTrajectoryMode(mode: TrajectoryDisplayMode) {
        _uiState.update { it.copy(trajectoryMode = mode) }
    }

    fun setSelectedScreen(tab: ScreenTab) {
        _uiState.update { it.copy(selectedScreen = tab) }
    }

    fun setMapDataState(state: MapDataState) {
        _uiState.update { it.copy(mapDataState = state) }
    }

    fun clearTrajectories() {
        synchronized(idrPoints) { idrPoints.clear() }
        synchronized(gnssPoints) { gnssPoints.clear() }
        _uiState.update {
            it.copy(
                idrTrajectory = emptyList(),
                gnssTrajectory = emptyList()
            )
        }
    }

    fun setRunning(running: Boolean) {
        _uiState.update { it.copy(isRunning = running) }
    }

    fun setError(message: String?) {
        _uiState.update { it.copy(errorMessage = message) }
    }

    private fun deriveHealthState(telemetry: NavigationTelemetry): SystemHealthState {
        val navHealth = when (telemetry.status) {
            NativeStatus.SUCCESS.code -> HealthStatus.HEALTHY
            NativeStatus.INITIALIZATION_FAILED.code,
            NativeStatus.INTERNAL_FAILURE.code,
            NativeStatus.NUMERICAL_FAILURE.code -> HealthStatus.UNAVAILABLE
            else -> HealthStatus.DEGRADED
        }

        val gnssHealth = if (telemetry.gnssValid) {
            HealthStatus.HEALTHY
        } else {
            HealthStatus.DEGRADED
        }

        val navicHealth = if (telemetry.navicSatellitesUsedInFix > 0) {
            HealthStatus.HEALTHY
        } else if (telemetry.navicSatellitesInView > 0) {
            HealthStatus.DEGRADED
        } else {
            HealthStatus.UNAVAILABLE
        }

        return SystemHealthState(
            imuStatus = HealthStatus.HEALTHY,
            gnssStatus = gnssHealth,
            navStatus = navHealth,
            mapStatus = HealthStatus.UNAVAILABLE, // Phase 11.4 baseline
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
        synchronizer?.setTelemetryListener(null)
        gnssManager?.removeListener(this)
    }
}

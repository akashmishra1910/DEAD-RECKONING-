package org.isro.idr.ui.state

import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GeoCoordinates
import org.isro.idr.ingestion.IngestionHealthSnapshot

enum class TrajectoryDisplayMode {
    BOTH,
    IDR_ONLY,
    GNSS_ONLY,
    OFF
}

enum class MapDataState {
    AVAILABLE,
    UNAVAILABLE,
    LOADING,
    OFFLINE
}

enum class ScreenTab {
    NAVIGATION,
    DIAGNOSTICS,
    MAP_DATA,
    SETTINGS
}

enum class HealthStatus {
    HEALTHY,
    DEGRADED,
    UNAVAILABLE
}

data class SystemHealthState(
    val imuStatus: HealthStatus = HealthStatus.UNAVAILABLE,
    val gnssStatus: HealthStatus = HealthStatus.UNAVAILABLE,
    val navStatus: HealthStatus = HealthStatus.UNAVAILABLE,
    val mapStatus: HealthStatus = HealthStatus.UNAVAILABLE,
    val navicStatus: HealthStatus = HealthStatus.UNAVAILABLE,
    val jniStatus: HealthStatus = HealthStatus.HEALTHY
)

data class GnssStatusState(
    val fixValid: Boolean = false,
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val altitude: Double = 0.0,
    val speedMps: Double = 0.0,
    val bearingDeg: Double = 0.0,
    val accuracyM: Double = 0.0,
    val ageSeconds: Double = 0.0,
    val totalSatellites: Int = 0,
    val navicSatellitesInView: Int = 0,
    val navicSatellitesUsed: Int = 0
)

data class TrajectoryPoint(
    val enuX: Double,
    val enuY: Double,
    val enuZ: Double = 0.0,
    val timestampSec: Double = 0.0,
    val isGnss: Boolean = false
)

data class NavigationUiState(
    val telemetry: NavigationTelemetry? = null,
    val gnssStatus: GnssStatusState = GnssStatusState(),
    val navigationMode: NavigationMode = NavigationMode.INITIALIZING,
    val nativeStatus: NativeStatus = NativeStatus.SUCCESS,
    val outageDurationSec: Double = 0.0,
    val isDrActive: Boolean = false,
    val isGnssOutage: Boolean = false,
    val mapDataState: MapDataState = MapDataState.OFFLINE,
    val trajectoryMode: TrajectoryDisplayMode = TrajectoryDisplayMode.BOTH,
    val idrTrajectory: List<TrajectoryPoint> = emptyList(),
    val gnssTrajectory: List<TrajectoryPoint> = emptyList(),
    val health: SystemHealthState = SystemHealthState(),
    val diagnostics: IngestionHealthSnapshot? = null,
    val selectedScreen: ScreenTab = ScreenTab.NAVIGATION,
    val enuAnchor: GeoCoordinates.Wgs84Coordinate? = null,
    val isRunning: Boolean = false,
    val errorMessage: String? = null
)

package org.isro.idr.ui.state

import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GeoCoordinates
import org.isro.idr.ingestion.IngestionDiagnostics

enum class TrajectoryDisplayMode(val displayName: String) {
    BOTH("ALL TRACKS"),
    IDR_ONLY("DR ONLY"),
    GNSS_ONLY("GNSS ONLY"),
    MAP_AIDED("MAP-AIDED"),
    OFF("OFF")
}

enum class MapDataState(val displayName: String) {
    READY("READY"),
    AVAILABLE("AVAILABLE"),
    UNAVAILABLE("NOT LOADED"),
    LOADING("LOADING..."),
    OFFLINE("OFFLINE TACTICAL")
}

enum class ScreenTab(val title: String) {
    NAVIGATION("NAV"),
    DIAGNOSTICS("DIAGNOSTICS"),
    MAP_DATA("MAP DATA"),
    SETTINGS("SETTINGS")
}

enum class HealthStatus(val label: String) {
    HEALTHY("ACTIVE"),
    SEARCHING("SEARCHING"),
    DEGRADED("DEGRADED"),
    OFFLINE("OFFLINE"),
    UNAVAILABLE("NOT AVAIL"),
    ERROR("ERROR")
}

data class SystemHealthState(
    val imuStatus: HealthStatus = HealthStatus.SEARCHING,
    val gnssStatus: HealthStatus = HealthStatus.SEARCHING,
    val navStatus: HealthStatus = HealthStatus.SEARCHING,
    val mapStatus: HealthStatus = HealthStatus.OFFLINE,
    val navicStatus: HealthStatus = HealthStatus.SEARCHING,
    val jniStatus: HealthStatus = HealthStatus.HEALTHY
)

data class GnssStatusState(
    val fixValid: Boolean = false,
    val isSearching: Boolean = true,
    val isPermissionGranted: Boolean = true,
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val altitude: Double = 0.0,
    val speedMps: Double = 0.0,
    val bearingDeg: Double = 0.0,
    val accuracyM: Double = 0.0,
    val ageSeconds: Double = 0.0,
    val totalSatellites: Int = 0,
    val totalSatellitesInView: Int = 0,
    val navicSatellitesInView: Int = 0,
    val navicSatellitesUsed: Int = 0
)

data class TrajectoryPoint(
    val enuX: Double,
    val enuY: Double,
    val enuZ: Double = 0.0,
    val timestampSec: Double = 0.0,
    val isGnss: Boolean = false,
    val isMapAided: Boolean = false
)

data class NavigationUiState(
    val telemetry: NavigationTelemetry? = null,
    val gnssStatus: GnssStatusState = GnssStatusState(),
    val navigationMode: NavigationMode = NavigationMode.INITIALIZING,
    val nativeStatus: NativeStatus = NativeStatus.OK,
    val outageDurationSec: Double = 0.0,
    val isDrActive: Boolean = false,
    val isGnssOutage: Boolean = false,
    val mapDataState: MapDataState = MapDataState.OFFLINE,
    val trajectoryMode: TrajectoryDisplayMode = TrajectoryDisplayMode.BOTH,
    val isNorthUp: Boolean = true,
    val mapZoomLevel: Float = 1.0f,
    val idrTrajectory: List<TrajectoryPoint> = emptyList(),
    val gnssTrajectory: List<TrajectoryPoint> = emptyList(),
    val mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    val mapMatchedPositionEnu: org.isro.idr.engine.Vector3D? = null,
    val selectedRoadName: String = "Local Tactical Corridor",
    val selectedRoadLayer: String = "Ground",
    val mapConfidence: String = "STANDBY",
    val isMapAmbiguous: Boolean = false,
    val candidateCount: Int = 0,
    val hypothesisCount: Int = 0,
    val scoreMargin: Double = 0.0,
    val health: SystemHealthState = SystemHealthState(),
    val diagnostics: IngestionDiagnostics? = null,
    val selectedScreen: ScreenTab = ScreenTab.NAVIGATION,
    val enuAnchor: GeoCoordinates.Wgs84Coordinate? = null,
    val isRunning: Boolean = true,
    val errorMessage: String? = null
)

package org.isro.idr.ui.state

import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GeoCoordinates
import org.isro.idr.ingestion.IngestionDiagnostics
import org.isro.idr.map.RoadMapGraph

/**
 * Primary 5 bottom-navigation destination tabs.
 */
enum class ScreenTab(val title: String) {
    NAVIGATION("Navigate"),
    MAP_EXPLORER("Map"),
    TRIP_ANALYTICS("Trip"),
    DIAGNOSTICS("Diagnose"),
    SETTINGS("Settings")
}

/**
 * Top-level application flow states (Splash -> Calibration -> Live Navigation).
 */
enum class AppFlowState {
    SPLASH,
    CALIBRATION,
    MAIN_APP
}

/**
 * Sub-screens accessible within the Diagnostics or Settings destinations.
 */
enum class SubScreen {
    NONE,
    SENSOR_MONITOR,
    GNSS_NAVIC_SKYVIEW,
    OFFLINE_MAPS_MANAGER
}

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

enum class MapStyleType(val label: String) {
    STANDARD("Standard"),
    SATELLITE("Satellite"),
    DARK("Dark"),
    TERRAIN("Terrain")
}

enum class HealthStatus(val label: String) {
    HEALTHY("ACTIVE"),
    SEARCHING("SEARCHING"),
    DEGRADED("DEGRADED"),
    OFFLINE("OFFLINE"),
    UNAVAILABLE("NOT AVAIL"),
    ERROR("ERROR")
}

enum class SensorPlotTab(val label: String) {
    ACCEL("Accel"),
    GYRO("Gyro"),
    MAG("Mag"),
    BARO("Baro")
}

enum class ConstellationType {
    GPS,
    NAVIC,
    GLONASS,
    GALILEO,
    BEIDOU
}

data class SatelliteInfo(
    val svid: Int,
    val constellation: ConstellationType,
    val elevationDeg: Float,
    val azimuthDeg: Float,
    val snrDbHz: Float,
    val usedInFix: Boolean
)

data class SystemHealthState(
    val imuStatus: HealthStatus = HealthStatus.HEALTHY,
    val gnssStatus: HealthStatus = HealthStatus.SEARCHING,
    val navStatus: HealthStatus = HealthStatus.SEARCHING,
    val mapStatus: HealthStatus = HealthStatus.HEALTHY,
    val navicStatus: HealthStatus = HealthStatus.HEALTHY,
    val jniStatus: HealthStatus = HealthStatus.HEALTHY
)

data class GnssStatusState(
    val fixValid: Boolean = false,
    val isSearching: Boolean = true,
    val isPermissionGranted: Boolean = true,
    val fixState: String = "SEARCHING", // SEARCHING, FIXED, STALE, OUTAGE
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val altitude: Double = 0.0,
    val speedMps: Double = 0.0,
    val bearingDeg: Double = 0.0,
    val accuracyM: Double = 0.0,
    val hdop: Double = 0.8,
    val ageSeconds: Double = 0.0,
    val totalSatellites: Int = 24,
    val totalSatellitesInView: Int = 24,
    val navicSatellitesInView: Int = 4,
    val navicSatellitesUsed: Int = 4,
    val satellites: List<SatelliteInfo> = emptyList()
)

data class TrajectoryPoint(
    val enuX: Double,
    val enuY: Double,
    val enuZ: Double = 0.0,
    val timestampSec: Double = 0.0,
    val isGnss: Boolean = false,
    val isMapAided: Boolean = false
)

data class SensorWaveformSample(
    val timestampMs: Long,
    val x: Float,
    val y: Float,
    val z: Float
)

data class TripEvent(
    val timestamp: String,
    val title: String,
    val type: String // "GNSS_ACQUIRED", "DR_ACTIVE", "MAP_AIDED", "GNSS_REACQUIRED"
)

data class DriftPoint(
    val distanceKm: Float,
    val gnssErrorM: Float,
    val rawDrErrorM: Float,
    val mapMatchedErrorM: Float
)

data class OfflineRegionPack(
    val id: String,
    val name: String,
    val subtitle: String,
    val sizeBytes: Long,
    val version: String = "v2024.10",
    val isInstalled: Boolean = false,
    val isDownloading: Boolean = false,
    val downloadProgress: Float = 0f,
    val category: String = "India" // "India", "Global", "My Regions"
)

data class NavigationUiState(
    // App Navigation & Screen State
    val appFlow: AppFlowState = AppFlowState.MAIN_APP,
    val selectedScreen: ScreenTab = ScreenTab.NAVIGATION,
    val activeSubScreen: SubScreen = SubScreen.NONE,

    // IMU Calibration Flow State
    val calibrationProgress: Float = 1.0f,
    val isAccelCalibrated: Boolean = true,
    val isGyroCalibrated: Boolean = true,
    val isMagCalibrated: Boolean = true,
    val isBaroCalibrated: Boolean = true,

    // Live Telemetry & Estimator States
    val telemetry: NavigationTelemetry? = null,
    val gnssStatus: GnssStatusState = GnssStatusState(),
    val navigationMode: NavigationMode = NavigationMode.INITIALIZING,
    val nativeStatus: NativeStatus = NativeStatus.OK,
    val outageDurationSec: Double = 0.0,
    val isDrActive: Boolean = false,
    val isGnssOutage: Boolean = false,
    val isGnssReacquired: Boolean = false,

    // Road Map & 3D HMM State
    val mapDataState: MapDataState = MapDataState.READY,
    val roadMapGraph: RoadMapGraph? = null,
    val selectedMapFileName: String = "prayagraj_corridor.idrmap",
    val availableMapNames: List<String> = listOf("prayagraj_corridor.idrmap", "dense_metro.idrmap", "regional_network.idrmap"),
    val trajectoryMode: TrajectoryDisplayMode = TrajectoryDisplayMode.BOTH,
    val isNorthUp: Boolean = true,
    val is3dMode: Boolean = true,
    val mapZoomLevel: Float = 1.0f,
    val mapStyle: MapStyleType = MapStyleType.DARK,
    val idrTrajectory: List<TrajectoryPoint> = emptyList(),
    val gnssTrajectory: List<TrajectoryPoint> = emptyList(),
    val mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    val mapMatchedPositionEnu: org.isro.idr.engine.Vector3D? = null,
    val selectedRoadName: String = "Mahatma Gandhi Marg",
    val nextTurnDistanceMeters: Int = 350,
    val nextTurnDirection: String = "Turn right",
    val selectedRoadLayer: String = "Ground",
    val mapConfidence: String = "HIGH",
    val isMapAmbiguous: Boolean = false,
    val candidateCount: Int = 3,
    val hypothesisCount: Int = 3,
    val scoreMargin: Double = 0.85,

    // Map Explorer Layer Toggles & Search
    val explorerSearchQuery: String = "",
    val showRoadsLayer: Boolean = true,
    val show3dBuildingsLayer: Boolean = true,
    val showLandmarksLayer: Boolean = true,
    val showRiversWaterLayer: Boolean = true,
    val showTerrainLayer: Boolean = false,

    // Sensor Monitor State
    val selectedSensorPlotTab: SensorPlotTab = SensorPlotTab.ACCEL,
    val accelHistory: List<SensorWaveformSample> = emptyList(),
    val gyroHistory: List<SensorWaveformSample> = emptyList(),
    val magHistory: List<SensorWaveformSample> = emptyList(),
    val baroHistory: List<SensorWaveformSample> = emptyList(),
    val sensorCallbackRateHz: Double = 468.8,
    val sensorConsumedRateHz: Double = 100.0,
    val baroPressureHpa: Double = 1013.25,
    val baroAltitudeM: Double = 98.4,

    // Trip & Analytics State
    val tripStartTime: String = "12 Nov 2024, 10:24",
    val tripDurationSec: Long = 9374,
    val tripDistanceMeters: Double = 12800.0,
    val tripAvgSpeedKmh: Double = 29.4,
    val tripMaxSpeedKmh: Double = 78.6,
    val tripDriftData: List<DriftPoint> = emptyList(),
    val tripEvents: List<TripEvent> = listOf(
        TripEvent("10:24:12", "GNSS Acquired", "GNSS_ACQUIRED"),
        TripEvent("11:05:24", "GNSS lost - DR active", "DR_ACTIVE"),
        TripEvent("11:05:30", "Map aided DR", "MAP_AIDED"),
        TripEvent("11:22:18", "GNSS reacquired", "GNSS_REACQUIRED")
    ),

    // Offline Maps Region Packs
    val offlineRegionPacks: List<OfflineRegionPack> = listOf(
        OfflineRegionPack("prayagraj", "Prayagraj (Allahabad)", "Roads, Buildings, Terrain, POIs", 1288490188L, "v2024.10", isInstalled = true, category = "India"),
        OfflineRegionPack("delhi", "Delhi NCR", "Full Capital Metro Vector Pack", 2576980377L, "v2024.10", isInstalled = false, category = "India"),
        OfflineRegionPack("mumbai", "Mumbai", "Coastal Corridor & Sea Link", 2791728742L, "v2024.10", isInstalled = false, category = "India"),
        OfflineRegionPack("bengaluru", "Bengaluru", "Tech Corridor & Outer Ring", 2254857830L, "v2024.10", isInstalled = false, category = "India"),
        OfflineRegionPack("chennai", "Chennai", "Urban Coastline & OMR", 1932735283L, "v2024.10", isInstalled = false, category = "India"),
        OfflineRegionPack("global_base", "World Base Map", "Low/Mid Zoom Global Continents", 536870912L, "v2024.10", isInstalled = true, category = "Global"),
        OfflineRegionPack("europe", "Western Europe", "OpenMapTiles Continental Pack", 15032385536L, "v2024.10", isInstalled = false, category = "Global"),
        OfflineRegionPack("north_america", "North America", "Continental Highway & Street Grid", 18253611008L, "v2024.10", isInstalled = false, category = "Global")
    ),

    // Diagnostics & Health State
    val health: SystemHealthState = SystemHealthState(),
    val diagnostics: IngestionDiagnostics? = null,
    val nativeLatencyUs: Long = 1402,
    val p95SensorJitterMs: Double = 2.13,
    val memoryUsageMb: Int = 72,

    // Settings State
    val followVehicle: Boolean = true,
    val headingUpMode: Boolean = true,
    val show3dBuildings: Boolean = true,
    val autoRecenter: Boolean = true,
    val speedUnit: String = "km/h",
    val distanceUnit: String = "Kilometers",
    val coordinateFormat: String = "Decimal (WGS84)",

    // Infrastructure
    val enuAnchor: GeoCoordinates.Wgs84Coordinate? = null,
    val isRunning: Boolean = true,
    val errorMessage: String? = null
)

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

enum class ConstellationType(val displayName: String) {
    GPS("GPS"),
    NAVIC("NavIC / IRNSS"),
    GLONASS("GLONASS"),
    GALILEO("Galileo"),
    BEIDOU("BeiDou"),
    QZSS("QZSS"),
    SBAS("SBAS"),
    UNKNOWN("Unknown")
}

data class SatelliteInfo(
    val svid: Int,
    val constellation: ConstellationType,
    val elevationDeg: Float,
    val azimuthDeg: Float,
    val snrDbHz: Float,
    val usedInFix: Boolean,
    val carrierFreqHz: Float = 0.0f
)

data class TrustedLocationFix(
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val accuracyM: Double,
    val timestampMs: Long,
    val isPreservedFromStorage: Boolean = false
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
    val totalSatellites: Int = 0,
    val totalSatellitesInView: Int = 0,
    val navicSatellitesInView: Int = 0,
    val navicSatellitesUsed: Int = 0,
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
    val appFlow: AppFlowState = AppFlowState.SPLASH,
    val selectedScreen: ScreenTab = ScreenTab.NAVIGATION,
    val activeSubScreen: SubScreen = SubScreen.NONE,

    // IMU Calibration Flow State
    val calibrationProgress: Float = 0.0f,
    val isAccelCalibrated: Boolean = false,
    val isGyroCalibrated: Boolean = false,
    val isMagCalibrated: Boolean = false,
    val isBaroCalibrated: Boolean = false,

    // Live Telemetry & Estimator States
    val telemetry: NavigationTelemetry? = null,
    val gnssStatus: GnssStatusState = GnssStatusState(),
    val navigationMode: NavigationMode = NavigationMode.INITIALIZING,
    val nativeStatus: NativeStatus = NativeStatus.OK,
    val outageDurationSec: Double = 0.0,
    val isDrActive: Boolean = false,
    val isGnssOutage: Boolean = false,
    val isGnssReacquired: Boolean = false,
    val lastTrustedFix: TrustedLocationFix? = null,
    val isAwaitingInitialFix: Boolean = false,

    // Road Map & 3D HMM State
    val mapDataState: MapDataState = MapDataState.READY,
    val roadMapGraph: RoadMapGraph? = null,
    val selectedMapFileName: String = "prayagraj_corridor.idrmap",
    val availableMapNames: List<String> = listOf("prayagraj_corridor.idrmap", "dense_metro.idrmap", "regional_network.idrmap"),
    val trajectoryMode: TrajectoryDisplayMode = TrajectoryDisplayMode.BOTH,
    val isNorthUp: Boolean = true,
    val is3dMode: Boolean = true,
    val mapZoomLevel: Float = 1.0f,
    val mapStyle: MapStyleType = MapStyleType.STANDARD,
    val idrTrajectory: List<TrajectoryPoint> = emptyList(),
    val gnssTrajectory: List<TrajectoryPoint> = emptyList(),
    val mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    val mapMatchedPositionEnu: org.isro.idr.engine.Vector3D? = null,
    val selectedRoadName: String = "",
    val nextTurnDistanceMeters: Int = 0,
    val nextTurnDirection: String = "",
    val selectedRoadLayer: String = "Ground",
    val mapConfidence: String = "HIGH",
    val isMapAmbiguous: Boolean = false,
    val candidateCount: Int = 0,
    val hypothesisCount: Int = 0,
    val scoreMargin: Double = 1.0,

    // Active End-to-End Trip Navigation State
    val isTripActive: Boolean = false,
    val startLocationName: String = "Current Location",
    val destinationLocationName: String = "",
    val destinationLatLon: Pair<Double, Double>? = null,
    val totalRouteDistanceMeters: Double = 0.0,
    val remainingDistanceMeters: Double = 0.0,
    val estimatedTimeMinutes: Int = 0,
    val currentManeuverIcon: String = "⮅",
    val currentManeuverInstruction: String = "Proceed to destination",
    val routeWaypoints: List<Pair<Double, Double>> = emptyList(),
    val deviceHeadingDeg: Double = 0.0,
    val isStationary: Boolean = true,

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
    val sensorCallbackRateHz: Double = 0.0,
    val sensorConsumedRateHz: Double = 0.0,
    val baroPressureHpa: Double = 1013.25,
    val baroAltitudeM: Double = 0.0,

    // Trip & Analytics State
    val tripStartTime: String = "--",
    val tripDurationSec: Long = 0L,
    val tripDistanceMeters: Double = 0.0,
    val tripAvgSpeedKmh: Double = 0.0,
    val tripMaxSpeedKmh: Double = 0.0,
    val tripDriftData: List<DriftPoint> = emptyList(),
    val tripEvents: List<TripEvent> = emptyList(),

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
    val nativeLatencyUs: Long = 0,
    val p95SensorJitterMs: Double = 0.0,
    val memoryUsageMb: Int = 42,
    val liveDriftPercentage: Double = 0.0,
    val outageDistanceMeters: Double = 0.0,
    val isOutageTestRunning: Boolean = false,
    val outageBenchmarkResult: String = "",

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

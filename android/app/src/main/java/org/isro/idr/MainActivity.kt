package org.isro.idr

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.BackHandler
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.Surface
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import org.isro.idr.engine.NativeNavigationEngine
import org.isro.idr.ingestion.GnssIngestionManager
import org.isro.idr.ingestion.NavigationPipelineSynchronizer
import org.isro.idr.ingestion.SensorIngestionManager
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrNavigationTheme
import org.isro.idr.ui.components.BottomNavBar
import org.isro.idr.ui.screens.DiagnosticsScreen
import org.isro.idr.ui.screens.GnssNavicScreen
import org.isro.idr.ui.screens.MainNavigationScreen
import org.isro.idr.ui.screens.MapExplorerScreen
import org.isro.idr.ui.screens.OfflineMapsScreen
import org.isro.idr.ui.screens.SensorCalibrationScreen
import org.isro.idr.ui.screens.SensorMonitorScreen
import org.isro.idr.ui.screens.SplashScreen
import org.isro.idr.ui.screens.SettingsScreen
import org.isro.idr.ui.screens.TripAnalyticsScreen
import org.isro.idr.ui.state.AppFlowState
import org.isro.idr.ui.state.ScreenTab
import org.isro.idr.ui.state.SubScreen
import org.isro.idr.ui.viewmodel.NavigationViewModel

class MainActivity : ComponentActivity() {

    private var engine: NativeNavigationEngine? = null
    private var sensorManager: SensorIngestionManager? = null
    private var gnssManager: GnssIngestionManager? = null
    private var synchronizer: NavigationPipelineSynchronizer? = null
    private var viewModel: NavigationViewModel? = null

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val fineLocationGranted = permissions[Manifest.permission.ACCESS_FINE_LOCATION] ?: false
        val coarseLocationGranted = permissions[Manifest.permission.ACCESS_COARSE_LOCATION] ?: false
        if (fineLocationGranted || coarseLocationGranted) {
            gnssManager?.start()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Keep screen on during navigation operations
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Initialize native navigation engine (100 Hz dt = 0.01s)
        engine = NativeNavigationEngine.create(0.01)

        // Initialize ingestion layer
        sensorManager = SensorIngestionManager(applicationContext)
        gnssManager = GnssIngestionManager(applicationContext)

        val localEngine = engine
        val localSensors = sensorManager
        val localGnss = gnssManager

        if (localEngine != null && localSensors != null && localGnss != null) {
            synchronizer = NavigationPipelineSynchronizer(
                engine = localEngine,
                sensorManager = localSensors,
                gnssManager = localGnss
            )
        }

        val vm = NavigationViewModel(
            synchronizer = synchronizer,
            gnssManager = gnssManager,
            engine = localEngine
        )
        viewModel = vm

        // Request location permissions and auto-start pipeline
        checkAndRequestPermissions()
        startNavigationPipeline()

        // Auto-load default offline road map dataset (Comprehensive Prayagraj corridor)
        vm.loadOfflineMap(applicationContext, "prayagraj_corridor.idrmap")

        setContent {
            IdrNavigationTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = IdrColors.Background
                ) {
                    val localVm = viewModel
                    if (localVm != null) {
                        val uiState by localVm.uiState.collectAsState()

                        when (uiState.appFlow) {
                            AppFlowState.SPLASH -> {
                                SplashScreen(
                                    onGetStarted = { localVm.startCalibrationFlow() }
                                )
                            }
                            AppFlowState.CALIBRATION -> {
                                SensorCalibrationScreen(
                                    uiState = uiState,
                                    onCalibrationComplete = { localVm.completeCalibration() },
                                    onCancel = { localVm.completeCalibration() }
                                )
                            }
                            AppFlowState.MAIN_APP -> {
                                MainApplicationShell(
                                    viewModel = localVm,
                                    onLoadMap = { localVm.loadOfflineMap(applicationContext, it) }
                                )
                            }
                        }
                    }
                }
            }
        }
    }

    private fun checkAndRequestPermissions() {
        val permissions = arrayOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        val missing = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) {
            permissionLauncher.launch(missing.toTypedArray())
        } else {
            gnssManager?.start()
        }
    }

    private fun startNavigationPipeline() {
        sensorManager?.start()
        gnssManager?.start()
        synchronizer?.start()
        viewModel?.setRunning(true)
    }

    private fun stopNavigationPipeline() {
        synchronizer?.stop()
        sensorManager?.stop()
        gnssManager?.stop()
        viewModel?.setRunning(false)
    }

    override fun onResume() {
        super.onResume()
        if (viewModel?.uiState?.value?.isRunning == true) {
            sensorManager?.start()
            gnssManager?.start()
            synchronizer?.start()
        }
    }

    override fun onPause() {
        super.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopNavigationPipeline()
        engine?.close()
    }
}

/**
 * Main Application Shell hosting 5 Bottom Destinations and Sub-Screens with Back Navigation.
 */
@Composable
private fun MainApplicationShell(
    viewModel: NavigationViewModel,
    onLoadMap: (String) -> Unit
) {
    val uiState by viewModel.uiState.collectAsState()

    // Handle Hardware Back Button for Sub-Screens
    if (uiState.activeSubScreen != SubScreen.NONE) {
        BackHandler {
            viewModel.closeSubScreen()
        }
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(IdrColors.Background)
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f)
        ) {
            // Render Active Sub-Screen or Destination Tab
            when (uiState.activeSubScreen) {
                SubScreen.SENSOR_MONITOR -> {
                    SensorMonitorScreen(
                        uiState = uiState,
                        onPlotTabSelected = { viewModel.setSensorPlotTab(it) }
                    )
                }
                SubScreen.GNSS_NAVIC_SKYVIEW -> {
                    GnssNavicScreen(
                        uiState = uiState
                    )
                }
                SubScreen.OFFLINE_MAPS_MANAGER -> {
                    OfflineMapsScreen(
                        uiState = uiState,
                        onDownloadRegion = { viewModel.downloadRegionPack(it) },
                        onActivateRegion = {
                            viewModel.activateRegionPack(it)
                            onLoadMap("${it}_corridor.idrmap")
                        }
                    )
                }
                SubScreen.NONE -> {
                    when (uiState.selectedScreen) {
                        ScreenTab.NAVIGATION -> {
                            MainNavigationScreen(
                                uiState = uiState,
                                onTrajectoryModeChanged = { viewModel.setTrajectoryMode(it) },
                                onZoomIn = { viewModel.zoomIn() },
                                onZoomOut = { viewModel.zoomOut() },
                                onToggleNorthUp = { viewModel.toggleNorthUp() },
                                onResetMap = { viewModel.resetMap() }
                            )
                        }
                        ScreenTab.MAP_EXPLORER -> {
                            MapExplorerScreen(
                                uiState = uiState,
                                onSearchQueryChanged = { viewModel.setExplorerSearchQuery(it) },
                                onStyleSelected = { viewModel.setMapStyle(it) },
                                onToggleRoads = { viewModel.toggleRoadsLayer(it) },
                                onToggle3dBuildings = { viewModel.toggle3dBuildingsLayer(it) },
                                onToggleLandmarks = { viewModel.toggleLandmarksLayer(it) },
                                onToggleWater = { viewModel.toggleRiversWaterLayer(it) },
                                onZoomIn = { viewModel.zoomIn() },
                                onZoomOut = { viewModel.zoomOut() },
                                onToggleNorthUp = { viewModel.toggleNorthUp() },
                                onResetMap = { viewModel.resetMap() }
                            )
                        }
                        ScreenTab.TRIP_ANALYTICS -> {
                            TripAnalyticsScreen(
                                uiState = uiState
                            )
                        }
                        ScreenTab.DIAGNOSTICS -> {
                            DiagnosticsScreen(
                                uiState = uiState
                            )
                        }
                        ScreenTab.SETTINGS -> {
                            SettingsScreen(
                                uiState = uiState,
                                onFollowVehicleToggled = { viewModel.toggleFollowVehicle(it) },
                                onHeadingUpToggled = { viewModel.toggleHeadingUp(it) },
                                on3dBuildingsToggled = { viewModel.toggle3dBuildingsSetting(it) },
                                onAutoRecenterToggled = { viewModel.toggleAutoRecenter(it) },
                                onOpenSubScreen = { viewModel.openSubScreen(it) }
                            )
                        }
                    }
                }
            }
        }

        // Bottom Navigation Bar (Visible on all 5 main destination tabs)
        if (uiState.activeSubScreen == SubScreen.NONE) {
            BottomNavBar(
                selectedTab = uiState.selectedScreen,
                onTabSelected = { viewModel.setSelectedScreen(it) }
            )
        }
    }
}

package org.isro.idr

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Surface
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
import org.isro.idr.ui.screens.MainNavigationScreen
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

        // Keep screen on during navigation experiments
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
                        MainNavigationScreen(
                            uiState = uiState,
                            onTrajectoryModeChanged = { localVm.setTrajectoryMode(it) },
                            onClearTrajectories = { localVm.clearTrajectories() },
                            onTabSelected = { localVm.setSelectedScreen(it) },
                            onToggleRunning = {
                                if (uiState.isRunning) {
                                    stopNavigationPipeline()
                                } else {
                                    startNavigationPipeline()
                                }
                            },
                            onZoomIn = { localVm.zoomIn() },
                            onZoomOut = { localVm.zoomOut() },
                            onToggleNorthUp = { localVm.toggleNorthUp() },
                            onResetMap = { localVm.resetMap() },
                            onMapSelected = { localVm.loadOfflineMap(applicationContext, it) }
                        )
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
        engine = null
    }
}

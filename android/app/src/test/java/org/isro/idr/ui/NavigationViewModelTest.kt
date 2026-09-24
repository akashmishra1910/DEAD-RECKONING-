package org.isro.idr.ui

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.UnconfinedTestDispatcher
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.setMain
import org.isro.idr.engine.GnssSample
import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.engine.Vector3D
import org.isro.idr.ui.state.HealthStatus
import org.isro.idr.ui.state.ScreenTab
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.viewmodel.NavigationViewModel
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class NavigationViewModelTest {

    private val testDispatcher = UnconfinedTestDispatcher()
    private lateinit var viewModel: NavigationViewModel

    @Before
    fun setUp() {
        Dispatchers.setMain(testDispatcher)
        viewModel = NavigationViewModel()
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun testInitialUiState() {
        val state = viewModel.uiState.value
        assertEquals(NavigationMode.INITIALIZING, state.navigationMode)
        assertEquals(NativeStatus.OK, state.nativeStatus)
        assertEquals(0.0, state.outageDurationSec, 1e-6)
        assertFalse(state.isDrActive)
        assertEquals(TrajectoryDisplayMode.BOTH, state.trajectoryMode)
        assertEquals(ScreenTab.NAVIGATION, state.selectedScreen)
        assertTrue(state.idrTrajectory.isEmpty())
        assertTrue(state.gnssTrajectory.isEmpty())
    }

    @Test
    fun testTelemetryConsumptionAndThrottling() {
        val telemetry = NavigationTelemetry(
            mode = NavigationMode.GNSS_AIDED,
            inOutage = false,
            timeSeconds = 10.0,
            posEnu = Vector3D(15.0, 25.0, 1.2),
            velEnu = Vector3D(5.0, 0.0, 0.0),
            speedKmh = 18.0,
            headingDeg = 45.0,
            pitchDeg = -1.0,
            rollDeg = 0.5,
            accelBias = Vector3D(),
            gyroBias = Vector3D(),
            outageDurationSeconds = 0.0,
            outageDistanceTraveledMeters = 0.0,
            gnssValid = true,
            isPotholeActive = false,
            isIdling = false,
            pTrace = 0.05
        )

        viewModel.onTelemetryReceived(telemetry)

        val state = viewModel.uiState.value
        assertNotNull(state.telemetry)
        assertEquals(15.0, state.telemetry!!.posX, 1e-6)
        assertEquals(25.0, state.telemetry!!.posY, 1e-6)
        assertEquals(NavigationMode.GNSS_AIDED, state.navigationMode)
        assertEquals(HealthStatus.HEALTHY, state.health.navStatus)
        assertEquals(HealthStatus.HEALTHY, state.health.gnssStatus)
        assertEquals(1, state.idrTrajectory.size)
    }

    @Test
    fun testDrOutageTimerTracking() {
        val drTelemetry1 = NavigationTelemetry(
            mode = NavigationMode.DR_ONLY,
            inOutage = true,
            timeSeconds = 100.0,
            posEnu = Vector3D(50.0, 50.0, 0.0),
            velEnu = Vector3D(10.0, 0.0, 0.0),
            speedKmh = 36.0,
            headingDeg = 90.0,
            pitchDeg = 0.0,
            rollDeg = 0.0,
            accelBias = Vector3D(),
            gyroBias = Vector3D(),
            outageDurationSeconds = 0.0,
            outageDistanceTraveledMeters = 0.0,
            gnssValid = false,
            isPotholeActive = false,
            isIdling = false,
            pTrace = 0.12
        )

        viewModel.onTelemetryReceived(drTelemetry1)

        val state1 = viewModel.uiState.value
        assertTrue(state1.isDrActive)
        assertEquals(0.0, state1.outageDurationSec, 1e-6)

        // Advance time by 5 seconds
        Thread.sleep(120) // Exceed 100ms throttle interval
        val drTelemetry2 = drTelemetry1.copy(
            timeSeconds = 105.0,
            posEnu = Vector3D(100.0, 50.0, 0.0)
        )
        viewModel.onTelemetryReceived(drTelemetry2)

        val state2 = viewModel.uiState.value
        assertTrue(state2.isDrActive)
        assertEquals(5.0, state2.outageDurationSec, 1e-6)
    }

    @Test
    fun testTrajectoryModeToggleAndClear() {
        viewModel.setTrajectoryMode(TrajectoryDisplayMode.IDR_ONLY)
        assertEquals(TrajectoryDisplayMode.IDR_ONLY, viewModel.uiState.value.trajectoryMode)

        viewModel.setSelectedScreen(ScreenTab.DIAGNOSTICS)
        assertEquals(ScreenTab.DIAGNOSTICS, viewModel.uiState.value.selectedScreen)

        viewModel.clearTrajectories()
        assertTrue(viewModel.uiState.value.idrTrajectory.isEmpty())
        assertTrue(viewModel.uiState.value.gnssTrajectory.isEmpty())
    }

    @Test
    fun testGnssSampleAccumulation() {
        val sample = GnssSample(
            timestampNanos = 1_000_000_000L,
            timeSeconds = 1.0,
            isValid = true,
            posEnu = Vector3D(12.5, 34.2, 0.5),
            velEnu = Vector3D(5.0, 0.0, 0.0),
            latitudeDeg = 25.4358,
            longitudeDeg = 81.8463,
            altitudeM = 98.0,
            groundSpeedMps = 14.2,
            courseDeg = 68.0,
            horizontalAccuracyMeters = 2.4f,
            navicSatelliteCount = 4,
            totalSatelliteCount = 18
        )

        viewModel.onGnssSample(sample)

        val state = viewModel.uiState.value
        assertTrue(state.gnssStatus.fixValid)
        assertEquals(18, state.gnssStatus.totalSatellites)
        assertEquals(4, state.gnssStatus.navicSatellitesUsed)
        assertEquals(1, state.gnssTrajectory.size)
        assertEquals(12.5, state.gnssTrajectory.first().enuX, 1e-6)
    }
}


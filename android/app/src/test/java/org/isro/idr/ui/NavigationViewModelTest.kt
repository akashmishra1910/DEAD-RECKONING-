package org.isro.idr.ui

import org.isro.idr.engine.NativeStatus
import org.isro.idr.engine.NavigationMode
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GnssSample
import org.isro.idr.ui.state.HealthStatus
import org.isro.idr.ui.state.ScreenTab
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.viewmodel.NavigationViewModel
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class NavigationViewModelTest {

    private lateinit var viewModel: NavigationViewModel

    @Before
    fun setUp() {
        viewModel = NavigationViewModel()
    }

    @Test
    fun testInitialUiState() {
        val state = viewModel.uiState.value
        assertEquals(NavigationMode.INITIALIZING, state.navigationMode)
        assertEquals(NativeStatus.SUCCESS, state.nativeStatus)
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
            timestampSec = 10.0,
            posX = 15.0,
            posY = 25.0,
            posZ = 1.2,
            velX = 5.0,
            velY = 0.0,
            velZ = 0.0,
            rollDeg = 0.5,
            pitchDeg = -1.0,
            azimuthDeg = 45.0,
            mode = NavigationMode.GNSS_NAV,
            status = NativeStatus.SUCCESS.code,
            gnssValid = true,
            navicSatellitesInView = 4,
            navicSatellitesUsedInFix = 2
        )

        viewModel.onTelemetryReceived(telemetry)

        val state = viewModel.uiState.value
        assertNotNull(state.telemetry)
        assertEquals(15.0, state.telemetry!!.posX, 1e-6)
        assertEquals(25.0, state.telemetry!!.posY, 1e-6)
        assertEquals(NavigationMode.GNSS_NAV, state.navigationMode)
        assertEquals(HealthStatus.HEALTHY, state.health.navStatus)
        assertEquals(HealthStatus.HEALTHY, state.health.gnssStatus)
        assertEquals(HealthStatus.HEALTHY, state.health.navicStatus)
        assertEquals(1, state.idrTrajectory.size)
    }

    @Test
    fun testDrOutageTimerTracking() {
        val drTelemetry1 = NavigationTelemetry(
            timestampSec = 100.0,
            posX = 50.0,
            posY = 50.0,
            posZ = 0.0,
            velX = 10.0,
            velY = 0.0,
            velZ = 0.0,
            rollDeg = 0.0,
            pitchDeg = 0.0,
            azimuthDeg = 90.0,
            mode = NavigationMode.DR_NAV,
            status = NativeStatus.SUCCESS.code,
            gnssValid = false,
            navicSatellitesInView = 0,
            navicSatellitesUsedInFix = 0
        )

        viewModel.onTelemetryReceived(drTelemetry1)

        val state1 = viewModel.uiState.value
        assertTrue(state1.isDrActive)
        assertEquals(0.0, state1.outageDurationSec, 1e-6)

        // Advance time by 5 seconds
        Thread.sleep(120) // Exceed 100ms throttle interval
        val drTelemetry2 = drTelemetry1.copy(timestampSec = 105.0, posX = 100.0)
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
            timestampSec = 1.0,
            latitude = 25.4358,
            longitude = 81.8463,
            altitude = 98.0,
            enuX = 12.5,
            enuY = 34.2,
            enuZ = 0.5,
            speedMps = 14.2,
            bearingDeg = 68.0,
            accuracyM = 2.4,
            totalSatellites = 18,
            navicSatellitesInView = 6,
            navicSatellitesUsed = 4,
            isValid = true
        )

        viewModel.onGnssSample(sample)

        val state = viewModel.uiState.value
        assertTrue(state.gnssStatus.fixValid)
        assertEquals(18, state.gnssStatus.totalSatellites)
        assertEquals(6, state.gnssStatus.navicSatellitesInView)
        assertEquals(4, state.gnssStatus.navicSatellitesUsed)
        assertEquals(1, state.gnssTrajectory.size)
        assertEquals(12.5, state.gnssTrajectory.first().enuX, 1e-6)
    }
}

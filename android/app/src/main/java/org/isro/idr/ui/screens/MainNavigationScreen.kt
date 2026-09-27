package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import org.isro.idr.theme.IdrColors
import org.isro.idr.ui.components.IsroIdrTopBar
import org.isro.idr.ui.components.NextTurnCard
import org.isro.idr.ui.components.SpeedHeadingHud
import org.isro.idr.ui.components.TopNavBanner
import org.isro.idr.ui.map.RealGeographicMapCanvas
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.TrajectoryDisplayMode

/**
 * Screens 3, 4, 5: Production End-to-End Navigation Dashboard.
 * Dominant full-screen real geographic world map with live turn guidance,
 * physical phone heading orientation, and standstill drift lock.
 */
@Composable
fun MainNavigationScreen(
    uiState: NavigationUiState,
    onTrajectoryModeChanged: (TrajectoryDisplayMode) -> Unit,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    onToggleMapStyle: (() -> Unit)? = null,
    onToggle3dMode: (() -> Unit)? = null,
    onClearDrift: (() -> Unit)? = null,
    onOpenDiagnostics: (() -> Unit)? = null,
    onOpenNavicSkyView: (() -> Unit)? = null,
    onSimulateOutageToggle: (() -> Unit)? = null,
    onEndTrip: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val speedKmh = if (uiState.isStationary) 0.0 else (uiState.telemetry?.speedKmh ?: (uiState.gnssStatus.speedMps * 3.6)).coerceAtLeast(0.0)
    val headingDeg = if (uiState.deviceHeadingDeg > 0.0) uiState.deviceHeadingDeg else (uiState.telemetry?.headingDeg ?: uiState.gnssStatus.bearingDeg)
    val isSearching = uiState.gnssStatus.isSearching && !uiState.gnssStatus.fixValid
    val isDrMode = uiState.isDrActive || (!uiState.gnssStatus.fixValid && !isSearching)
    val accuracyOrError = if (isDrMode) {
        (uiState.outageDurationSec * 0.08 + 4.2).coerceIn(4.2, 35.0)
    } else {
        uiState.gnssStatus.accuracyM.coerceAtLeast(1.4)
    }

    Box(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
    ) {
        // 1. Full Viewport Real Worldwide Geographic Map Canvas
        RealGeographicMapCanvas(
            telemetry = uiState.telemetry,
            gnssStatus = uiState.gnssStatus,
            idrTrajectory = uiState.idrTrajectory,
            gnssTrajectory = uiState.gnssTrajectory,
            mapAidedTrajectory = uiState.mapAidedTrajectory,
            roadMapGraph = uiState.roadMapGraph,
            trajectoryMode = uiState.trajectoryMode,
            mapDataState = uiState.mapDataState,
            mapStyle = uiState.mapStyle,
            isNorthUp = uiState.isNorthUp,
            userZoomFactor = uiState.mapZoomLevel,
            routeWaypoints = uiState.routeWaypoints,
            deviceHeadingDeg = headingDeg,
            isStationary = uiState.isStationary,
            enuAnchor = uiState.enuAnchor,
            liveDriftPercentage = uiState.liveDriftPercentage,
            lastTrustedFix = uiState.lastTrustedFix,
            isAwaitingInitialFix = uiState.isAwaitingInitialFix,
            onZoomIn = onZoomIn,
            onZoomOut = onZoomOut,
            onToggleNorthUp = onToggleNorthUp,
            onResetMap = onResetMap,
            onToggleMapStyle = onToggleMapStyle,
            onToggle3dMode = onToggle3dMode,
            onClearDrift = onClearDrift,
            modifier = Modifier.fillMaxSize()
        )

        // 2. Top Header Stack: ISRO IDR Bar + Dynamic Navigation Status Banner
        Column(
            modifier = Modifier
                .align(Alignment.TopCenter)
                .fillMaxWidth()
                .padding(horizontal = 12.dp, vertical = 8.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            IsroIdrTopBar(
                uiState = uiState,
                onOpenDiagnostics = onOpenDiagnostics,
                onOpenNavicSkyView = onOpenNavicSkyView
            )

            TopNavBanner(
                isDrActive = isDrMode,
                isGnssReacquired = uiState.isGnssReacquired,
                gnssStatus = uiState.gnssStatus,
                outageDurationSec = uiState.outageDurationSec,
                onSimulateOutageToggle = onSimulateOutageToggle
            )
        }

        // 3. Bottom Dock: NextTurnCard stacked neatly above SpeedHeadingHud (Only visible during active trip)
        Column(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .padding(start = 12.dp, end = 12.dp, bottom = 8.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            // Dynamic Next Turn Guidance & Active Trip Card (Only during active navigation trip)
            if (uiState.isTripActive) {
                NextTurnCard(
                    distanceMeters = uiState.nextTurnDistanceMeters,
                    turnDirection = uiState.nextTurnDirection,
                    roadName = uiState.selectedRoadName,
                    turnIcon = uiState.currentManeuverIcon,
                    isTripActive = uiState.isTripActive,
                    remainingDistanceMeters = uiState.remainingDistanceMeters,
                    estimatedMinutes = uiState.estimatedTimeMinutes,
                    onEndTrip = onEndTrip
                )
            }

            // Bottom Instrument Strip
            SpeedHeadingHud(
                speedKmh = speedKmh,
                headingDeg = headingDeg,
                accuracyOrErrorM = accuracyOrError,
                isDrMode = isDrMode,
                mapConfidence = uiState.mapConfidence
            )
        }
    }
}

package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import org.isro.idr.theme.IdrColors
import org.isro.idr.ui.components.NextTurnCard
import org.isro.idr.ui.components.SpeedHeadingHud
import org.isro.idr.ui.components.TopNavBanner
import org.isro.idr.ui.map.TacticalGridMapCanvas
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.TrajectoryDisplayMode

/**
 * Screens 3, 4, 5: Production Navigation Screen.
 * Full-screen map dominant navigation supporting:
 * - Screen 3: GNSS Navigation (Green Top HUD)
 * - Screen 4: Dead Reckoning Navigation (Amber/Saffron Top HUD + DR Trail)
 * - Screen 5: GNSS Recovery (Cyan Top HUD + Track Reacquisition)
 */
@Composable
fun MainNavigationScreen(
    uiState: NavigationUiState,
    onTrajectoryModeChanged: (TrajectoryDisplayMode) -> Unit,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val speedKmh = (uiState.telemetry?.speedKmh ?: (uiState.gnssStatus.speedMps * 3.6)).coerceAtLeast(0.0)
    val headingDeg = uiState.telemetry?.headingDeg ?: uiState.gnssStatus.bearingDeg
    val isDrMode = uiState.isDrActive || !uiState.gnssStatus.fixValid
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
        // 1. Full Viewport Offline Geographic Vector Canvas
        TacticalGridMapCanvas(
            telemetry = uiState.telemetry,
            idrTrajectory = uiState.idrTrajectory,
            gnssTrajectory = uiState.gnssTrajectory,
            mapAidedTrajectory = uiState.mapAidedTrajectory,
            roadMapGraph = uiState.roadMapGraph,
            trajectoryMode = uiState.trajectoryMode,
            mapDataState = uiState.mapDataState,
            isNorthUp = uiState.isNorthUp,
            userZoomFactor = uiState.mapZoomLevel,
            onZoomIn = onZoomIn,
            onZoomOut = onZoomOut,
            onToggleNorthUp = onToggleNorthUp,
            onResetMap = onResetMap,
            modifier = Modifier.fillMaxSize()
        )

        // 2. Top Floating Navigation Status Banner
        Box(
            modifier = Modifier
                .align(Alignment.TopCenter)
                .fillMaxWidth()
                .padding(horizontal = 12.dp, vertical = 10.dp)
        ) {
            TopNavBanner(
                isDrActive = isDrMode,
                isGnssReacquired = uiState.isGnssReacquired,
                gnssStatus = uiState.gnssStatus,
                outageDurationSec = uiState.outageDurationSec
            )
        }

        // 3. Floating Next Turn Guidance Card (Bottom Left over Map)
        Box(
            modifier = Modifier
                .align(Alignment.BottomStart)
                .padding(start = 12.dp, bottom = 78.dp)
        ) {
            NextTurnCard(
                distanceMeters = uiState.nextTurnDistanceMeters,
                turnDirection = uiState.nextTurnDirection,
                roadName = uiState.selectedRoadName
            )
        }

        // 4. Bottom Instrument Strip (Speed, Heading, Accuracy / Est. Error, Map Confidence)
        Box(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .padding(horizontal = 12.dp, vertical = 8.dp)
        ) {
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

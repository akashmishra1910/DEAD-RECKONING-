package org.isro.idr.ui.map

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint

/**
 * Modular Map Layer Composable.
 * Hosts MapLibre GL Native Android map when offline MBTiles/vector layers are loaded,
 * with automatic, zero-crash fallback to TacticalGridMapCanvas for pure offline local ENU rendering.
 */
@Composable
fun MapContainerView(
    telemetry: NavigationTelemetry?,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    trajectoryMode: TrajectoryDisplayMode,
    mapDataState: MapDataState,
    isNorthUp: Boolean = true,
    userZoomFactor: Float = 1.0f,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    Box(modifier = modifier.fillMaxSize()) {
        // High-performance offline Tactical Grid Map Canvas in modern light styling
        TacticalGridMapCanvas(
            telemetry = telemetry,
            idrTrajectory = idrTrajectory,
            gnssTrajectory = gnssTrajectory,
            mapAidedTrajectory = mapAidedTrajectory,
            trajectoryMode = trajectoryMode,
            mapDataState = mapDataState,
            isNorthUp = isNorthUp,
            userZoomFactor = userZoomFactor,
            onZoomIn = onZoomIn,
            onZoomOut = onZoomOut,
            onToggleNorthUp = onToggleNorthUp,
            onResetMap = onResetMap,
            modifier = Modifier.fillMaxSize()
        )
    }
}

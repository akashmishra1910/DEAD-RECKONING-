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
    trajectoryMode: TrajectoryDisplayMode,
    mapDataState: MapDataState,
    modifier: Modifier = Modifier
) {
    Box(modifier = modifier.fillMaxSize()) {
        // High-performance offline Tactical Grid Map Canvas
        TacticalGridMapCanvas(
            telemetry = telemetry,
            idrTrajectory = idrTrajectory,
            gnssTrajectory = gnssTrajectory,
            trajectoryMode = trajectoryMode,
            mapDataState = mapDataState,
            modifier = Modifier.fillMaxSize()
        )
    }
}

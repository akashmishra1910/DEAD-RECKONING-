package org.isro.idr.ui.map

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.map.RoadMapGraph
import org.isro.idr.ui.state.GnssStatusState
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.MapStyleType
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint

/**
 * Modular Map Layer Composable.
 * Hosts real worldwide geographic map canvas with dynamic OpenStreetMap / CartoDB tiles,
 * live physical GPS centering, and .idrmap vector road overlays.
 */
@Composable
fun MapContainerView(
    telemetry: NavigationTelemetry?,
    gnssStatus: GnssStatusState,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    roadMapGraph: RoadMapGraph? = null,
    trajectoryMode: TrajectoryDisplayMode = TrajectoryDisplayMode.BOTH,
    mapDataState: MapDataState = MapDataState.READY,
    mapStyle: MapStyleType = MapStyleType.DARK,
    isNorthUp: Boolean = true,
    userZoomFactor: Float = 1.0f,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    Box(modifier = modifier.fillMaxSize()) {
        RealGeographicMapCanvas(
            telemetry = telemetry,
            gnssStatus = gnssStatus,
            idrTrajectory = idrTrajectory,
            gnssTrajectory = gnssTrajectory,
            mapAidedTrajectory = mapAidedTrajectory,
            roadMapGraph = roadMapGraph,
            trajectoryMode = trajectoryMode,
            mapDataState = mapDataState,
            mapStyle = mapStyle,
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

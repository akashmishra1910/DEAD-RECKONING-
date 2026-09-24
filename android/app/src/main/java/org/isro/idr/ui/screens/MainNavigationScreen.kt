package org.isro.idr.ui.screens

import android.content.res.Configuration
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.components.NavicStatusBadge
import org.isro.idr.ui.components.SystemHealthBar
import org.isro.idr.ui.components.TelemetryPanel
import org.isro.idr.ui.components.TopStatusBanner
import org.isro.idr.ui.components.TrajectoryToggleControls
import org.isro.idr.ui.map.MapContainerView
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.ScreenTab
import org.isro.idr.ui.state.TrajectoryDisplayMode

@Composable
fun MainNavigationScreen(
    uiState: NavigationUiState,
    onTrajectoryModeChanged: (TrajectoryDisplayMode) -> Unit,
    onClearTrajectories: () -> Unit,
    onTabSelected: (ScreenTab) -> Unit,
    onToggleRunning: () -> Unit,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    onMapSelected: ((String) -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val configuration = LocalConfiguration.current
    val isLandscape = configuration.orientation == Configuration.ORIENTATION_LANDSCAPE

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .padding(6.dp)
    ) {
        // App Header / Navigation Bar
        AppHeader(
            selectedTab = uiState.selectedScreen,
            isRunning = uiState.isRunning,
            onTabSelected = onTabSelected,
            onToggleRunning = onToggleRunning
        )

        Spacer(modifier = Modifier.height(4.dp))

        when (uiState.selectedScreen) {
            ScreenTab.NAVIGATION -> {
                if (isLandscape) {
                    LandscapeLayout(
                        uiState = uiState,
                        onTrajectoryModeChanged = onTrajectoryModeChanged,
                        onClearTrajectories = onClearTrajectories,
                        onZoomIn = onZoomIn,
                        onZoomOut = onZoomOut,
                        onToggleNorthUp = onToggleNorthUp,
                        onResetMap = onResetMap
                    )
                } else {
                    PortraitLayout(
                        uiState = uiState,
                        onTrajectoryModeChanged = onTrajectoryModeChanged,
                        onClearTrajectories = onClearTrajectories,
                        onZoomIn = onZoomIn,
                        onZoomOut = onZoomOut,
                        onToggleNorthUp = onToggleNorthUp,
                        onResetMap = onResetMap
                    )
                }
            }
            ScreenTab.DIAGNOSTICS -> {
                DiagnosticsScreen(
                    uiState = uiState,
                    modifier = Modifier.fillMaxSize()
                )
            }
            ScreenTab.MAP_DATA -> {
                MapDataScreen(
                    uiState = uiState,
                    onMapSelected = onMapSelected,
                    modifier = Modifier.fillMaxSize()
                )
            }
            ScreenTab.SETTINGS -> {
                SettingsScreen(
                    uiState = uiState,
                    modifier = Modifier.fillMaxSize()
                )
            }
        }
    }
}

@Composable
private fun LandscapeLayout(
    uiState: NavigationUiState,
    onTrajectoryModeChanged: (TrajectoryDisplayMode) -> Unit,
    onClearTrajectories: () -> Unit,
    onZoomIn: (() -> Unit)?,
    onZoomOut: (() -> Unit)?,
    onToggleNorthUp: (() -> Unit)?,
    onResetMap: (() -> Unit)?
) {
    Row(
        modifier = Modifier.fillMaxSize(),
        horizontalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        // Left Column: Dominant Map Area (65% width)
        Box(
            modifier = Modifier
                .weight(1.6f)
                .fillMaxHeight()
        ) {
            MapContainerView(
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
        }

        // Right Column: Instrument Telemetry Dashboard (35% width)
        Column(
            modifier = Modifier
                .weight(1.0f)
                .fillMaxHeight()
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            TopStatusBanner(
                mode = uiState.navigationMode,
                outageDurationSec = uiState.outageDurationSec,
                isGnssValid = uiState.telemetry?.gnssValid ?: uiState.gnssStatus.fixValid,
                isSearchingFix = uiState.gnssStatus.isSearching
            )

            TelemetryPanel(
                telemetry = uiState.telemetry,
                enuAnchor = uiState.enuAnchor,
                gnssAccuracyM = uiState.gnssStatus.accuracyM
            )

            NavicStatusBadge(
                totalSatellites = uiState.gnssStatus.totalSatellites,
                navicInView = uiState.gnssStatus.navicSatellitesInView,
                navicUsed = uiState.gnssStatus.navicSatellitesUsed
            )

            TrajectoryToggleControls(
                currentMode = uiState.trajectoryMode,
                idrPointCount = uiState.idrTrajectory.size,
                gnssPointCount = uiState.gnssTrajectory.size,
                mapAidedPointCount = uiState.mapAidedTrajectory.size,
                onModeSelected = onTrajectoryModeChanged,
                onClearClicked = onClearTrajectories
            )

            SystemHealthBar(health = uiState.health)
        }
    }
}

@Composable
private fun PortraitLayout(
    uiState: NavigationUiState,
    onTrajectoryModeChanged: (TrajectoryDisplayMode) -> Unit,
    onClearTrajectories: () -> Unit,
    onZoomIn: (() -> Unit)?,
    onZoomOut: (() -> Unit)?,
    onToggleNorthUp: (() -> Unit)?,
    onResetMap: (() -> Unit)?
) {
    Column(
        modifier = Modifier.fillMaxSize(),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        // 1. Top HUD Mode & Outage Banner
        TopStatusBanner(
            mode = uiState.navigationMode,
            outageDurationSec = uiState.outageDurationSec,
            isGnssValid = uiState.telemetry?.gnssValid ?: uiState.gnssStatus.fixValid,
            isSearchingFix = uiState.gnssStatus.isSearching
        )

        // 2. Dominant Geographic Map Canvas (Fills majority of screen)
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f)
        ) {
            MapContainerView(
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
        }

        // 3. Compact Instrument HUD: Speed, Heading, Confidence, Trajectory Toggles
        Column(
            modifier = Modifier.fillMaxWidth(),
            verticalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            TelemetryPanel(
                telemetry = uiState.telemetry,
                enuAnchor = uiState.enuAnchor,
                gnssAccuracyM = uiState.gnssStatus.accuracyM
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                Box(modifier = Modifier.weight(1f)) {
                    NavicStatusBadge(
                        totalSatellites = uiState.gnssStatus.totalSatellites,
                        navicInView = uiState.gnssStatus.navicSatellitesInView,
                        navicUsed = uiState.gnssStatus.navicSatellitesUsed
                    )
                }
                Box(modifier = Modifier.weight(1.3f)) {
                    TrajectoryToggleControls(
                        currentMode = uiState.trajectoryMode,
                        idrPointCount = uiState.idrTrajectory.size,
                        gnssPointCount = uiState.gnssTrajectory.size,
                        mapAidedPointCount = uiState.mapAidedTrajectory.size,
                        onModeSelected = onTrajectoryModeChanged,
                        onClearClicked = onClearTrajectories
                    )
                }
            }
        }
    }
}

@Composable
fun AppHeader(
    selectedTab: ScreenTab,
    isRunning: Boolean,
    onTabSelected: (ScreenTab) -> Unit,
    onToggleRunning: () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(2.dp, RoundedCornerShape(8.dp))
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
            .padding(horizontal = 8.dp, vertical = 5.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // App Title & Tabs
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                Text(
                    text = "ISRO IDR",
                    style = IdrTypography.headlineMedium.copy(fontSize = 14.sp),
                    color = IdrColors.SaffronPrimary,
                    fontWeight = FontWeight.Bold
                )

                Spacer(modifier = Modifier.width(4.dp))

                TabItem(
                    title = "NAV",
                    isSelected = selectedTab == ScreenTab.NAVIGATION,
                    onClick = { onTabSelected(ScreenTab.NAVIGATION) }
                )
                TabItem(
                    title = "DIAG",
                    isSelected = selectedTab == ScreenTab.DIAGNOSTICS,
                    onClick = { onTabSelected(ScreenTab.DIAGNOSTICS) }
                )
                TabItem(
                    title = "MAP",
                    isSelected = selectedTab == ScreenTab.MAP_DATA,
                    onClick = { onTabSelected(ScreenTab.MAP_DATA) }
                )
                TabItem(
                    title = "SETTINGS",
                    isSelected = selectedTab == ScreenTab.SETTINGS,
                    onClick = { onTabSelected(ScreenTab.SETTINGS) }
                )
            }

            // Engine Start / Stop Button
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(if (isRunning) IdrColors.StatusRedBg else IdrColors.StatusGreenBg)
                    .border(
                        1.dp,
                        if (isRunning) IdrColors.StatusRed.copy(alpha = 0.5f) else IdrColors.StatusGreen.copy(alpha = 0.5f),
                        RoundedCornerShape(6.dp)
                    )
                    .clickable { onToggleRunning() }
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = if (isRunning) "STOP" else "START",
                    color = if (isRunning) IdrColors.StatusRed else IdrColors.StatusGreenDark,
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

@Composable
fun TabItem(
    title: String,
    isSelected: Boolean,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(4.dp))
            .background(if (isSelected) IdrColors.SurfaceSubtle else Color.Transparent)
            .border(
                1.dp,
                if (isSelected) IdrColors.SaffronPrimary else Color.Transparent,
                RoundedCornerShape(4.dp)
            )
            .clickable { onClick() }
            .padding(horizontal = 6.dp, vertical = 3.dp)
    ) {
        Text(
            text = title,
            style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
            color = if (isSelected) IdrColors.NavyDarkest else IdrColors.TextSecondary,
            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal
        )
    }
}

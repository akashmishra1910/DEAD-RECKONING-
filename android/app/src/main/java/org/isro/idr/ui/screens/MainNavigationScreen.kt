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
    modifier: Modifier = Modifier
) {
    val configuration = LocalConfiguration.current
    val isLandscape = configuration.orientation == Configuration.ORIENTATION_LANDSCAPE

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.NavyDarkest)
            .padding(8.dp)
    ) {
        // App Header / Navigation Bar
        AppHeader(
            selectedTab = uiState.selectedScreen,
            isRunning = uiState.isRunning,
            onTabSelected = onTabSelected,
            onToggleRunning = onToggleRunning
        )

        Spacer(modifier = Modifier.height(6.dp))

        when (uiState.selectedScreen) {
            ScreenTab.NAVIGATION -> {
                if (isLandscape) {
                    LandscapeLayout(
                        uiState = uiState,
                        onTrajectoryModeChanged = onTrajectoryModeChanged,
                        onClearTrajectories = onClearTrajectories
                    )
                } else {
                    PortraitLayout(
                        uiState = uiState,
                        onTrajectoryModeChanged = onTrajectoryModeChanged,
                        onClearTrajectories = onClearTrajectories
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
                    mapDataState = uiState.mapDataState,
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
    onClearTrajectories: () -> Unit
) {
    Row(
        modifier = Modifier.fillMaxSize(),
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        // Left Column: Dominant Map Area (62% width)
        Box(
            modifier = Modifier
                .weight(1.6f)
                .fillMaxHeight()
        ) {
            MapContainerView(
                telemetry = uiState.telemetry,
                idrTrajectory = uiState.idrTrajectory,
                gnssTrajectory = uiState.gnssTrajectory,
                trajectoryMode = uiState.trajectoryMode,
                mapDataState = uiState.mapDataState,
                modifier = Modifier.fillMaxSize()
            )
        }

        // Right Column: Instrument Telemetry Dashboard (38% width)
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
                isGnssValid = uiState.telemetry?.gnssValid ?: false
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
    onClearTrajectories: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        TopStatusBanner(
            mode = uiState.navigationMode,
            outageDurationSec = uiState.outageDurationSec,
            isGnssValid = uiState.telemetry?.gnssValid ?: false
        )

        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(280.dp)
        ) {
            MapContainerView(
                telemetry = uiState.telemetry,
                idrTrajectory = uiState.idrTrajectory,
                gnssTrajectory = uiState.gnssTrajectory,
                trajectoryMode = uiState.trajectoryMode,
                mapDataState = uiState.mapDataState,
                modifier = Modifier.fillMaxSize()
            )
        }

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
            onModeSelected = onTrajectoryModeChanged,
            onClearClicked = onClearTrajectories
        )

        SystemHealthBar(health = uiState.health)
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
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
            .padding(horizontal = 10.dp, vertical = 6.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // App Title & Tabs
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Text(
                    text = "ISRO IDR",
                    style = IdrTypography.headlineMedium.copy(fontSize = 16.sp),
                    color = IdrColors.SaffronAccent,
                    fontWeight = FontWeight.Bold
                )

                Spacer(modifier = Modifier.width(8.dp))

                TabItem(
                    title = "NAV",
                    isSelected = selectedTab == ScreenTab.NAVIGATION,
                    onClick = { onTabSelected(ScreenTab.NAVIGATION) }
                )
                TabItem(
                    title = "DIAGNOSTICS",
                    isSelected = selectedTab == ScreenTab.DIAGNOSTICS,
                    onClick = { onTabSelected(ScreenTab.DIAGNOSTICS) }
                )
                TabItem(
                    title = "MAP DATA",
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
                    .clip(RoundedCornerShape(4.dp))
                    .background(if (isRunning) IdrColors.StatusRedDim else IdrColors.StatusGreenDim)
                    .border(
                        1.dp,
                        if (isRunning) IdrColors.StatusRed else IdrColors.StatusGreen,
                        RoundedCornerShape(4.dp)
                    )
                    .clickable { onToggleRunning() }
                    .padding(horizontal = 10.dp, vertical = 4.dp)
            ) {
                Text(
                    text = if (isRunning) "STOP ENGINE" else "START ENGINE",
                    color = if (isRunning) IdrColors.StatusRed else IdrColors.StatusGreen,
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
            .background(if (isSelected) IdrColors.NavyCard else Color.Transparent)
            .border(
                1.dp,
                if (isSelected) IdrColors.SaffronAccent else Color.Transparent,
                RoundedCornerShape(4.dp)
            )
            .clickable { onClick() }
            .padding(horizontal = 8.dp, vertical = 4.dp)
    ) {
        Text(
            text = title,
            style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
            color = if (isSelected) IdrColors.TextPrimary else IdrColors.TextTertiary,
            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal
        )
    }
}

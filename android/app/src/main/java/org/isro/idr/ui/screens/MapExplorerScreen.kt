package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.material3.TextFieldDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.map.TacticalGridMapCanvas
import org.isro.idr.ui.state.MapStyleType
import org.isro.idr.ui.state.NavigationUiState

/**
 * Screen 6: Map Explorer.
 * Interactive continuous global-to-street map explorer with search,
 * style selection (Standard, Satellite, Dark, Terrain) and layer toggles.
 */
@Composable
fun MapExplorerScreen(
    uiState: NavigationUiState,
    onSearchQueryChanged: (String) -> Unit,
    onStyleSelected: (MapStyleType) -> Unit,
    onToggleRoads: (Boolean) -> Unit,
    onToggle3dBuildings: (Boolean) -> Unit,
    onToggleLandmarks: (Boolean) -> Unit,
    onToggleWater: (Boolean) -> Unit,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    var isLayersSheetOpen by remember { mutableStateOf(false) }

    Box(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
    ) {
        // 1. Full Viewport Vector Map
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

        // 2. Top Global Search Bar
        Box(
            modifier = Modifier
                .align(Alignment.TopCenter)
                .fillMaxWidth()
                .padding(horizontal = 14.dp, vertical = 12.dp)
        ) {
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .shadow(12.dp, RoundedCornerShape(24.dp))
                    .clip(RoundedCornerShape(24.dp))
                    .background(IdrColors.SurfaceElevated.copy(alpha = 0.96f))
                    .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(24.dp))
                    .padding(horizontal = 14.dp, vertical = 4.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "🔍",
                    fontSize = 15.sp,
                    color = Color(0xFF94A3B8)
                )

                Spacer(modifier = Modifier.width(8.dp))

                TextField(
                    value = uiState.explorerSearchQuery,
                    onValueChange = onSearchQueryChanged,
                    placeholder = {
                        Text(
                            text = "Search places, roads, landmarks...",
                            style = IdrTypography.bodyMedium.copy(fontSize = 13.sp),
                            color = Color(0xFF64748B)
                        )
                    },
                    colors = TextFieldDefaults.colors(
                        focusedContainerColor = Color.Transparent,
                        unfocusedContainerColor = Color.Transparent,
                        focusedIndicatorColor = Color.Transparent,
                        unfocusedIndicatorColor = Color.Transparent,
                        focusedTextColor = Color.White,
                        unfocusedTextColor = Color.White
                    ),
                    modifier = Modifier.weight(1f)
                )

                Box(
                    modifier = Modifier
                        .size(32.dp)
                        .clip(CircleShape)
                        .background(Color(0xFF1688FF).copy(alpha = 0.2f))
                        .clickable { /* Voice input trigger */ },
                    contentAlignment = Alignment.Center
                ) {
                    Text(text = "🎤", fontSize = 13.sp)
                }
            }
        }

        // 3. Floating Right Map Action Controls (Layers, 2D/3D, Compass, Recenter)
        Column(
            modifier = Modifier
                .align(Alignment.CenterEnd)
                .padding(end = 12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            FloatingActionIcon(
                icon = "◫",
                isSelected = isLayersSheetOpen,
                onClick = { isLayersSheetOpen = !isLayersSheetOpen }
            )
            FloatingActionIcon(
                icon = if (uiState.is3dMode) "3D" else "2D",
                onClick = { /* Toggle 3D */ }
            )
            FloatingActionIcon(
                icon = if (uiState.isNorthUp) "N" else "H",
                onClick = { onToggleNorthUp?.invoke() }
            )
            FloatingActionIcon(
                icon = "⌖",
                onClick = { onResetMap?.invoke() }
            )
        }

        // 4. Bottom Map Layers Sheet (Appears when toggled)
        if (isLayersSheetOpen) {
            Box(
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .fillMaxWidth()
                    .padding(start = 12.dp, end = 12.dp, bottom = 12.dp)
                    .shadow(16.dp, RoundedCornerShape(18.dp))
                    .clip(RoundedCornerShape(18.dp))
                    .background(IdrColors.SurfaceElevated.copy(alpha = 0.97f))
                    .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(18.dp))
                    .padding(16.dp)
            ) {
                Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "Map Layers",
                            style = IdrTypography.titleMedium.copy(
                                fontSize = 15.sp,
                                fontWeight = FontWeight.Bold
                            ),
                            color = Color.White
                        )
                        Text(
                            text = "✕",
                            fontSize = 14.sp,
                            color = Color(0xFF94A3B8),
                            modifier = Modifier.clickable { isLayersSheetOpen = false }
                        )
                    }

                    // Base Map Style Selector
                    Text(
                        text = "BASE MAP",
                        style = IdrTypography.labelSmall.copy(
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold,
                            letterSpacing = 1.sp
                        ),
                        color = Color(0xFF64748B)
                    )

                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        MapStyleChip(
                            label = "Standard",
                            isSelected = uiState.mapStyle == MapStyleType.STANDARD,
                            onClick = { onStyleSelected(MapStyleType.STANDARD) },
                            modifier = Modifier.weight(1f)
                        )
                        MapStyleChip(
                            label = "Satellite",
                            isSelected = uiState.mapStyle == MapStyleType.SATELLITE,
                            onClick = { onStyleSelected(MapStyleType.SATELLITE) },
                            modifier = Modifier.weight(1f)
                        )
                        MapStyleChip(
                            label = "Dark",
                            isSelected = uiState.mapStyle == MapStyleType.DARK,
                            onClick = { onStyleSelected(MapStyleType.DARK) },
                            modifier = Modifier.weight(1f)
                        )
                        MapStyleChip(
                            label = "Terrain",
                            isSelected = uiState.mapStyle == MapStyleType.TERRAIN,
                            onClick = { onStyleSelected(MapStyleType.TERRAIN) },
                            modifier = Modifier.weight(1f)
                        )
                    }

                    // Layer Visibility Toggles
                    Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                        LayerToggleRow("Roads", uiState.showRoadsLayer, onToggleRoads)
                        LayerToggleRow("Buildings (3D)", uiState.show3dBuildingsLayer, onToggle3dBuildings)
                        LayerToggleRow("Landmarks & POIs", uiState.showLandmarksLayer, onToggleLandmarks)
                        LayerToggleRow("Rivers & Water", uiState.showRiversWaterLayer, onToggleWater)
                    }
                }
            }
        }
    }
}

@Composable
private fun FloatingActionIcon(
    icon: String,
    isSelected: Boolean = false,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .size(40.dp)
            .shadow(6.dp, CircleShape)
            .clip(CircleShape)
            .background(if (isSelected) Color(0xFF1688FF) else IrColorsSurfaceElevated)
            .border(1.dp, if (isSelected) Color(0xFF4DA6FF) else IdrColors.CardBorder, CircleShape)
            .clickable { onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = icon,
            fontSize = 14.sp,
            color = Color.White,
            fontWeight = FontWeight.Bold
        )
    }
}

private val IrColorsSurfaceElevated = Color(0xFF10263A)

@Composable
private fun MapStyleChip(
    label: String,
    isSelected: Boolean,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(if (isSelected) Color(0xFF1688FF).copy(alpha = 0.25f) else Color(0xFF0B1B2B))
            .border(1.dp, if (isSelected) Color(0xFF1688FF) else Color(0xFF17324A), RoundedCornerShape(8.dp))
            .clickable { onClick() }
            .padding(vertical = 8.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = label,
            style = IdrTypography.labelSmall.copy(
                fontSize = 11.sp,
                fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Medium
            ),
            color = if (isSelected) Color.White else Color(0xFF94A3B8)
        )
    }
}

@Composable
private fun LayerToggleRow(
    title: String,
    isChecked: Boolean,
    onCheckedChange: (Boolean) -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = title,
            style = IdrTypography.bodyMedium.copy(fontSize = 13.sp),
            color = Color.White
        )
        Switch(
            checked = isChecked,
            onCheckedChange = onCheckedChange,
            colors = SwitchDefaults.colors(
                checkedThumbColor = Color.White,
                checkedTrackColor = Color(0xFF1688FF),
                uncheckedThumbColor = Color(0xFF94A3B8),
                uncheckedTrackColor = Color(0xFF10263A)
            )
        )
    }
}

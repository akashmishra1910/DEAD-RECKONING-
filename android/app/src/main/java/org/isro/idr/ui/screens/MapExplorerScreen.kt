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
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
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
import org.isro.idr.ui.map.RealGeographicMapCanvas
import org.isro.idr.ui.state.MapStyleType
import org.isro.idr.ui.state.NavigationUiState

import org.isro.idr.map.OfflineSearchEngine
import org.isro.idr.map.SearchPoiResult

/**
 * Screen 6: Map Explorer & Offline-First Turn-by-Turn Route Trip Planner.
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
    onStartNavigationToPoi: ((SearchPoiResult) -> Unit)? = null,
    onPlanRoute: ((SearchPoiResult, SearchPoiResult?) -> Unit)? = null,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    var isLayersSheetOpen by remember { mutableStateOf(false) }
    var isRoutePlannerOpen by remember { mutableStateOf(false) }
    var searchQuery by remember { mutableStateOf("") }
    var selectedDestination by remember { mutableStateOf<SearchPoiResult?>(null) }
    var selectedStart by remember { mutableStateOf<SearchPoiResult?>(null) }
    var isSearchActive by remember { mutableStateOf(false) }

    val currentLat = if (uiState.gnssStatus.fixValid && uiState.gnssStatus.latitude != 0.0) uiState.gnssStatus.latitude else (uiState.roadMapGraph?.centerLat ?: 25.4331)
    val currentLon = if (uiState.gnssStatus.fixValid && uiState.gnssStatus.longitude != 0.0) uiState.gnssStatus.longitude else (uiState.roadMapGraph?.centerLon ?: 81.8148)

    val filteredPois = remember(searchQuery, uiState.roadMapGraph, currentLat, currentLon) {
        OfflineSearchEngine.search(searchQuery, currentLat, currentLon, uiState.roadMapGraph)
    }

    Box(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
    ) {
        // 1. Full Viewport Geographic Map Canvas
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
            targetLatLon = selectedDestination?.let { Pair(it.lat, it.lon) },
            routeWaypoints = uiState.routeWaypoints,
            deviceHeadingDeg = uiState.deviceHeadingDeg,
            isStationary = uiState.isStationary,
            onZoomIn = onZoomIn,
            onZoomOut = onZoomOut,
            onToggleNorthUp = onToggleNorthUp,
            onResetMap = {
                selectedDestination = null
                onResetMap?.invoke()
            },
            modifier = Modifier.fillMaxSize()
        )

        // 2. Top Interactive Search Bar & Route Planning Overlay
        Column(
            modifier = Modifier
                .align(Alignment.TopCenter)
                .fillMaxWidth()
                .padding(horizontal = 14.dp, vertical = 12.dp)
        ) {
            if (!isRoutePlannerOpen) {
                // Single Search Bar
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .shadow(12.dp, RoundedCornerShape(24.dp))
                        .clip(RoundedCornerShape(24.dp))
                        .background(Color(0xFF0D2238).copy(alpha = 0.96f))
                        .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(24.dp))
                        .padding(horizontal = 14.dp, vertical = 2.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(text = "🔍", fontSize = 15.sp, color = Color(0xFF94A3B8))
                    Spacer(modifier = Modifier.width(8.dp))
                    TextField(
                        value = searchQuery,
                        onValueChange = {
                            searchQuery = it
                            isSearchActive = true
                            onSearchQueryChanged(it)
                        },
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

                    if (searchQuery.isNotEmpty()) {
                        Text(
                            text = "✕",
                            fontSize = 14.sp,
                            color = Color(0xFF94A3B8),
                            modifier = Modifier
                                .clickable {
                                    searchQuery = ""
                                    isSearchActive = false
                                    selectedDestination = null
                                }
                                .padding(8.dp)
                        )
                    }

                    Box(
                        modifier = Modifier
                            .size(32.dp)
                            .clip(CircleShape)
                            .background(Color(0xFF1688FF).copy(alpha = 0.2f))
                            .clickable { isLayersSheetOpen = !isLayersSheetOpen },
                        contentAlignment = Alignment.Center
                    ) {
                        Text(text = "◫", fontSize = 14.sp, color = Color(0xFF1688FF), fontWeight = FontWeight.Bold)
                    }
                }
            } else {
                // Route Planner Box: Start + Destination fields (Google Maps Style)
                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .shadow(16.dp, RoundedCornerShape(16.dp))
                        .clip(RoundedCornerShape(16.dp))
                        .background(Color(0xFF0D2238).copy(alpha = 0.98f))
                        .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(16.dp))
                        .padding(14.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "Set Trip Route",
                            style = IdrTypography.titleMedium.copy(fontSize = 14.sp, fontWeight = FontWeight.Bold),
                            color = Color.White
                        )
                        Text(
                            text = "✕ Close",
                            fontSize = 12.sp,
                            color = Color(0xFF94A3B8),
                            modifier = Modifier.clickable { isRoutePlannerOpen = false }
                        )
                    }

                    // Start Field
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .clip(RoundedCornerShape(8.dp))
                            .background(Color(0xFF17324A))
                            .padding(horizontal = 10.dp, vertical = 8.dp),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(text = "🟢", fontSize = 10.sp)
                        Spacer(modifier = Modifier.width(8.dp))
                        Text(
                            text = selectedStart?.title ?: "Your Location (GPS Fix)",
                            fontSize = 12.sp,
                            color = Color(0xFFCBD5E1),
                            fontWeight = FontWeight.SemiBold
                        )
                    }

                    // Destination Field
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .clip(RoundedCornerShape(8.dp))
                            .background(Color(0xFF17324A))
                            .padding(horizontal = 10.dp, vertical = 8.dp),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(text = "🔴", fontSize = 10.sp)
                        Spacer(modifier = Modifier.width(8.dp))
                        Text(
                            text = selectedDestination?.title ?: "Select destination...",
                            fontSize = 12.sp,
                            color = if (selectedDestination != null) Color.White else Color(0xFF64748B),
                            fontWeight = FontWeight.SemiBold
                        )
                    }
                }
            }

            // Dropdown search suggestions when typing
            if (isSearchActive && filteredPois.isNotEmpty()) {
                Spacer(modifier = Modifier.height(6.dp))
                LazyColumn(
                    modifier = Modifier
                        .fillMaxWidth()
                        .shadow(16.dp, RoundedCornerShape(14.dp))
                        .clip(RoundedCornerShape(14.dp))
                        .background(Color(0xFF0D2238).copy(alpha = 0.98f))
                        .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
                        .padding(vertical = 4.dp)
                ) {
                    items(filteredPois) { poi ->
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable {
                                    selectedDestination = poi
                                    searchQuery = poi.title
                                    isSearchActive = false
                                    onStartNavigationToPoi?.invoke(poi)
                                }
                                .padding(horizontal = 14.dp, vertical = 10.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text(text = "📍", fontSize = 14.sp)
                            Spacer(modifier = Modifier.width(10.dp))
                            Column(modifier = Modifier.weight(1f)) {
                                Text(
                                    text = poi.title,
                                    fontSize = 13.sp,
                                    fontWeight = FontWeight.Bold,
                                    color = Color.White
                                )
                                Text(
                                    text = poi.subtitle,
                                    fontSize = 11.sp,
                                    color = Color(0xFF94A3B8)
                                )
                            }
                            Box(
                                modifier = Modifier
                                    .clip(RoundedCornerShape(4.dp))
                                    .background(Color(0xFF17324A))
                                    .padding(horizontal = 6.dp, vertical = 2.dp)
                            ) {
                                Text(
                                    text = poi.category,
                                    fontSize = 9.sp,
                                    color = Color(0xFFFFD54F)
                                )
                            }
                        }
                    }
                }
            }
        }

        // 3. Destination Bottom Card when a place is selected
        if (selectedDestination != null && !isLayersSheetOpen) {
            val poi = selectedDestination!!
            Box(
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .fillMaxWidth()
                    .padding(start = 14.dp, end = 14.dp, bottom = 14.dp)
                    .shadow(16.dp, RoundedCornerShape(16.dp))
                    .clip(RoundedCornerShape(16.dp))
                    .background(Color(0xFF0D2238).copy(alpha = 0.98f))
                    .border(1.2.dp, Color(0xFFFF8A00).copy(alpha = 0.8f), RoundedCornerShape(16.dp))
                    .padding(16.dp)
            ) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Column(modifier = Modifier.weight(1f)) {
                        Text(
                            text = poi.title,
                            fontSize = 15.sp,
                            fontWeight = FontWeight.ExtraBold,
                            color = Color.White
                        )
                        Text(
                            text = poi.subtitle,
                            fontSize = 11.sp,
                            color = Color(0xFFCBD5E1)
                        )
                        Spacer(modifier = Modifier.height(4.dp))
                        Text(
                            text = String.format("%.4f° N, %.4f° E", poi.lat, poi.lon),
                            fontSize = 10.sp,
                            color = Color(0xFFFFAB40)
                        )
                    }

                    Spacer(modifier = Modifier.width(12.dp))

                    Button(
                        onClick = { onStartNavigationToPoi?.invoke(poi) },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFFF8A00)),
                        shape = RoundedCornerShape(10.dp)
                    ) {
                        Text(
                            text = "START TRIP",
                            fontSize = 12.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color.White
                        )
                    }
                }
            }
        }

        // 4. Map Layers Sheet (Appears when layer button is clicked)
        if (isLayersSheetOpen) {
            Box(
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .fillMaxWidth()
                    .padding(start = 12.dp, end = 12.dp, bottom = 12.dp)
                    .shadow(16.dp, RoundedCornerShape(18.dp))
                    .clip(RoundedCornerShape(18.dp))
                    .background(Color(0xFF0D2238).copy(alpha = 0.98f))
                    .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(18.dp))
                    .padding(16.dp)
            ) {
                Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "Map Layers & Style",
                            style = IdrTypography.titleMedium.copy(fontSize = 15.sp, fontWeight = FontWeight.Bold),
                            color = Color.White
                        )
                        Text(
                            text = "✕",
                            fontSize = 14.sp,
                            color = Color(0xFF94A3B8),
                            modifier = Modifier.clickable { isLayersSheetOpen = false }
                        )
                    }

                    Text(
                        text = "BASE MAP STYLE",
                        style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.Bold),
                        color = Color(0xFF64748B)
                    )

                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        MapStyleType.values().forEach { style ->
                            val isSelected = uiState.mapStyle == style
                            Box(
                                modifier = Modifier
                                    .weight(1f)
                                    .clip(RoundedCornerShape(10.dp))
                                    .background(if (isSelected) Color(0xFFFF8A00) else Color(0xFF17324A))
                                    .clickable { onStyleSelected(style) }
                                    .padding(vertical = 8.dp),
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    text = style.label,
                                    fontSize = 11.sp,
                                    fontWeight = FontWeight.Bold,
                                    color = Color.White
                                )
                            }
                        }
                    }

                    LayerSwitchRow("Roads & Highways", uiState.showRoadsLayer, onToggleRoads)
                    LayerSwitchRow("3D Buildings", uiState.show3dBuildingsLayer, onToggle3dBuildings)
                    LayerSwitchRow("Rivers & Water", uiState.showRiversWaterLayer, onToggleWater)
                    LayerSwitchRow("Landmarks & POIs", uiState.showLandmarksLayer, onToggleLandmarks)
                }
            }
        }
    }
}

@Composable
private fun LayerSwitchRow(
    label: String,
    isChecked: Boolean,
    onCheckedChange: (Boolean) -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = label, fontSize = 12.sp, color = Color.White)
        Switch(
            checked = isChecked,
            onCheckedChange = onCheckedChange,
            colors = SwitchDefaults.colors(
                checkedThumbColor = Color.White,
                checkedTrackColor = Color(0xFFFF8A00),
                uncheckedThumbColor = Color(0xFF64748B),
                uncheckedTrackColor = Color(0xFF17324A)
            )
        )
    }
}

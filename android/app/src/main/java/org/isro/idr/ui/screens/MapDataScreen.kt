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
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState

@Composable
fun MapDataScreen(
    uiState: NavigationUiState,
    onMapSelected: ((String) -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val graph = uiState.roadMapGraph

    Column(
        modifier = modifier
            .fillMaxSize()
            .shadow(2.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(16.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Text(
            text = "OFFLINE GEOGRAPHIC ROAD GRAPH MANAGER",
            style = IdrTypography.headlineMedium,
            color = IdrColors.SaffronPrimary,
            fontWeight = FontWeight.Bold
        )

        // Active Map Status Card
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(8.dp))
                .background(IdrColors.SurfaceSubtle)
                .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
                .padding(14.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "ACTIVE ROAD GRAPH:",
                        style = IdrTypography.labelSmall,
                        color = IdrColors.TextSecondary
                    )
                    Text(
                        text = if (graph != null) "${graph.mapName} (ACTIVE)" else "OFFLINE TACTICAL",
                        style = IdrTypography.titleMedium,
                        color = if (graph != null) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
                        fontWeight = FontWeight.Bold
                    )
                }

                if (graph != null) {
                    Text(
                        text = "• Road Edges: ${graph.edgeCount}  |  Topological Nodes: ${graph.nodeCount}",
                        style = IdrTypography.bodyMedium,
                        color = IdrColors.TextPrimary,
                        fontWeight = FontWeight.Bold
                    )
                    Text(
                        text = "• Bounding Box: [${String.format("%.5f", graph.minLat)}, ${String.format("%.5f", graph.minLon)}] to [${String.format("%.5f", graph.maxLat)}, ${String.format("%.5f", graph.maxLon)}]",
                        style = IdrTypography.bodyMedium,
                        color = IdrColors.TextSecondary
                    )
                    Text(
                        text = "• Center Anchor: ${String.format("%.5f° N, %.5f° E", graph.centerLat, graph.centerLon)}",
                        style = IdrTypography.bodyMedium,
                        color = IdrColors.TextSecondary
                    )
                } else {
                    Text(
                        text = "No .idrmap binary loaded. Tactical local ENU grid is rendering for dead reckoning.",
                        style = IdrTypography.bodyMedium,
                        color = IdrColors.TextSecondary
                    )
                }
            }
        }

        // Available Bundled Offline Datasets
        Text(
            text = "BUNDLED OFFLINE DATASETS (.idrmap v1.0)",
            style = IdrTypography.labelSmall,
            color = IdrColors.TextSecondary,
            fontWeight = FontWeight.Bold
        )

        val mapFiles = listOf(
            Triple("prayagraj_corridor.idrmap", "Prayagraj Urban & River Corridor (ISRO Trial)", "Extensive road network, Ganga & Yamuna rivers, Shastri & New Yamuna Bridges"),
            Triple("sample_prayagraj.idrmap", "Prayagraj Small Corridor (Unit Test Fixture)", "12 nodes, 8 edges, bridge/flyover validation track"),
            Triple("dense_metro.idrmap", "Delhi Central Metro Network", "900 nodes, 1,740 edges, multi-layer junctions"),
            Triple("regional_network.idrmap", "Regional Highway Corridor", "High-speed multi-lane dual carriageway network")
        )

        for ((fileName, title, desc) in mapFiles) {
            val isSelected = (uiState.selectedMapFileName == fileName)
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(8.dp))
                    .background(if (isSelected) IdrColors.SaffronDim else IdrColors.SurfaceSubtle)
                    .border(
                        1.dp,
                        if (isSelected) IdrColors.SaffronPrimary else IdrColors.CardBorder,
                        RoundedCornerShape(8.dp)
                    )
                    .clickable { onMapSelected?.invoke(fileName) }
                    .padding(12.dp)
            ) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column(modifier = Modifier.weight(1f)) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            if (isSelected) {
                                Box(
                                    modifier = Modifier
                                        .size(6.dp)
                                        .clip(CircleShape)
                                        .background(IdrColors.SaffronPrimary)
                                )
                                Spacer(modifier = Modifier.width(6.dp))
                            }
                            Text(
                                text = title,
                                style = IdrTypography.titleMedium.copy(fontSize = 13.sp),
                                color = if (isSelected) IdrColors.SaffronDark else IdrColors.TextPrimary,
                                fontWeight = FontWeight.Bold
                            )
                        }
                        Spacer(modifier = Modifier.height(2.dp))
                        Text(
                            text = "$fileName • $desc",
                            style = IdrTypography.bodyMedium.copy(fontSize = 11.sp),
                            color = IdrColors.TextSecondary
                        )
                    }

                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(4.dp))
                            .background(if (isSelected) IdrColors.SaffronPrimary else IdrColors.SurfaceElevated)
                            .border(1.dp, if (isSelected) IdrColors.SaffronPrimary else IdrColors.CardBorder, RoundedCornerShape(4.dp))
                            .padding(horizontal = 8.dp, vertical = 4.dp)
                    ) {
                        Text(
                            text = if (isSelected) "ACTIVE" else "LOAD",
                            style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                            color = if (isSelected) IdrColors.SurfaceElevated else IdrColors.TextSecondary,
                            fontWeight = FontWeight.Bold
                        )
                    }
                }
            }
        }

        // Attribution & Provenance
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(8.dp))
                .background(IdrColors.SurfaceSubtle)
                .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
                .padding(12.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text(
                    text = "DATA PROVENANCE & LICENSE",
                    style = IdrTypography.labelSmall,
                    color = IdrColors.TextSecondary
                )
                Text(
                    text = "Map data © OpenStreetMap contributors, licensed under the Open Database License (ODbL 1.0). Optimized for ISRO IDR onboard inference.",
                    style = IdrTypography.bodyMedium.copy(fontSize = 11.sp),
                    color = IdrColors.TextTertiary
                )
            }
        }
    }
}

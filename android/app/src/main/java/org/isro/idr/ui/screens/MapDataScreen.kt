package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.MapDataState

@Composable
fun MapDataScreen(
    mapDataState: MapDataState,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Text(
            text = "OFFLINE OPENSTREETMAP (OSM) DATASET MANAGER",
            style = IdrTypography.headlineMedium,
            color = IdrColors.SaffronAccent,
            fontWeight = FontWeight.Bold
        )

        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(6.dp))
                .background(IdrColors.NavyCard)
                .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(6.dp))
                .padding(12.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text(
                    text = "CURRENT MAP STATUS: ${mapDataState.name}",
                    style = IdrTypography.titleMedium,
                    color = if (mapDataState == MapDataState.AVAILABLE) IdrColors.StatusGreen else IdrColors.StatusAmber,
                    fontWeight = FontWeight.Bold
                )
                Text(
                    text = "Offline road graph extraction and MBTiles packaging pipeline are scheduled for Phase 11.5 (OSM Extraction Pipeline) and Phase 11.6 (Compact Road Graph & Spatial Index).",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextSecondary
                )
                Spacer(modifier = Modifier.height(6.dp))
                Text(
                    text = "Tactical local ENU grid rendering is active for live dead reckoning without requiring external map downloads.",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary
                )
            }
        }
    }
}

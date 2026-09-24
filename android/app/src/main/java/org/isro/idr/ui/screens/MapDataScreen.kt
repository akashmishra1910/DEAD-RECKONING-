package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
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
import org.isro.idr.ui.state.MapDataState

@Composable
fun MapDataScreen(
    mapDataState: MapDataState,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .shadow(2.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Text(
            text = "OFFLINE MAP & TOPOLOGICAL ROAD GRAPH",
            style = IdrTypography.headlineMedium,
            color = IdrColors.SaffronPrimary,
            fontWeight = FontWeight.Bold
        )

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
                        text = "MAP ENGINE STATUS:",
                        style = IdrTypography.labelSmall,
                        color = IdrColors.TextSecondary
                    )
                    Text(
                        text = mapDataState.name,
                        style = IdrTypography.titleMedium,
                        color = if (mapDataState == MapDataState.AVAILABLE || mapDataState == MapDataState.READY) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
                        fontWeight = FontWeight.Bold
                    )
                }

                Spacer(modifier = Modifier.height(2.dp))

                Text(
                    text = "Topological Graph Engine (.idrmap v1.0):",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary,
                    fontWeight = FontWeight.Bold
                )
                Text(
                    text = "• Binary memory-mapped (.idrmap v1.0) spatial road index with 3D elevation barrier support.\n• Causal 3D HMM Viterbi decoder with topological emission & transition probabilities.\n• 100% offline deterministic execution with zero runtime network roundtrips.",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextSecondary
                )

                Spacer(modifier = Modifier.height(4.dp))

                Text(
                    text = "Tactical Visual Canvas:",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary,
                    fontWeight = FontWeight.Bold
                )
                Text(
                    text = "• High-contrast vector ENU canvas with live heading-up / north-up rotation.\n• Multi-trajectory visual comparison: GNSS (Blue) vs IDR Filter (Saffron) vs Map-Aided (Purple).\n• Smooth uninterrupted navigation tracking during complete GNSS denial.",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextSecondary
                )
            }
        }
    }
}

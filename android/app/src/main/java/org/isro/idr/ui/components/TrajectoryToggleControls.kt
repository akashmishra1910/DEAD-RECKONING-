package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
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
import org.isro.idr.ui.state.TrajectoryDisplayMode

@Composable
fun TrajectoryToggleControls(
    currentMode: TrajectoryDisplayMode,
    idrPointCount: Int,
    gnssPointCount: Int,
    mapAidedPointCount: Int = 0,
    onModeSelected: (TrajectoryDisplayMode) -> Unit,
    onClearClicked: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(2.dp, RoundedCornerShape(10.dp))
            .clip(RoundedCornerShape(10.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(10.dp))
            .padding(horizontal = 8.dp, vertical = 6.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Trajectory Mode Segmented Chips
            Row(
                horizontalArrangement = Arrangement.spacedBy(4.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "LAYERS:",
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = IdrColors.TextSecondary,
                    modifier = Modifier.padding(end = 2.dp)
                )

                TrajectoryChip(
                    label = "ALL",
                    isSelected = currentMode == TrajectoryDisplayMode.BOTH,
                    accentColor = IdrColors.NavyDarkest,
                    onClick = { onModeSelected(TrajectoryDisplayMode.BOTH) }
                )

                TrajectoryChip(
                    label = "DR ($idrPointCount)",
                    isSelected = currentMode == TrajectoryDisplayMode.IDR_ONLY,
                    accentColor = IdrColors.TrajectoryIdr,
                    onClick = { onModeSelected(TrajectoryDisplayMode.IDR_ONLY) }
                )

                TrajectoryChip(
                    label = "GNSS ($gnssPointCount)",
                    isSelected = currentMode == TrajectoryDisplayMode.GNSS_ONLY,
                    accentColor = IdrColors.TrajectoryGnss,
                    onClick = { onModeSelected(TrajectoryDisplayMode.GNSS_ONLY) }
                )

                TrajectoryChip(
                    label = "MAP ($mapAidedPointCount)",
                    isSelected = currentMode == TrajectoryDisplayMode.MAP_AIDED,
                    accentColor = IdrColors.TrajectoryMap,
                    onClick = { onModeSelected(TrajectoryDisplayMode.MAP_AIDED) }
                )

                TrajectoryChip(
                    label = "OFF",
                    isSelected = currentMode == TrajectoryDisplayMode.OFF,
                    accentColor = IdrColors.SlateMuted,
                    onClick = { onModeSelected(TrajectoryDisplayMode.OFF) }
                )
            }

            // Reset / Clear Button
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(IdrColors.StatusRedBg)
                    .border(1.dp, IdrColors.StatusRed.copy(alpha = 0.4f), RoundedCornerShape(6.dp))
                    .clickable { onClearClicked() }
                    .padding(horizontal = 7.dp, vertical = 4.dp)
            ) {
                Text(
                    text = "CLEAR",
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = IdrColors.StatusRed,
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

@Composable
fun TrajectoryChip(
    label: String,
    isSelected: Boolean,
    accentColor: Color,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(6.dp))
            .background(if (isSelected) IdrColors.SurfaceSubtle else Color.Transparent)
            .border(
                1.dp,
                if (isSelected) accentColor else Color.Transparent,
                RoundedCornerShape(6.dp)
            )
            .clickable { onClick() }
            .padding(horizontal = 6.dp, vertical = 4.dp)
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            if (isSelected) {
                Box(
                    modifier = Modifier
                        .size(5.dp)
                        .clip(CircleShape)
                        .background(accentColor)
                )
                Spacer(modifier = Modifier.width(3.dp))
            }
            Text(
                text = label,
                style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                color = if (isSelected) IdrColors.TextPrimary else IdrColors.TextSecondary,
                fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal
            )
        }
    }
}

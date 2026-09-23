package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationMode
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography

@Composable
fun TopStatusBanner(
    mode: NavigationMode,
    outageDurationSec: Double,
    isGnssValid: Boolean,
    modifier: Modifier = Modifier
) {
    val (modeText, badgeColor, badgeBg) = when (mode) {
        NavigationMode.GNSS_NAV -> Triple("GNSS NAVIGATION", IdrColors.StatusGreen, IdrColors.StatusGreenDim)
        NavigationMode.TRANSITION_TO_DR -> Triple("TRANSITION TO DR", IdrColors.StatusAmber, IdrColors.StatusAmberDim)
        NavigationMode.DR_NAV -> Triple("DEAD RECKONING ACTIVE", IdrColors.StatusAmber, IdrColors.StatusAmberDim)
        NavigationMode.MAP_AIDED_DR -> Triple("MAP-AIDED DR", IdrColors.StatusAmber, IdrColors.StatusAmberDim)
        NavigationMode.GNSS_RECOVERY -> Triple("GNSS RECOVERY", IdrColors.CyanAccent, IdrColors.NavyDark)
        NavigationMode.RECOVERED -> Triple("GNSS RECOVERED", IdrColors.StatusGreen, IdrColors.StatusGreenDim)
        NavigationMode.INITIALIZING -> Triple("INITIALIZING SENSORS", IdrColors.StatusBlue, IdrColors.NavyDark)
    }

    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
            .padding(horizontal = 12.dp, vertical = 8.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Left: Navigation Mode Pill
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(4.dp))
                        .background(badgeBg)
                        .border(1.dp, badgeColor, RoundedCornerShape(4.dp))
                        .padding(horizontal = 8.dp, vertical = 4.dp)
                ) {
                    Text(
                        text = modeText,
                        color = badgeColor,
                        style = IdrTypography.titleMedium,
                        fontWeight = FontWeight.Bold
                    )
                }

                if (outageDurationSec > 0.0) {
                    Spacer(modifier = Modifier.width(12.dp))
                    val minutes = (outageDurationSec / 60).toInt()
                    val seconds = outageDurationSec % 60
                    val timerStr = String.format("OUTAGE: %02d:%04.1fs", minutes, seconds)
                    Text(
                        text = timerStr,
                        color = IdrColors.StatusAmber,
                        style = IdrTypography.bodyMedium,
                        fontWeight = FontWeight.Bold
                    )
                }
            }

            // Right: GNSS Fix Status Badge
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(if (isGnssValid) IdrColors.StatusGreenDim else IdrColors.StatusAmberDim)
                    .border(
                        1.dp,
                        if (isGnssValid) IdrColors.StatusGreen else IdrColors.StatusAmber,
                        RoundedCornerShape(4.dp)
                    )
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = if (isGnssValid) "GNSS 3D FIX" else "GNSS OUTAGE",
                    color = if (isGnssValid) IdrColors.StatusGreen else IdrColors.StatusAmber,
                    style = IdrTypography.labelSmall,
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

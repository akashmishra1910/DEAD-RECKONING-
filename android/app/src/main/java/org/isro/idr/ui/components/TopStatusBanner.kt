package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
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
import org.isro.idr.engine.NavigationMode
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography

private data class BannerModeStyle(
    val modeText: String,
    val textColor: Color,
    val bgColor: Color,
    val dotColor: Color
)

@Composable
fun TopStatusBanner(
    mode: NavigationMode,
    outageDurationSec: Double,
    isGnssValid: Boolean,
    isSearchingFix: Boolean = false,
    modifier: Modifier = Modifier
) {
    val style = when (mode) {
        NavigationMode.INITIALIZING -> BannerModeStyle("INITIALIZING SENSORS", IdrColors.NavBlueDark, IdrColors.StatusBlueBg, IdrColors.NavBlue)
        NavigationMode.GNSS_AIDED -> BannerModeStyle("GNSS NAVIGATION", IdrColors.StatusGreenDark, IdrColors.StatusGreenBg, IdrColors.StatusGreen)
        NavigationMode.GNSS_DEGRADED -> BannerModeStyle("GNSS DEGRADED", IdrColors.StatusAmberDark, IdrColors.StatusAmberBg, IdrColors.StatusAmber)
        NavigationMode.TRANSITION_TO_DR -> BannerModeStyle("TRANSITION TO DR", IdrColors.StatusAmberDark, IdrColors.StatusAmberBg, IdrColors.StatusAmber)
        NavigationMode.DR_ONLY -> BannerModeStyle("DEAD RECKONING ACTIVE", IdrColors.StatusOrangeDark, IdrColors.StatusOrangeBg, IdrColors.StatusOrange)
        NavigationMode.TRANSITION_TO_GNSS -> BannerModeStyle("GNSS REACQUISITION", IdrColors.NavBlueDark, IdrColors.StatusBlueBg, IdrColors.NavBlue)
        NavigationMode.GNSS_REACQUIRED -> BannerModeStyle("GNSS RESTORED", IdrColors.StatusGreenDark, IdrColors.StatusGreenBg, IdrColors.StatusGreen)
        NavigationMode.UNKNOWN -> BannerModeStyle("STANDBY", IdrColors.TextSecondary, IdrColors.SurfaceElevated, IdrColors.SlateMuted)
    }

    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(2.dp, RoundedCornerShape(10.dp))
            .clip(RoundedCornerShape(10.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(10.dp))
            .padding(horizontal = 12.dp, vertical = 8.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Left: Mode Pill + Outage Timer
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(6.dp))
                        .background(style.bgColor)
                        .border(1.dp, style.dotColor.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
                        .padding(horizontal = 8.dp, vertical = 4.dp)
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Box(
                            modifier = Modifier
                                .size(6.dp)
                                .clip(CircleShape)
                                .background(style.dotColor)
                        )
                        Spacer(modifier = Modifier.width(6.dp))
                        Text(
                            text = style.modeText,
                            color = style.textColor,
                            style = IdrTypography.titleMedium.copy(fontSize = 12.sp),
                            fontWeight = FontWeight.Bold
                        )
                    }
                }

                if (outageDurationSec > 0.0) {
                    Spacer(modifier = Modifier.width(8.dp))
                    val minutes = (outageDurationSec / 60).toInt()
                    val seconds = outageDurationSec % 60
                    val timerStr = String.format("%02d:%04.1fs", minutes, seconds)
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(6.dp))
                            .background(IdrColors.StatusAmberBg)
                            .border(1.dp, IdrColors.StatusAmber.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
                            .padding(horizontal = 6.dp, vertical = 4.dp)
                    ) {
                        Text(
                            text = "OUTAGE: $timerStr",
                            color = IdrColors.StatusAmberDark,
                            style = IdrTypography.bodyMedium.copy(fontSize = 11.sp),
                            fontWeight = FontWeight.Bold
                        )
                    }
                }
            }

            // Right: GNSS Fix Status Badge
            val fixText = when {
                isGnssValid -> "GNSS 3D FIX"
                isSearchingFix -> "SEARCHING FIX"
                else -> "GNSS OUTAGE"
            }
            val fixColor = when {
                isGnssValid -> IdrColors.StatusGreenDark
                isSearchingFix -> IdrColors.NavBlueDark
                else -> IdrColors.StatusAmberDark
            }
            val fixBg = when {
                isGnssValid -> IdrColors.StatusGreenBg
                isSearchingFix -> IdrColors.StatusBlueBg
                else -> IdrColors.StatusAmberBg
            }
            val fixBorder = when {
                isGnssValid -> IdrColors.StatusGreen.copy(alpha = 0.5f)
                isSearchingFix -> IdrColors.NavBlue.copy(alpha = 0.5f)
                else -> IdrColors.StatusAmber.copy(alpha = 0.5f)
            }

            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(fixBg)
                    .border(1.dp, fixBorder, RoundedCornerShape(6.dp))
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = fixText,
                    color = fixColor,
                    style = IdrTypography.labelSmall.copy(fontSize = 11.sp),
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

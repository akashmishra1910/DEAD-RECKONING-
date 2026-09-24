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
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.GnssStatusState

/**
 * Top Floating Navigation Status Banner.
 * Dynamically presents:
 * - GNSS NAVIGATION (Green #18D47B)
 * - DEAD RECKONING ACTIVE (Amber/Orange #FF7A00)
 * - GNSS REACQUIRED (Cyan #00B8FF)
 */
@Composable
fun TopNavBanner(
    isDrActive: Boolean,
    isGnssReacquired: Boolean,
    gnssStatus: GnssStatusState,
    outageDurationSec: Double,
    modifier: Modifier = Modifier
) {
    val isOutage = isDrActive || !gnssStatus.fixValid
    val isReacquired = isGnssReacquired && !isDrActive

    val (bgGradient, borderColor, titleColor, statusText, subText, icon) = when {
        isOutage -> {
            val mins = (outageDurationSec / 60).toInt()
            val secs = (outageDurationSec % 60).toInt()
            val timeStr = String.format("%02d:%02d", mins, secs)
            Tuples6(
                Brush.horizontalGradient(listOf(Color(0xFF8A3800), Color(0xFF5A2400), Color(0xFF1A120A))),
                Color(0xFFFF7A00),
                Color(0xFFFFB020),
                "DEAD RECKONING ACTIVE",
                "GNSS Lost: $timeStr | Maintaining navigation with IDR",
                "⚠"
            )
        }
        isReacquired -> {
            Tuples6(
                Brush.horizontalGradient(listOf(Color(0xFF005A8A), Color(0xFF00385A), Color(0xFF0A1724))),
                Color(0xFF00B8FF),
                Color(0xFF66D4FF),
                "GNSS REACQUIRED",
                "Synchronizing navigation... | ${gnssStatus.totalSatellites} sats | NavIC: ${gnssStatus.navicSatellitesUsed} | ±${String.format("%.1f", gnssStatus.accuracyM.coerceAtLeast(1.2))} m",
                "⚡"
            )
        }
        else -> {
            Tuples6(
                Brush.horizontalGradient(listOf(Color(0xFF0B4A2B), Color(0xFF08331E), Color(0xFF061A10))),
                Color(0xFF18D47B),
                Color(0xFF4EE89D),
                "GNSS NAVIGATION",
                "${gnssStatus.totalSatellites} sats | NavIC: ${gnssStatus.navicSatellitesUsed} | HDOP ${gnssStatus.hdop} | ±${String.format("%.1f", gnssStatus.accuracyM.coerceAtLeast(1.0))} m",
                "●"
            )
        }
    }

    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(10.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(bgGradient)
            .border(1.2.dp, borderColor.copy(alpha = 0.85f), RoundedCornerShape(12.dp))
            .padding(horizontal = 14.dp, vertical = 10.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(28.dp)
                    .clip(CircleShape)
                    .background(borderColor.copy(alpha = 0.25f))
                    .border(1.dp, borderColor, CircleShape),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = icon,
                    fontSize = 12.sp,
                    color = borderColor,
                    fontWeight = FontWeight.Bold
                )
            }

            Spacer(modifier = Modifier.width(10.dp))

            Column(modifier = Modifier.weight(1f)) {
                Text(
                    text = statusText,
                    style = IdrTypography.titleMedium.copy(
                        fontSize = 13.sp,
                        fontWeight = FontWeight.Black,
                        letterSpacing = 1.sp
                    ),
                    color = Color.White
                )
                Text(
                    text = subText,
                    style = IdrTypography.labelSmall.copy(fontSize = 11.sp),
                    color = titleColor
                )
            }
        }
    }
}

private data class Tuples6<A, B, C, D, E, F>(
    val a: A, val b: B, val c: C, val d: D, val e: E, val f: F
)

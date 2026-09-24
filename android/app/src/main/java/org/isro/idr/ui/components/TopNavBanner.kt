package org.isro.idr.ui.components

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
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
import androidx.compose.runtime.getValue
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
 * Dynamically transitions between:
 * - GNSS NAVIGATION (Green #18D47B)
 * - DEAD RECKONING ACTIVE (Saffron/Orange #FF7A00)
 * - GNSS REACQUIRED (Cyan #00B8FF)
 * - SEARCHING SATELLITES (Blue #1688FF)
 */
@Composable
fun TopNavBanner(
    isDrActive: Boolean,
    isGnssReacquired: Boolean,
    gnssStatus: GnssStatusState,
    outageDurationSec: Double,
    onSimulateOutageToggle: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val isSearching = gnssStatus.isSearching && !gnssStatus.fixValid
    val isOutage = isDrActive || (!gnssStatus.fixValid && !isSearching)
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
                "GNSS Lost: $timeStr | Maintaining navigation with 15-State ESKF + GBDT",
                "⚠"
            )
        }
        isReacquired -> {
            Tuples6(
                Brush.horizontalGradient(listOf(Color(0xFF005A8A), Color(0xFF00385A), Color(0xFF0A1724))),
                Color(0xFF00B8FF),
                Color(0xFF66D4FF),
                "GNSS REACQUIRED",
                "Smooth Chi-Square Convergence | ${gnssStatus.totalSatellites} sats | ±${String.format("%.1f", gnssStatus.accuracyM.coerceAtLeast(1.2))} m",
                "⚡"
            )
        }
        isSearching -> {
            Tuples6(
                Brush.horizontalGradient(listOf(Color(0xFF0D3256), Color(0xFF08223D), Color(0xFF06111F))),
                Color(0xFF1688FF),
                Color(0xFF66B5FF),
                "GNSS SEARCHING",
                "Acquiring Satellites (${gnssStatus.totalSatellites} in view) | Standby Mode",
                "🛰"
            )
        }
        else -> {
            Tuples6(
                Brush.horizontalGradient(listOf(Color(0xFF0B4A2B), Color(0xFF08331E), Color(0xFF061A10))),
                Color(0xFF18D47B),
                Color(0xFF4EE89D),
                "GNSS LOCKED (READY)",
                "${gnssStatus.totalSatellites} sats | NavIC: ${gnssStatus.navicSatellitesUsed} | HDOP ${gnssStatus.hdop} | Accuracy ±${String.format("%.1f", gnssStatus.accuracyM.coerceAtLeast(1.0))} m",
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
            .clickable { onSimulateOutageToggle?.invoke() }
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
                    .border(1.dp, borderColor.copy(alpha = 0.6f), CircleShape),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = icon,
                    color = titleColor,
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Bold
                )
            }

            Spacer(modifier = Modifier.width(10.dp))

            Column(modifier = Modifier.weight(1f)) {
                Text(
                    text = statusText,
                    style = IdrTypography.titleMedium.copy(
                        fontSize = 13.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 0.5.sp
                    ),
                    color = Color.White
                )

                Text(
                    text = subText,
                    style = IdrTypography.bodySmall.copy(fontSize = 10.sp),
                    color = Color(0xFFCBD5E1),
                    maxLines = 1
                )
            }
        }
    }
}

private data class Tuples6<A, B, C, D, E, F>(
    val a: A, val b: B, val c: C, val d: D, val e: E, val f: F
)

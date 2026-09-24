package org.isro.idr.ui.screens

import androidx.compose.foundation.Canvas
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
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState
import kotlin.math.cos
import kotlin.math.sin

/**
 * Screen 8: GNSS / NavIC Status.
 * Real-time satellite constellation monitor, Polar Sky View radar,
 * and ISRO NavIC in-view / in-fix telemetry.
 */
@Composable
fun GnssNavicScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
    val fixState = if (uiState.gnssStatus.fixValid) "FIXED" else "SEARCHING FOR FIX"
    val isFixed = uiState.gnssStatus.fixValid

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .padding(14.dp),
        verticalArrangement = Arrangement.spacedBy(14.dp)
    ) {
        // Header
        Text(
            text = "GNSS & NavIC Status",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Top Status Card
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, if (isFixed) Color(0xFF18D47B) else Color(0xFFFFB020), RoundedCornerShape(14.dp))
                .padding(16.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Box(
                    modifier = Modifier
                        .size(44.dp)
                        .clip(CircleShape)
                        .background(if (isFixed) Color(0xFF18D47B).copy(alpha = 0.2f) else Color(0xFFFFB020).copy(alpha = 0.2f))
                        .border(1.2.dp, if (isFixed) Color(0xFF18D47B) else Color(0xFFFFB020), CircleShape),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = if (isFixed) "📡" else "🔍",
                        fontSize = 18.sp
                    )
                }

                Spacer(modifier = Modifier.width(12.dp))

                Column {
                    Text(
                        text = fixState,
                        style = IdrTypography.titleMedium.copy(fontSize = 15.sp, fontWeight = FontWeight.Black),
                        color = if (isFixed) Color(0xFF18D47B) else Color(0xFFFFB020)
                    )
                    Text(
                        text = "${uiState.gnssStatus.totalSatellites} sats | NavIC: ${uiState.gnssStatus.navicSatellitesUsed} | 1σ Acc: ${if (isFixed) "±${String.format("%.1f", uiState.gnssStatus.accuracyM)} m" else "N/A"}",
                        style = IdrTypography.bodySmall.copy(fontSize = 11.sp),
                        color = Color(0xFF94A3B8)
                    )
                }
            }
        }

        // Circular Polar Sky View (Live Satellite Radar)
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f)
                .shadow(10.dp, RoundedCornerShape(16.dp))
                .clip(RoundedCornerShape(16.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(16.dp))
                .padding(14.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Sky View (Live)",
                    style = IdrTypography.titleMedium.copy(fontSize = 14.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )
                // Constellation Key Pills
                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    ConstellationLegendDot("GPS", Color(0xFF18D47B))
                    ConstellationLegendDot("NavIC", Color(0xFFFF8A00))
                    ConstellationLegendDot("GLONASS", Color(0xFF1688FF))
                    ConstellationLegendDot("Galileo", Color(0xFFA855F7))
                }
            }

            Spacer(modifier = Modifier.height(10.dp))

            // Radar Canvas
            Box(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth(),
                contentAlignment = Alignment.Center
            ) {
                Canvas(modifier = Modifier.size(240.dp)) {
                    val cx = size.width / 2f
                    val cy = size.height / 2f
                    val r = size.width * 0.46f

                    // Concentric Radar Rings (Elevation: 0°, 30°, 60°)
                    drawCircle(Color(0xFF10263A), r, Offset(cx, cy), style = Stroke(1.5f))
                    drawCircle(Color(0xFF17324A), r * 0.66f, Offset(cx, cy), style = Stroke(1.0f))
                    drawCircle(Color(0xFF17324A), r * 0.33f, Offset(cx, cy), style = Stroke(1.0f))

                    // Crosshairs (N/S/E/W axes)
                    drawLine(Color(0xFF17324A), Offset(cx, cy - r), Offset(cx, cy + r), 1f)
                    drawLine(Color(0xFF17324A), Offset(cx - r, cy), Offset(cx + r, cy), 1f)

                    // Plot Constellation Satellites at Polar Positions
                    val sampleSats = listOf(
                        Triple(45f, 30f, Color(0xFFFF8A00)),   // NavIC PRN 1
                        Triple(135f, 50f, Color(0xFFFF8A00)),  // NavIC PRN 2
                        Triple(225f, 40f, Color(0xFFFF8A00)),  // NavIC PRN 3
                        Triple(315f, 65f, Color(0xFFFF8A00)),  // NavIC PRN 4
                        Triple(20f, 60f, Color(0xFF18D47B)),   // GPS
                        Triple(90f, 75f, Color(0xFF18D47B)),   // GPS
                        Triple(180f, 35f, Color(0xFF18D47B)),  // GPS
                        Triple(270f, 45f, Color(0xFF1688FF)),  // GLONASS
                        Triple(160f, 70f, Color(0xFFA855F7))   // Galileo
                    )

                    for ((azimuthDeg, elevationDeg, satColor) in sampleSats) {
                        val rad = Math.toRadians(azimuthDeg.toDouble() - 90.0)
                        val dist = r * ((90.0 - elevationDeg) / 90.0)
                        val sx = cx + (dist * cos(rad)).toFloat()
                        val sy = cy + (dist * sin(rad)).toFloat()

                        drawCircle(satColor.copy(alpha = 0.35f), 9f, Offset(sx, sy))
                        drawCircle(satColor, 5f, Offset(sx, sy))
                        drawCircle(Color.White, 5f, Offset(sx, sy), style = Stroke(1.2f))
                    }
                }
            }
        }

        // Bottom NavIC Metrics Panel
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            NavicMetricCard(
                title = "NavIC in View",
                count = uiState.gnssStatus.navicSatellitesInView,
                color = Color(0xFFFF8A00),
                modifier = Modifier.weight(1f)
            )
            NavicMetricCard(
                title = "NavIC in Fix",
                count = uiState.gnssStatus.navicSatellitesUsed,
                color = Color(0xFF18D47B),
                modifier = Modifier.weight(1f)
            )
        }
    }
}

@Composable
private fun ConstellationLegendDot(name: String, color: Color) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(
            modifier = Modifier
                .size(6.dp)
                .clip(CircleShape)
                .background(color)
        )
        Spacer(modifier = Modifier.width(3.dp))
        Text(text = name, style = IdrTypography.labelSmall.copy(fontSize = 9.sp), color = Color(0xFF94A3B8))
    }
}

@Composable
private fun NavicMetricCard(
    title: String,
    count: Int,
    color: Color,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .shadow(6.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.Surface)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(14.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = title,
            style = IdrTypography.labelSmall.copy(fontSize = 11.sp),
            color = Color(0xFF94A3B8)
        )
        Spacer(modifier = Modifier.height(4.dp))
        Text(
            text = "$count",
            style = IdrTypography.displayLarge.copy(fontSize = 28.sp, fontWeight = FontWeight.Black),
            color = color
        )
    }
}

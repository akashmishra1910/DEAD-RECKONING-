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
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.ConstellationType
import org.isro.idr.ui.state.NavigationUiState
import kotlin.math.cos
import kotlin.math.sin

/**
 * Screen 8: GNSS & ISRO NavIC Status.
 * Real-time satellite constellation monitor, Polar Sky View radar,
 * and live hardware-level satellite diagnostic inspection.
 */
@Composable
fun GnssNavicScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
    val isFixed = uiState.gnssStatus.fixValid
    val fixState = if (isFixed) "FIXED" else "SEARCHING FOR FIX"
    val navicInView = uiState.gnssStatus.navicSatellitesInView
    val navicUsed = uiState.gnssStatus.navicSatellitesUsed
    val totalInView = uiState.gnssStatus.totalSatellitesInView
    val totalUsed = uiState.gnssStatus.totalSatellites

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .verticalScroll(rememberScrollState())
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
                        text = "Total GNSS: $totalUsed in fix / $totalInView in view | NavIC: $navicUsed in fix / $navicInView in view | Acc: ${if (isFixed) "±${String.format("%.1f", uiState.gnssStatus.accuracyM)} m" else "N/A"}",
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
                    text = "Polar Sky View (${uiState.gnssStatus.satellites.size} visible)",
                    style = IdrTypography.titleMedium.copy(fontSize = 14.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )
                // Constellation Key Pills
                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    val gpsCnt = uiState.gnssStatus.satellites.count { it.constellation == ConstellationType.GPS }
                    val navicCnt = uiState.gnssStatus.satellites.count { it.constellation == ConstellationType.NAVIC }
                    val gloCnt = uiState.gnssStatus.satellites.count { it.constellation == ConstellationType.GLONASS }
                    val galCnt = uiState.gnssStatus.satellites.count { it.constellation == ConstellationType.GALILEO }
                    ConstellationLegendDot("GPS ($gpsCnt)", Color(0xFF18D47B))
                    ConstellationLegendDot("NavIC ($navicCnt)", Color(0xFFFF8A00))
                    ConstellationLegendDot("GLO ($gloCnt)", Color(0xFF1688FF))
                    ConstellationLegendDot("GAL ($galCnt)", Color(0xFFA855F7))
                }
            }

            Spacer(modifier = Modifier.height(10.dp))

            // Radar Canvas
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(240.dp),
                contentAlignment = Alignment.Center
            ) {
                Canvas(modifier = Modifier.size(230.dp)) {
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

                    // Plot Real Constellation Satellites at Polar Positions
                    val activeSats = uiState.gnssStatus.satellites
                    if (activeSats.isNotEmpty()) {
                        for (sat in activeSats) {
                            val satColor = when (sat.constellation) {
                                ConstellationType.GPS -> Color(0xFF18D47B)
                                ConstellationType.NAVIC -> Color(0xFFFF8A00)
                                ConstellationType.GLONASS -> Color(0xFF1688FF)
                                ConstellationType.GALILEO -> Color(0xFFA855F7)
                                ConstellationType.BEIDOU -> Color(0xFF00E5FF)
                                ConstellationType.QZSS -> Color(0xFFFBBF24)
                                ConstellationType.SBAS -> Color(0xFF94A3B8)
                                ConstellationType.UNKNOWN -> Color(0xFF64748B)
                            }
                            val rad = Math.toRadians(sat.azimuthDeg.toDouble() - 90.0)
                            val dist = r * ((90.0 - sat.elevationDeg.coerceIn(0f, 90f)) / 90.0)
                            val sx = cx + (dist * cos(rad)).toFloat()
                            val sy = cy + (dist * sin(rad)).toFloat()

                            val alpha = if (sat.usedInFix) 0.95f else 0.45f
                            drawCircle(satColor.copy(alpha = 0.35f * alpha), 9f, Offset(sx, sy))
                            drawCircle(satColor.copy(alpha = alpha), 5f, Offset(sx, sy))
                            if (sat.usedInFix) {
                                drawCircle(Color.White, 5f, Offset(sx, sy), style = Stroke(1.2f))
                            }
                        }
                    }
                }
                if (uiState.gnssStatus.satellites.isEmpty()) {
                    Text(
                        text = "Searching for GNSS / NavIC signals...",
                        style = IdrTypography.bodySmall.copy(fontSize = 11.sp),
                        color = Color(0xFF64748B)
                    )
                }
            }
        }

        // NavIC Diagnostic Banner if no IRNSS satellites visible
        if (navicInView == 0) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(10.dp))
                    .background(Color(0xFF1E293B))
                    .border(1.dp, Color(0xFF334155), RoundedCornerShape(10.dp))
                    .padding(12.dp)
            ) {
                Column {
                    Text(
                        text = "NavIC (IRNSS) currently not visible at device location",
                        fontSize = 12.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color(0xFFFFAB40)
                    )
                    Spacer(modifier = Modifier.height(2.dp))
                    Text(
                        text = "Device chipset is monitoring CONSTELLATION_IRNSS (ID 7) and L5 carrier frequencies. Satellites will appear automatically when in line-of-sight.",
                        fontSize = 10.sp,
                        color = Color(0xFF94A3B8)
                    )
                }
            }
        }

        // NavIC & GNSS Metrics Cards
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            NavicMetricCard(
                title = "NavIC in View",
                count = navicInView,
                color = Color(0xFFFF8A00),
                modifier = Modifier.weight(1f)
            )
            NavicMetricCard(
                title = "NavIC in Fix",
                count = navicUsed,
                color = Color(0xFF18D47B),
                modifier = Modifier.weight(1f)
            )
            NavicMetricCard(
                title = "Total Sats",
                count = totalInView,
                color = Color(0xFF00B8FF),
                modifier = Modifier.weight(1f)
            )
        }

        // Raw Satellite Hardware Diagnostic Table
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(Color(0xFF0D2238))
                .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text(
                text = "Live Satellite Hardware Telemetry",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            // Table Header
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(Color(0xFF17324A))
                    .padding(horizontal = 8.dp, vertical = 6.dp),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text("SVID", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color.White, modifier = Modifier.weight(0.8f))
                Text("CONST", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color.White, modifier = Modifier.weight(1.2f))
                Text("ELEV", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color.White, modifier = Modifier.weight(0.8f))
                Text("AZIM", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color.White, modifier = Modifier.weight(0.8f))
                Text("C/N0", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color.White, modifier = Modifier.weight(0.9f))
                Text("FIX", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color.White, modifier = Modifier.weight(0.8f))
            }

            if (uiState.gnssStatus.satellites.isNotEmpty()) {
                uiState.gnssStatus.satellites.take(15).forEach { sat ->
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(horizontal = 8.dp, vertical = 4.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text("${sat.svid}", fontSize = 10.sp, fontFamily = FontFamily.Monospace, color = Color(0xFFCBD5E1), modifier = Modifier.weight(0.8f))
                        Text(sat.constellation.displayName, fontSize = 10.sp, fontWeight = FontWeight.SemiBold, color = when(sat.constellation) {
                            ConstellationType.NAVIC -> Color(0xFFFF8A00)
                            ConstellationType.GPS -> Color(0xFF18D47B)
                            ConstellationType.GLONASS -> Color(0xFF1688FF)
                            ConstellationType.GALILEO -> Color(0xFFA855F7)
                            ConstellationType.BEIDOU -> Color(0xFF00E5FF)
                            else -> Color(0xFF94A3B8)
                        }, modifier = Modifier.weight(1.2f))
                        Text(String.format("%.0f°", sat.elevationDeg), fontSize = 10.sp, fontFamily = FontFamily.Monospace, color = Color(0xFF94A3B8), modifier = Modifier.weight(0.8f))
                        Text(String.format("%.0f°", sat.azimuthDeg), fontSize = 10.sp, fontFamily = FontFamily.Monospace, color = Color(0xFF94A3B8), modifier = Modifier.weight(0.8f))
                        Text(String.format("%.1f", sat.snrDbHz), fontSize = 10.sp, fontFamily = FontFamily.Monospace, color = if (sat.snrDbHz > 30) Color(0xFF18D47B) else Color(0xFFFFB020), modifier = Modifier.weight(0.9f))
                        Text(if (sat.usedInFix) "USED" else "VIEW", fontSize = 10.sp, fontWeight = FontWeight.Bold, color = if (sat.usedInFix) Color(0xFF18D47B) else Color(0xFF64748B), modifier = Modifier.weight(0.8f))
                    }
                }
            } else {
                Text(
                    text = "No GNSS/NavIC hardware telemetry received yet",
                    fontSize = 11.sp,
                    color = Color(0xFF64748B),
                    modifier = Modifier.padding(8.dp)
                )
            }
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

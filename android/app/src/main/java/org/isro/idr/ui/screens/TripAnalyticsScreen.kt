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
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.TripEvent

/**
 * Screen 9: Trip & Analytics.
 * Journey metrics, position & drift error progression chart, and event timeline.
 */
@Composable
fun TripAnalyticsScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
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
            text = "Trip Analytics",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Top Journey Summary Card
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Column {
                    Text(text = "Start Time", style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
                    Text(text = uiState.tripStartTime, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold), color = Color.White)
                }
                Column(horizontalAlignment = Alignment.End) {
                    Text(text = "Duration", style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
                    val hours = uiState.tripDurationSec / 3600
                    val mins = (uiState.tripDurationSec % 3600) / 60
                    val secs = uiState.tripDurationSec % 60
                    Text(
                        text = String.format("%02d:%02d:%02d", hours, mins, secs),
                        style = IdrTypography.bodyMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold),
                        color = Color(0xFF00B8FF)
                    )
                }
            }

            // 3-Metric Strip (Distance, Avg Speed, Max Speed)
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(10.dp))
                    .background(Color(0xFF06111F))
                    .padding(vertical = 10.dp, horizontal = 12.dp),
                horizontalArrangement = Arrangement.SpaceAround
            ) {
                TripStatItem("Distance", "${String.format("%.1f", uiState.tripDistanceMeters / 1000.0)} km", Color.White)
                TripStatItem("Avg. Speed", "${String.format("%.1f", uiState.tripAvgSpeedKmh)} km/h", Color.White)
                TripStatItem("Max Speed", "${String.format("%.1f", uiState.tripMaxSpeedKmh)} km/h", Color(0xFF18D47B))
            }
        }

        // Position & Drift Error Progression Chart Panel
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Position & Drift (m)",
                    style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )

                // Chart Legend
                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    ChartLegendItem("GNSS", Color(0xFF1688FF))
                    ChartLegendItem("Raw DR", Color(0xFFFF7A00))
                    ChartLegendItem("Map Matched", Color(0xFFA855F7))
                }
            }

            // Drift Multi-Line Canvas
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(140.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color(0xFF06111F))
                    .border(1.dp, Color(0xFF17324A), RoundedCornerShape(8.dp))
            ) {
                DriftChartCanvas()
            }
        }

        // Events Timeline Section
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Text(
                text = "Events Timeline",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                uiState.tripEvents.forEach { event ->
                    TimelineEventRow(event)
                }
            }
        }
    }
}

@Composable
private fun TripStatItem(title: String, value: String, valueColor: Color) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(text = value, style = IdrTypography.titleMedium.copy(fontSize = 15.sp, fontWeight = FontWeight.Black), color = valueColor)
        Text(text = title, style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
    }
}

@Composable
private fun ChartLegendItem(name: String, color: Color) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(modifier = Modifier.size(6.dp).clip(CircleShape).background(color))
        Spacer(modifier = Modifier.width(3.dp))
        Text(text = name, style = IdrTypography.labelSmall.copy(fontSize = 9.sp), color = Color(0xFF94A3B8))
    }
}

@Composable
private fun DriftChartCanvas() {
    Canvas(modifier = Modifier.fillMaxSize().padding(8.dp)) {
        val w = size.width
        val h = size.height

        // Grid lines
        drawLine(Color(0xFF10263A), Offset(0f, h * 0.33f), Offset(w, h * 0.33f), 1f)
        drawLine(Color(0xFF10263A), Offset(0f, h * 0.66f), Offset(w, h * 0.66f), 1f)

        // Plot GNSS Error (Blue Line - Flat/bounded)
        val pathGnss = Path()
        val pathDr = Path()
        val pathMap = Path()

        val n = 20
        for (i in 0..n) {
            val px = (i.toFloat() / n) * w
            val pyGnss = h - (15f + (i % 3) * 4f)
            // Raw DR error grows with outage
            val pyDr = h - (12f + (i * i * 0.28f).coerceAtMost(h * 0.85f))
            // Map Matched error stays bounded
            val pyMap = h - (10f + (i * 2.2f).coerceAtMost(h * 0.45f))

            if (i == 0) {
                pathGnss.moveTo(px, pyGnss)
                pathDr.moveTo(px, pyDr)
                pathMap.moveTo(px, pyMap)
            } else {
                pathGnss.lineTo(px, pyGnss)
                pathDr.lineTo(px, pyDr)
                pathMap.lineTo(px, pyMap)
            }
        }

        drawPath(pathGnss, color = Color(0xFF1688FF), style = Stroke(2.0f, cap = StrokeCap.Round, join = StrokeJoin.Round))
        drawPath(pathDr, color = Color(0xFFFF7A00), style = Stroke(2.2f, cap = StrokeCap.Round, join = StrokeJoin.Round))
        drawPath(pathMap, color = Color(0xFFA855F7), style = Stroke(2.0f, cap = StrokeCap.Round, join = StrokeJoin.Round))
    }
}

@Composable
private fun TimelineEventRow(event: TripEvent) {
    val dotColor = when (event.type) {
        "GNSS_ACQUIRED" -> Color(0xFF18D47B)
        "DR_ACTIVE" -> Color(0xFFFF7A00)
        "MAP_AIDED" -> Color(0xFFA855F7)
        else -> Color(0xFF00B8FF)
    }

    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(modifier = Modifier.size(7.dp).clip(CircleShape).background(dotColor))
        Spacer(modifier = Modifier.width(10.dp))
        Text(text = event.title, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.SemiBold), color = Color.White)
        Spacer(modifier = Modifier.weight(1f))
        Text(text = event.timestamp, style = IdrTypography.labelSmall.copy(fontSize = 11.sp), color = Color(0xFF64748B))
    }
}

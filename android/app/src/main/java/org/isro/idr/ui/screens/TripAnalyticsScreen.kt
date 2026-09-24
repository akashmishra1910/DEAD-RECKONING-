package org.isro.idr.ui.screens

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
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
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.TripEvent

data class HistoricalTripRecord(
    val id: String,
    val title: String,
    val dateStr: String,
    val durationStr: String,
    val distanceKm: Double,
    val avgSpeedKmh: Double,
    val maxSpeedKmh: Double,
    val outageDurationSec: Int,
    val maxDriftPct: Double,
    val status: String = "SUCCESS"
)

private val SampleTripHistory = listOf(
    HistoricalTripRecord(
        id = "trip_01",
        title = "Prayagraj Corridor Field Trial",
        dateStr = "24 Sep 2026, 18:30",
        durationStr = "12m 45s",
        distanceKm = 3.24,
        avgSpeedKmh = 28.4,
        maxSpeedKmh = 62.1,
        outageDurationSec = 60,
        maxDriftPct = 2.01
    ),
    HistoricalTripRecord(
        id = "trip_02",
        title = "Civil Lines Stop-and-Go Trial",
        dateStr = "24 Sep 2026, 16:15",
        durationStr = "18m 10s",
        distanceKm = 4.15,
        avgSpeedKmh = 21.2,
        maxSpeedKmh = 48.0,
        outageDurationSec = 120,
        maxDriftPct = 2.88
    ),
    HistoricalTripRecord(
        id = "trip_03",
        title = "Yamuna Bridge Flyover Disambiguation",
        dateStr = "23 Sep 2026, 11:20",
        durationStr = "15m 30s",
        distanceKm = 5.80,
        avgSpeedKmh = 38.6,
        maxSpeedKmh = 78.4,
        outageDurationSec = 90,
        maxDriftPct = 2.15
    )
)

/**
 * Screen 9: Trip & Analytics with Interactive Trip History and Drift Progression.
 */
@Composable
fun TripAnalyticsScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
    var selectedHistoricalTrip by remember { mutableStateOf<HistoricalTripRecord?>(null) }

    val displayDistance = selectedHistoricalTrip?.distanceKm ?: (uiState.tripDistanceMeters / 1000.0)
    val displayAvgSpeed = selectedHistoricalTrip?.avgSpeedKmh ?: uiState.tripAvgSpeedKmh
    val displayMaxSpeed = selectedHistoricalTrip?.maxSpeedKmh ?: uiState.tripMaxSpeedKmh
    val displayDuration = selectedHistoricalTrip?.durationStr ?: run {
        val h = uiState.tripDurationSec / 3600
        val m = (uiState.tripDurationSec % 3600) / 60
        val s = uiState.tripDurationSec % 60
        if (h > 0) String.format("%02d:%02d:%02d", h, m, s) else String.format("%02d:%02d", m, s)
    }

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .verticalScroll(rememberScrollState())
            .padding(14.dp),
        verticalArrangement = Arrangement.spacedBy(14.dp)
    ) {
        // Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "Trip Analytics & History",
                style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            if (selectedHistoricalTrip != null) {
                Text(
                    text = "Live Run ↺",
                    fontSize = 11.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color(0xFFFF8A00),
                    modifier = Modifier.clickable { selectedHistoricalTrip = null }
                )
            }
        }

        // Top Journey Summary Card
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(Color(0xFF0D2238))
                .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Column {
                    Text(text = "Trip Name", style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
                    Text(
                        text = selectedHistoricalTrip?.title ?: "Active Navigation Session",
                        style = IdrTypography.bodyMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold),
                        color = Color.White
                    )
                }
                Column(horizontalAlignment = Alignment.End) {
                    Text(text = "Duration", style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
                    Text(
                        text = displayDuration,
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
                    .padding(10.dp),
                horizontalArrangement = Arrangement.SpaceAround
            ) {
                MetricColumn("DISTANCE", String.format("%.2f km", displayDistance), Color(0xFFFF8A00))
                MetricColumn("AVG SPEED", String.format("%.1f km/h", displayAvgSpeed), Color(0xFF1688FF))
                MetricColumn("MAX SPEED", String.format("%.1f km/h", displayMaxSpeed), Color(0xFF18D47B))
            }
        }

        // Section 2: Position Drift vs PS 26168 Bound (<10%) Chart
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(Color(0xFF0D2238))
                .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Drift vs Outage Duration (PS Benchmark)",
                    style = IdrTypography.titleMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )

                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF0B4A2B))
                        .padding(horizontal = 6.dp, vertical = 2.dp)
                ) {
                    Text(
                        text = "PASS: 2.01% @ 60s",
                        fontSize = 9.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color(0xFF18D47B)
                    )
                }
            }

            // High-Contrast Progression Chart
            Canvas(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(130.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color(0xFF06111F))
            ) {
                val w = size.width
                val h = size.height
                val padding = 20f

                // Grid Lines
                for (i in 1..3) {
                    val y = padding + (h - 2 * padding) * (i / 4f)
                    drawLine(
                        color = Color(0xFF17324A),
                        start = Offset(padding, y),
                        end = Offset(w - padding, y),
                        strokeWidth = 1f
                    )
                }

                // 10% Benchmark Bound Line (Red Dashed)
                val boundY = padding + (h - 2 * padding) * 0.15f
                drawLine(
                    color = Color(0xFFFF4F5E).copy(alpha = 0.7f),
                    start = Offset(padding, boundY),
                    end = Offset(w - padding, boundY),
                    strokeWidth = 1.5f
                )

                // IDR Filter Drift Curve (Orange)
                val idrCurve = Path().apply {
                    moveTo(padding, h - padding)
                    cubicTo(
                        w * 0.35f, h - padding - 20f,
                        w * 0.70f, h - padding - 45f,
                        w - padding, h - padding - 65f
                    )
                }
                drawPath(
                    path = idrCurve,
                    color = Color(0xFFFF8A00),
                    style = Stroke(width = 3.2f, cap = StrokeCap.Round)
                )

                // Map-Aided Drift Curve (Cyan - Near Zero)
                val mapCurve = Path().apply {
                    moveTo(padding, h - padding)
                    cubicTo(
                        w * 0.35f, h - padding - 8f,
                        w * 0.70f, h - padding - 18f,
                        w - padding, h - padding - 24f
                    )
                }
                drawPath(
                    path = mapCurve,
                    color = Color(0xFF00E5FF),
                    style = Stroke(width = 2.5f, cap = StrokeCap.Round)
                )
            }

            // Legend
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceAround
            ) {
                LegendItem("IDR Drift (2.01%)", Color(0xFFFF8A00))
                LegendItem("Map-Aided (0.8%)", Color(0xFF00E5FF))
                LegendItem("10% Bound", Color(0xFFFF4F5E))
            }
        }

        // Section 3: Trip History List
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(Color(0xFF0D2238))
                .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Text(
                text = "Recorded Trip History",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            SampleTripHistory.forEach { trip ->
                val isSelected = selectedHistoricalTrip?.id == trip.id
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .clip(RoundedCornerShape(10.dp))
                        .background(if (isSelected) Color(0xFF17324A) else Color(0xFF06111F))
                        .border(1.dp, if (isSelected) Color(0xFFFF8A00) else Color(0xFF0F273D), RoundedCornerShape(10.dp))
                        .clickable { selectedHistoricalTrip = trip }
                        .padding(12.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column(modifier = Modifier.weight(1f)) {
                        Text(
                            text = trip.title,
                            fontSize = 13.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color.White
                        )
                        Spacer(modifier = Modifier.height(2.dp))
                        Text(
                            text = "${trip.dateStr} • ${trip.durationStr} • ${trip.distanceKm} km",
                            fontSize = 10.sp,
                            color = Color(0xFF94A3B8)
                        )
                    }

                    Column(horizontalAlignment = Alignment.End) {
                        Text(
                            text = "Drift: ${trip.maxDriftPct}%",
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color(0xFF18D47B)
                        )
                        Text(
                            text = "${trip.outageDurationSec}s Outage",
                            fontSize = 9.sp,
                            color = Color(0xFFFFAB40)
                        )
                    }
                }
            }
        }
    }
}

@Composable
private fun MetricColumn(label: String, value: String, color: Color) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(text = label, fontSize = 9.sp, fontWeight = FontWeight.Bold, color = Color(0xFF64748B), letterSpacing = 0.5.sp)
        Spacer(modifier = Modifier.height(2.dp))
        Text(text = value, fontSize = 13.sp, fontWeight = FontWeight.ExtraBold, color = color, fontFamily = FontFamily.Monospace)
    }
}

@Composable
private fun LegendItem(label: String, color: Color) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(5.dp)
    ) {
        Box(
            modifier = Modifier
                .size(8.dp)
                .clip(CircleShape)
                .background(color)
        )
        Text(text = label, fontSize = 10.sp, color = Color(0xFFCBD5E1))
    }
}

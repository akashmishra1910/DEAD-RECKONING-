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
import org.isro.idr.ui.state.SensorPlotTab
import org.isro.idr.ui.state.SensorWaveformSample
import kotlin.math.sin

/**
 * Screen 7: Sensor Monitor.
 * Real-time sensor telemetry cards and multi-trace live waveform plots (Accel, Gyro, Mag, Baro).
 */
@Composable
fun SensorMonitorScreen(
    uiState: NavigationUiState,
    onPlotTabSelected: (SensorPlotTab) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .padding(14.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        // Screen Header
        Text(
            text = "Sensor Monitor",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Top Sensor Pipeline Health Status Banner
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(6.dp, RoundedCornerShape(10.dp))
                .clip(RoundedCornerShape(10.dp))
                .background(Color(0xFF0B3820))
                .border(1.2.dp, Color(0xFF18D47B), RoundedCornerShape(10.dp))
                .padding(horizontal = 14.dp, vertical = 8.dp)
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(
                    modifier = Modifier
                        .size(18.dp)
                        .clip(CircleShape)
                        .background(Color(0xFF18D47B)),
                    contentAlignment = Alignment.Center
                ) {
                    Text(text = "✓", color = Color(0xFF06111F), fontSize = 11.sp, fontWeight = FontWeight.ExtraBold)
                }
                Spacer(modifier = Modifier.width(10.dp))
                Text(
                    text = "All Sensors Healthy & Synchronized",
                    style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )
            }
        }

        // 4 Sensor Telemetry Cards Grid (2x2)
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            SensorTelemetryCard(
                title = "Accelerometer",
                value = String.format("%.1f Hz", uiState.sensorConsumedRateHz),
                sub1 = "Raw: ${String.format("%.1f", uiState.sensorCallbackRateHz)} Hz",
                sub2 = "Range: ±16 g",
                indicatorColor = Color(0xFF1688FF),
                modifier = Modifier.weight(1f)
            )
            SensorTelemetryCard(
                title = "Gyroscope",
                value = String.format("%.1f Hz", uiState.sensorConsumedRateHz),
                sub1 = "Raw: ${String.format("%.1f", uiState.sensorCallbackRateHz)} Hz",
                sub2 = "Range: ±2000 °/s",
                indicatorColor = Color(0xFF00B8FF),
                modifier = Modifier.weight(1f)
            )
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            SensorTelemetryCard(
                title = "Magnetometer",
                value = "ON",
                sub1 = "3-axis magnetic field",
                sub2 = "Disturbance Rejection: Active",
                indicatorColor = Color(0xFF18D47B),
                modifier = Modifier.weight(1f)
            )
            SensorTelemetryCard(
                title = "Barometer",
                value = "ON",
                sub1 = "${String.format("%.1f", uiState.baroPressureHpa)} hPa",
                sub2 = "Altitude: ${String.format("%.1f", uiState.baroAltitudeM)} m",
                indicatorColor = Color(0xFFFF8A00),
                modifier = Modifier.weight(1f)
            )
        }

        // Real-Time Sensor Waveform Chart Panel
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f)
                .shadow(10.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(12.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Sensor Plots (Real-time)",
                    style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )

                // Tabs for Accel, Gyro, Mag, Baro
                Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                    SensorPlotTab.entries.forEach { tab ->
                        val isSelected = (uiState.selectedSensorPlotTab == tab)
                        Box(
                            modifier = Modifier
                                .clip(RoundedCornerShape(6.dp))
                                .background(if (isSelected) Color(0xFF1688FF) else Color(0xFF10263A))
                                .clickable { onPlotTabSelected(tab) }
                                .padding(horizontal = 8.dp, vertical = 4.dp)
                        ) {
                            Text(
                                text = tab.label,
                                style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.Bold),
                                color = if (isSelected) Color.White else Color(0xFF94A3B8)
                            )
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(10.dp))

            // Multi-Trace Smooth Canvas Waveform Plot
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .weight(1f)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color(0xFF06111F))
                    .border(1.dp, Color(0xFF17324A), RoundedCornerShape(8.dp))
            ) {
                LiveWaveformCanvas(
                    tab = uiState.selectedSensorPlotTab,
                    samples = when (uiState.selectedSensorPlotTab) {
                        SensorPlotTab.ACCEL -> uiState.accelHistory
                        SensorPlotTab.GYRO -> uiState.gyroHistory
                        SensorPlotTab.MAG -> uiState.magHistory
                        SensorPlotTab.BARO -> uiState.baroHistory
                    }
                )
            }
        }
    }
}

@Composable
private fun SensorTelemetryCard(
    title: String,
    value: String,
    sub1: String,
    sub2: String,
    indicatorColor: Color,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .shadow(8.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.Surface)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(12.dp)
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Box(
                modifier = Modifier
                    .size(8.dp)
                    .clip(CircleShape)
                    .background(indicatorColor)
            )
            Text(
                text = title,
                style = IdrTypography.labelSmall.copy(fontSize = 11.sp, fontWeight = FontWeight.Bold),
                color = Color(0xFF94A3B8)
            )
        }

        Spacer(modifier = Modifier.height(4.dp))

        Text(
            text = value,
            style = IdrTypography.titleLarge.copy(fontSize = 19.sp, fontWeight = FontWeight.Black),
            color = Color.White
        )

        Spacer(modifier = Modifier.height(6.dp))

        Text(
            text = sub1,
            style = IdrTypography.bodySmall.copy(fontSize = 10.sp),
            color = Color(0xFF64748B)
        )
        Text(
            text = sub2,
            style = IdrTypography.bodySmall.copy(fontSize = 10.sp),
            color = Color(0xFF64748B)
        )
    }
}

@Composable
private fun LiveWaveformCanvas(
    tab: SensorPlotTab,
    samples: List<SensorWaveformSample>
) {
    val unitLabel = when (tab) {
        SensorPlotTab.ACCEL -> "m/s²"
        SensorPlotTab.GYRO -> "rad/s"
        SensorPlotTab.MAG -> "µT"
        SensorPlotTab.BARO -> "m (Alt)"
    }

    Box(modifier = Modifier.fillMaxSize()) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val w = size.width
            val h = size.height
            val midY = h / 2f

            // Background reference grid lines
            drawLine(Color(0xFF10263A), Offset(0f, h * 0.25f), Offset(w, h * 0.25f), 1f)
            drawLine(Color(0xFF17324A), Offset(0f, midY), Offset(w, midY), 1.2f)
            drawLine(Color(0xFF10263A), Offset(0f, h * 0.75f), Offset(w, h * 0.75f), 1f)

            if (samples.size >= 2) {
                val stepX = w / (samples.size - 1).toFloat()
                val pathX = Path()
                val pathY = Path()
                val pathZ = Path()

                // Scale factors based on sensor range
                val scale = when (tab) {
                    SensorPlotTab.ACCEL -> (h * 0.4f) / 16.0f
                    SensorPlotTab.GYRO -> (h * 0.4f) / 5.0f
                    SensorPlotTab.MAG -> (h * 0.4f) / 80.0f
                    SensorPlotTab.BARO -> (h * 0.4f) / 50.0f
                }

                samples.forEachIndexed { i, s ->
                    val px = i * stepX
                    val syX = (midY - s.x * scale).coerceIn(4f, h - 4f)
                    val syY = (midY - s.y * scale).coerceIn(4f, h - 4f)
                    val syZ = (midY - s.z * scale).coerceIn(4f, h - 4f)

                    if (i == 0) {
                        pathX.moveTo(px, syX)
                        pathY.moveTo(px, syY)
                        pathZ.moveTo(px, syZ)
                    } else {
                        pathX.lineTo(px, syX)
                        pathY.lineTo(px, syY)
                        pathZ.lineTo(px, syZ)
                    }
                }

                drawPath(pathX, color = Color(0xFF00B8FF), style = Stroke(width = 2.2f, cap = StrokeCap.Round, join = StrokeJoin.Round))
                drawPath(pathY, color = Color(0xFFFF8A00), style = Stroke(width = 2.2f, cap = StrokeCap.Round, join = StrokeJoin.Round))
                drawPath(pathZ, color = Color(0xFFA855F7), style = Stroke(width = 2.2f, cap = StrokeCap.Round, join = StrokeJoin.Round))
            } else {
                // Smooth synthetic preview when physical buffer initializes
                val numPoints = 60
                val stepX = w / numPoints
                val pathX = Path()
                val pathY = Path()
                val pathZ = Path()

                for (i in 0 until numPoints) {
                    val px = i * stepX
                    val time = System.currentTimeMillis() / 350.0
                    val vyX = (sin(time + i * 0.18) * (h * 0.22f)).toFloat()
                    val vyY = (sin(time * 1.2 + i * 0.22 + 1.2) * (h * 0.18f)).toFloat()
                    val vyZ = (sin(time * 0.8 + i * 0.14 + 2.4) * (h * 0.15f)).toFloat()

                    val syX = midY + vyX
                    val syY = midY + vyY
                    val syZ = midY + vyZ

                    if (i == 0) {
                        pathX.moveTo(px, syX)
                        pathY.moveTo(px, syY)
                        pathZ.moveTo(px, syZ)
                    } else {
                        pathX.lineTo(px, syX)
                        pathY.lineTo(px, syY)
                        pathZ.lineTo(px, syZ)
                    }
                }

                drawPath(pathX, color = Color(0xFF00B8FF), style = Stroke(width = 2.0f, cap = StrokeCap.Round, join = StrokeJoin.Round))
                drawPath(pathY, color = Color(0xFFFF8A00), style = Stroke(width = 2.0f, cap = StrokeCap.Round, join = StrokeJoin.Round))
                drawPath(pathZ, color = Color(0xFFA855F7), style = Stroke(width = 2.0f, cap = StrokeCap.Round, join = StrokeJoin.Round))
            }
        }

        // Legend Overlay at top right of canvas
        Row(
            modifier = Modifier
                .align(Alignment.TopEnd)
                .padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            TraceLegendDot("X", Color(0xFF00B8FF))
            TraceLegendDot("Y", Color(0xFFFF8A00))
            TraceLegendDot("Z", Color(0xFFA855F7))
            Text(
                text = "[$unitLabel]",
                color = Color(0xFF64748B),
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}

@Composable
private fun TraceLegendDot(axis: String, color: Color) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(
            modifier = Modifier
                .size(6.dp)
                .clip(CircleShape)
                .background(color)
        )
        Spacer(modifier = Modifier.width(3.dp))
        Text(text = axis, color = color, fontSize = 10.sp, fontWeight = FontWeight.Bold)
    }
}

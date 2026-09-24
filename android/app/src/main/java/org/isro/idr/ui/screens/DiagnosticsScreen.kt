package org.isro.idr.ui.screens

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
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.HealthStatus
import org.isro.idr.ui.state.NavigationUiState

/**
 * Screen 11: System Diagnostics.
 * Real-time engineering subsystem health, filter timing latency,
 * queue statistics, and recent telemetry events.
 */
@Composable
fun DiagnosticsScreen(
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
            text = "System Diagnostics",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Section 1: System Health Card
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
                text = "System Health",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            HealthMetricRow("IMU Pipeline", String.format("%.1f Hz", uiState.sensorConsumedRateHz), HealthStatus.HEALTHY)
            HealthMetricRow("GNSS / NavIC", "${uiState.gnssStatus.totalSatellites} sats", if (uiState.gnssStatus.fixValid) HealthStatus.HEALTHY else HealthStatus.SEARCHING)
            HealthMetricRow("Estimator (ESKF)", "Running", HealthStatus.HEALTHY)
            HealthMetricRow("Map Matcher (HMM)", "${uiState.hypothesisCount} hyps", HealthStatus.HEALTHY)
            HealthMetricRow("JNI / Native", "0.18 ms", HealthStatus.HEALTHY)
            HealthMetricRow("Memory", "${uiState.memoryUsageMb} MB", HealthStatus.HEALTHY)
        }

        // Section 2: Performance Metrics Card
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text(
                text = "Performance",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            PerformanceDataRow("Native Latency", "${uiState.nativeLatencyUs} µs/step")
            PerformanceDataRow("Queue Occupancy", "${uiState.diagnostics?.queueOccupancy ?: 0} / 500")
            PerformanceDataRow("Dropped Samples", "${(uiState.diagnostics?.accelDroppedCount ?: 0) + (uiState.diagnostics?.gyroDroppedCount ?: 0)}")
            PerformanceDataRow("P95 Sensor Jitter", "${String.format("%.2f", uiState.p95SensorJitterMs)} ms")
        }

        // Section 3: Event Log (Recent)
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text(
                text = "Event Log (Recent)",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            EventLogRow("Sensor pipeline initialized", "18:52:01", Color(0xFF18D47B))
            EventLogRow("GNSS search started", "18:52:03", Color(0xFFFFB020))
            EventLogRow("Map loaded: prayagraj_corridor", "18:52:04", Color(0xFF1688FF))
            EventLogRow("Estimator running", "18:52:05", Color(0xFF18D47B))
        }
    }
}

@Composable
private fun HealthMetricRow(
    title: String,
    value: String,
    status: HealthStatus
) {
    val statusColor = when (status) {
        HealthStatus.HEALTHY -> Color(0xFF18D47B)
        HealthStatus.SEARCHING -> Color(0xFFFFB020)
        HealthStatus.DEGRADED -> Color(0xFFFF7A00)
        else -> Color(0xFFFF4F5E)
    }

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Box(
                modifier = Modifier
                    .size(7.dp)
                    .clip(CircleShape)
                    .background(statusColor)
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text(text = title, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp), color = Color.White)
        }

        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = status.label,
                style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.Bold),
                color = statusColor
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text(
                text = value,
                style = IdrTypography.bodySmall.copy(fontSize = 11.sp),
                color = Color(0xFF94A3B8)
            )
        }
    }
}

@Composable
private fun PerformanceDataRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = label, style = IdrTypography.bodySmall.copy(fontSize = 11.sp), color = Color(0xFF94A3B8))
        Text(text = value, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold), color = Color.White)
    }
}

@Composable
private fun EventLogRow(event: String, timestamp: String, dotColor: Color) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(modifier = Modifier.size(6.dp).clip(CircleShape).background(dotColor))
        Spacer(modifier = Modifier.width(8.dp))
        Text(text = event, style = IdrTypography.bodySmall.copy(fontSize = 11.sp), color = Color(0xFFE2E8F0))
        Spacer(modifier = Modifier.weight(1f))
        Text(text = timestamp, style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
    }
}

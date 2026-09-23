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
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.components.MetricCard
import org.isro.idr.ui.state.NavigationUiState

@Composable
fun DiagnosticsScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
    val diag = uiState.diagnostics

    Column(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(
            text = "ENGINEERING & INGESTION DIAGNOSTICS",
            style = IdrTypography.labelSmall,
            color = IdrColors.SaffronAccent,
            fontWeight = FontWeight.Bold
        )

        // Sensor Ingestion Rates
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            MetricCard(
                label = "ACCEL RATE",
                primaryValue = String.format("%.1f", diag?.accelRateHz ?: 0.0),
                primaryUnit = "Hz",
                secondaryValue = "Target: 100 Hz",
                valueColor = if ((diag?.accelRateHz ?: 0.0) > 80.0) IdrColors.StatusGreen else IdrColors.StatusAmber,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "GYRO RATE",
                primaryValue = String.format("%.1f", diag?.gyroRateHz ?: 0.0),
                primaryUnit = "Hz",
                secondaryValue = "Target: 100 Hz",
                valueColor = if ((diag?.gyroRateHz ?: 0.0) > 80.0) IdrColors.StatusGreen else IdrColors.StatusAmber,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "MAG / BARO RATE",
                primaryValue = String.format("%.1f / %.1f", diag?.magRateHz ?: 0.0, diag?.baroRateHz ?: 0.0),
                primaryUnit = "Hz",
                secondaryValue = "Mag 25Hz / Baro 10Hz",
                valueColor = IdrColors.TextPrimary,
                modifier = Modifier.weight(1.2f)
            )
        }

        // Bounded Sensor Queue Diagnostics
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            MetricCard(
                label = "QUEUE OCCUPANCY",
                primaryValue = "${diag?.queueOccupancy ?: 0}",
                primaryUnit = "/ 500",
                secondaryValue = if (diag?.isQueueSaturated == true) "SATURATED" else "NORMAL",
                valueColor = if (diag?.isQueueSaturated == true) IdrColors.StatusRed else IdrColors.StatusGreen,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "DROPPED SAMPLES",
                primaryValue = "${diag?.droppedSensorEvents ?: 0}",
                primaryUnit = "events",
                secondaryValue = "Queue overflow drops",
                valueColor = if ((diag?.droppedSensorEvents ?: 0L) > 0) IdrColors.StatusAmber else IdrColors.StatusGreen,
                modifier = Modifier.weight(1f)
            )
        }

        // GNSS & Native Status
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            MetricCard(
                label = "GNSS FIX AGE",
                primaryValue = String.format("%.2f", uiState.gnssStatus.ageSeconds),
                primaryUnit = "s",
                secondaryValue = if (uiState.gnssStatus.fixValid) "Valid (<1.5s)" else "Outage",
                valueColor = if (uiState.gnssStatus.fixValid && uiState.gnssStatus.ageSeconds <= 1.5) IdrColors.StatusGreen else IdrColors.StatusAmber,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "NATIVE STATUS",
                primaryValue = uiState.nativeStatus.name,
                primaryUnit = "code ${uiState.nativeStatus.code}",
                secondaryValue = "idr-core C++ status",
                valueColor = if (uiState.nativeStatus.isSuccess) IdrColors.StatusGreen else IdrColors.StatusRed,
                modifier = Modifier.weight(1f)
            )
        }

        // Ingestion Anomaly Breakdown
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(6.dp))
                .background(IdrColors.NavyCard)
                .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(6.dp))
                .padding(10.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text(
                    text = "TIMESTAMP ANOMALIES",
                    style = IdrTypography.labelSmall,
                    color = IdrColors.TextSecondary
                )
                Text(
                    text = "Duplicate Timestamps: ${diag?.duplicateTimestamps ?: 0}",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary
                )
                Text(
                    text = "Backwards Timestamps: ${diag?.backwardsTimestamps ?: 0}",
                    style = IdrTypography.bodyMedium,
                    color = if ((diag?.backwardsTimestamps ?: 0L) > 0) IdrColors.StatusAmber else IdrColors.TextPrimary
                )
                Text(
                    text = "Large Time Gaps (>100ms): ${diag?.largeTimeGaps ?: 0}",
                    style = IdrTypography.bodyMedium,
                    color = if ((diag?.largeTimeGaps ?: 0L) > 0) IdrColors.StatusAmber else IdrColors.TextPrimary
                )
            }
        }
    }
}

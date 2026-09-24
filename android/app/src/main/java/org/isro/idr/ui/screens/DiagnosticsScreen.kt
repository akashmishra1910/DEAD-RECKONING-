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
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.components.MetricCard
import org.isro.idr.ui.state.NavigationUiState

/**
 * Diagnostics & Engineering Ingestion Metrics Screen.
 * Visualizes live hardware sensor rates, bounded buffer occupancies, timestamp anomalies,
 * and GNSS lock metrics to ensure zero-data-loss validation.
 */
@Composable
fun DiagnosticsScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
    val diag = uiState.diagnostics

    Column(
        modifier = modifier
            .fillMaxSize()
            .shadow(2.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(14.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Text(
                text = "ENGINEERING & INGESTION DIAGNOSTICS",
                style = IdrTypography.labelSmall,
                color = IdrColors.SaffronPrimary,
                fontWeight = FontWeight.Bold
            )
            Text(
                text = "LIVE HARDWARE TELEMETRY",
                style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                color = IdrColors.TextTertiary
            )
        }

        // Sensor Ingestion Rates
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            val accelHz = diag?.accelMeasuredHz ?: 0.0
            val gyroHz = diag?.gyroMeasuredHz ?: 0.0

            MetricCard(
                label = "ACCEL RATE",
                primaryValue = String.format("%.1f", accelHz),
                primaryUnit = "Hz",
                secondaryValue = "Target: 100 Hz",
                valueColor = if (accelHz > 40.0) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "GYRO RATE",
                primaryValue = String.format("%.1f", gyroHz),
                primaryUnit = "Hz",
                secondaryValue = "Target: 100 Hz",
                valueColor = if (gyroHz > 40.0) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "MAG / BARO",
                primaryValue = "${if (diag?.isMagnetometerAvailable == true) "ON" else "OFF"} / ${if (diag?.isBarometerAvailable == true) "ON" else "OFF"}",
                primaryUnit = "",
                secondaryValue = "Hardware state",
                valueColor = IdrColors.NavyDarkest,
                modifier = Modifier.weight(1.1f)
            )
        }

        // Bounded Sensor Queue Diagnostics
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            val isQueueSaturated = (diag?.queueOccupancy ?: 0) >= (diag?.queueCapacity ?: 500) * 0.8
            MetricCard(
                label = "QUEUE OCCUPANCY",
                primaryValue = "${diag?.queueOccupancy ?: 0}",
                primaryUnit = "/ ${diag?.queueCapacity ?: 500}",
                secondaryValue = if (isQueueSaturated) "SATURATED" else "NORMAL BUFFER",
                valueColor = if (isQueueSaturated) IdrColors.StatusRed else IdrColors.StatusGreenDark,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "DROPPED SAMPLES",
                primaryValue = "${diag?.queueOverflowDrops ?: 0}",
                primaryUnit = "events",
                secondaryValue = "Queue overflow drops",
                valueColor = if ((diag?.queueOverflowDrops ?: 0L) > 0) IdrColors.StatusAmberDark else IdrColors.StatusGreenDark,
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
                secondaryValue = if (uiState.gnssStatus.fixValid) "Valid (<1.5s)" else "Searching / Outage",
                valueColor = if (uiState.gnssStatus.fixValid && uiState.gnssStatus.ageSeconds <= 1.5) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "NATIVE STATUS",
                primaryValue = uiState.nativeStatus.name,
                primaryUnit = "code ${uiState.nativeStatus.code}",
                secondaryValue = "idr-core C++ engine",
                valueColor = if (uiState.nativeStatus.isSuccess) IdrColors.StatusGreenDark else IdrColors.StatusRed,
                modifier = Modifier.weight(1f)
            )
        }

        // Ingestion Anomaly & Health Breakdown
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(8.dp))
                .background(IdrColors.SurfaceSubtle)
                .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
                .padding(12.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text(
                    text = "TIMESTAMP ANOMALIES & EXECUTION STATS",
                    style = IdrTypography.labelSmall,
                    color = IdrColors.TextSecondary
                )
                Text(
                    text = "Accel Dropped / Duplicates: ${diag?.accelDroppedCount ?: 0}",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary
                )
                Text(
                    text = "Gyro Dropped / Duplicates: ${diag?.gyroDroppedCount ?: 0}",
                    style = IdrTypography.bodyMedium,
                    color = if ((diag?.gyroDroppedCount ?: 0L) > 0) IdrColors.StatusAmberDark else IdrColors.TextPrimary
                )
                Text(
                    text = "Large Time Gaps (>100ms): ${ (diag?.accelLargeGapCount ?: 0L) + (diag?.gyroLargeGapCount ?: 0L) }",
                    style = IdrTypography.bodyMedium,
                    color = if (((diag?.accelLargeGapCount ?: 0L) + (diag?.gyroLargeGapCount ?: 0L)) > 0) IdrColors.StatusAmberDark else IdrColors.TextPrimary
                )
                Text(
                    text = "Total Steps Processed: ${diag?.totalStepsProcessed ?: 0}  (Step Latency: ${diag?.lastStepDurationMicros ?: 0} µs)",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.NavBlueDark,
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

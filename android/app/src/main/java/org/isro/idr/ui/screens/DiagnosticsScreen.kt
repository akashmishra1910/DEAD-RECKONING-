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
 * Visualizes live hardware sensor rates (callback vs consumed), bounded buffer occupancies,
 * inter-sample timing statistics, GNSS lock & NavIC metrics, and native C++ step latencies.
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
                text = "ENGINEERING & SENSOR INGESTION DIAGNOSTICS",
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

        // Section 1: Raw Callback Rate vs Navigation Consumed Rate
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            val accelCbHz = diag?.accelCallbackHz ?: 0.0
            val accelConsHz = diag?.accelConsumedHz ?: 0.0

            MetricCard(
                label = "ACCEL RATES",
                primaryValue = String.format("%.1f", accelConsHz),
                primaryUnit = "Hz",
                secondaryValue = String.format("Raw HW: %.1f Hz", accelCbHz),
                valueColor = if (accelConsHz in 70.0..130.0) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
                modifier = Modifier.weight(1f)
            )

            val gyroCbHz = diag?.gyroCallbackHz ?: 0.0
            val gyroConsHz = diag?.gyroConsumedHz ?: 0.0

            MetricCard(
                label = "GYRO RATES",
                primaryValue = String.format("%.1f", gyroConsHz),
                primaryUnit = "Hz",
                secondaryValue = String.format("Raw HW: %.1f Hz", gyroCbHz),
                valueColor = if (gyroConsHz in 70.0..130.0) IdrColors.StatusGreenDark else IdrColors.NavBlueDark,
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

        // Section 2: Bounded Sensor Queue & Dropped Samples
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            val queueOcc = diag?.queueOccupancy ?: 0
            val queueCap = diag?.queueCapacity ?: 500
            val isQueueSaturated = queueOcc >= queueCap * 0.8

            MetricCard(
                label = "QUEUE OCCUPANCY",
                primaryValue = "$queueOcc",
                primaryUnit = "/ $queueCap",
                secondaryValue = if (isQueueSaturated) "SATURATED" else "NORMAL BUFFER",
                valueColor = if (isQueueSaturated) IdrColors.StatusRed else IdrColors.StatusGreenDark,
                modifier = Modifier.weight(1f)
            )

            val drops = diag?.queueOverflowDrops ?: 0L
            MetricCard(
                label = "DROPPED SAMPLES",
                primaryValue = "$drops",
                primaryUnit = "events",
                secondaryValue = "Queue overflow drops",
                valueColor = if (drops > 0) IdrColors.StatusAmberDark else IdrColors.StatusGreenDark,
                modifier = Modifier.weight(1f)
            )
        }

        // Section 3: GNSS Tracking & Fix State
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            val fixState = diag?.gnssFixState ?: "SEARCHING"
            val fixAgeS = if ((diag?.gnssFixAgeMs ?: -1L) >= 0) (diag?.gnssFixAgeMs ?: 0L) / 1000.0 else -1.0

            MetricCard(
                label = "GNSS FIX STATE",
                primaryValue = fixState,
                primaryUnit = if (fixAgeS >= 0) String.format("(%.1fs)", fixAgeS) else "",
                secondaryValue = "1σ Acc: ${if (diag?.horizontalAccuracyM ?: 0f > 0f) String.format("%.1fm", diag?.horizontalAccuracyM) else "N/A"}",
                valueColor = when (fixState) {
                    "FIXED" -> IdrColors.StatusGreenDark
                    "STALE" -> IdrColors.StatusAmberDark
                    "OUTAGE" -> IdrColors.StatusRed
                    else -> IdrColors.NavBlueDark
                },
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "SATELLITE TELEMETRY",
                primaryValue = "${diag?.totalSatCount ?: 0} / ${diag?.totalInViewCount ?: 0}",
                primaryUnit = "sats",
                secondaryValue = "NavIC: ${diag?.navicSatCount ?: 0} used / ${diag?.navicInViewCount ?: 0} view",
                valueColor = if ((diag?.navicSatCount ?: 0) > 0) IdrColors.StatusGreenDark else IdrColors.NavyDarkest,
                modifier = Modifier.weight(1.1f)
            )
        }

        // Section 4: Map Matching & Navigation Estimator
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            MetricCard(
                label = "MAP CONFIDENCE",
                primaryValue = uiState.mapConfidence,
                primaryUnit = if (uiState.candidateCount > 0) "(${uiState.candidateCount} cands)" else "",
                secondaryValue = "Hyps: ${uiState.hypothesisCount} | Margin: ${String.format("%.2f", uiState.scoreMargin)}",
                valueColor = when (uiState.mapConfidence) {
                    "HIGH" -> IdrColors.StatusGreenDark
                    "MEDIUM" -> IdrColors.StatusAmberDark
                    "LOW" -> IdrColors.StatusOrangeDark
                    else -> IdrColors.TextTertiary
                },
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "NATIVE LATENCY",
                primaryValue = "${diag?.lastStepDurationMicros ?: 0}",
                primaryUnit = "µs/step",
                secondaryValue = "Steps: ${diag?.totalStepsProcessed ?: 0}",
                valueColor = if ((diag?.lastStepDurationMicros ?: 0) < 500) IdrColors.StatusGreenDark else IdrColors.StatusAmberDark,
                modifier = Modifier.weight(1f)
            )
        }

        // Section 5: Inter-sample Timing & Anomaly Breakdown
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(8.dp))
                .background(IdrColors.SurfaceSubtle)
                .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
                .padding(12.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(5.dp)) {
                Text(
                    text = "SENSOR TIMING & MONOTONICITY AUDIT",
                    style = IdrTypography.labelSmall,
                    color = IdrColors.TextSecondary
                )
                Text(
                    text = "Accel dt: Mean = ${String.format("%.2f", diag?.accelMeanDtMs ?: 0.0)} ms | Min = ${String.format("%.2f", diag?.accelMinDtMs ?: 0.0)} ms | Max = ${String.format("%.2f", diag?.accelMaxDtMs ?: 0.0)} ms | P95 = ${String.format("%.2f", diag?.accelP95DtMs ?: 0.0)} ms",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary
                )
                Text(
                    text = "Gyro dt: Mean = ${String.format("%.2f", diag?.gyroMeanDtMs ?: 0.0)} ms",
                    style = IdrTypography.bodyMedium,
                    color = IdrColors.TextPrimary
                )
                Text(
                    text = "Dropped / Duplicate Callbacks: Accel = ${diag?.accelDroppedCount ?: 0} | Gyro = ${diag?.gyroDroppedCount ?: 0}",
                    style = IdrTypography.bodyMedium,
                    color = if (((diag?.accelDroppedCount ?: 0L) + (diag?.gyroDroppedCount ?: 0L)) > 0) IdrColors.StatusAmberDark else IdrColors.StatusGreenDark
                )
                Text(
                    text = "Large Time Gaps (>100ms): ${ (diag?.accelLargeGapCount ?: 0L) + (diag?.gyroLargeGapCount ?: 0L) }",
                    style = IdrTypography.bodyMedium,
                    color = if (((diag?.accelLargeGapCount ?: 0L) + (diag?.gyroLargeGapCount ?: 0L)) > 0) IdrColors.StatusAmberDark else IdrColors.TextPrimary
                )
            }
        }
    }
}

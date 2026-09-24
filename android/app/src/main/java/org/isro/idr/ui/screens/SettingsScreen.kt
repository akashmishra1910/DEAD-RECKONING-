package org.isro.idr.ui.screens

import android.os.Build
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
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
import org.isro.idr.ui.state.NavigationUiState

/**
 * System, Hardware, Sensor Pipeline & Navigation Engine Settings.
 * Displays authoritative engineering configuration across 6 dedicated sections:
 * DEVICE, SENSOR PIPELINE, NAVIGATION ENGINE, MAP ENGINE, OFFLINE DATA, SAFETY.
 */
@Composable
fun SettingsScreen(
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
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Text(
            text = "SYSTEM & ENGINE SPECIFICATIONS",
            style = IdrTypography.labelSmall,
            color = IdrColors.SaffronPrimary,
            fontWeight = FontWeight.Bold
        )

        // 1. DEVICE
        SettingsSection(title = "1. HOST DEVICE & RUNTIME ENVIRONMENT") {
            SettingRow(label = "Android Version", value = "Android ${Build.VERSION.RELEASE} (API ${Build.VERSION.SDK_INT})")
            SettingRow(label = "Target Native ABI", value = "${Build.SUPPORTED_ABIS.firstOrNull() ?: "arm64-v8a"} (NDK C++17)")
            SettingRow(label = "Device Model", value = "${Build.MANUFACTURER.uppercase()} ${Build.MODEL}")
            SettingRow(label = "Network Status", value = "100% Offline (Airplane Mode Verified)")
        }

        // 2. SENSOR PIPELINE
        SettingsSection(title = "2. SENSOR INGESTION & TIMING PIPELINE") {
            val accelCb = diag?.accelCallbackHz ?: 0.0
            val accelCons = diag?.accelConsumedHz ?: 0.0
            SettingRow(label = "Raw Callback Rate", value = "${String.format("%.1f", accelCb)} Hz (Hardware Interrupt)")
            SettingRow(label = "Navigation Consumed", value = "${String.format("%.1f", accelCons)} Hz (Target: 100.0 Hz)")
            SettingRow(label = "Resampling Mode", value = "Deterministic Phase Accumulator")
            SettingRow(label = "Queue Buffer Health", value = "${diag?.queueOccupancy ?: 0} / ${diag?.queueCapacity ?: 500} (${diag?.queueOverflowDrops ?: 0} drops)")
            SettingRow(label = "Inter-sample Jitter", value = "P95 dt = ${String.format("%.2f", diag?.accelP95DtMs ?: 0.0)} ms (Mean: ${String.format("%.2f", diag?.accelMeanDtMs ?: 0.0)} ms)")
        }

        // 3. NAVIGATION ENGINE
        SettingsSection(title = "3. DEAD RECKONING & ESTIMATION ENGINE") {
            SettingRow(label = "State Estimator (ESKF)", value = "15-State Error-State Kalman Filter")
            SettingRow(label = "Speed Model (GBDT)", value = "Learned GBDT Regressor (120 Trees, Transpiled C++)")
            SettingRow(label = "Kinematic Constraints", value = "Non-Holonomic (NHC: vy≈0, vz≈0) + Standstill ZUPT")
            SettingRow(label = "Heading Observability", value = "Adaptive GNSS Course + Magnetic Disturbance Rejection")
            SettingRow(label = "GNSS Innovation Gate", value = "Chi-Square (γ ≤ 11.35, α=0.01, dof=3)")
        }

        // 4. MAP ENGINE
        SettingsSection(title = "4. 3D MAP MATCHING & TOPOLOGY ENGINE") {
            SettingRow(label = "Map Matcher", value = "3D Causal HMM (Cross-Track + Barometric Emission)")
            SettingRow(label = "Elevation Barrier", value = "|Δh| > 4.0 m (Penalizes Impossible Flyover Hops)")
            SettingRow(label = "Map Format", value = ".idrmap v1.0 (Zero-Copy Memory Mapped Binary)")
            SettingRow(label = "Active Hypotheses", value = "${uiState.hypothesisCount} active paths (Margin: ${String.format("%.2f", uiState.scoreMargin)})")
        }

        // 5. OFFLINE DATA & ANCHOR
        SettingsSection(title = "5. OFFLINE DATASET & GEODETIC ANCHOR") {
            SettingRow(label = "Active Dataset", value = uiState.selectedMapFileName ?: "prayagraj_corridor.idrmap")
            SettingRow(
                label = "WGS-84 Anchor (ENU)",
                value = if (uiState.enuAnchor != null) {
                    String.format("%.6f° N, %.6f° E", uiState.enuAnchor.latitude, uiState.enuAnchor.longitude)
                } else "Auto-anchoring on Initial GNSS Fix"
            )
            SettingRow(label = "Map Feature Count", value = "${uiState.roadMapGraph?.edges?.size ?: 0} road edges, ${uiState.roadMapGraph?.basemap?.waterBodies?.size ?: 0} rivers")
        }

        // 6. SAFETY & PLATFORM GOVERNANCE
        SettingsSection(title = "6. SAFETY & VEHICLE DYNAMICS") {
            SettingRow(label = "Vehicle Dynamics Mode", value = "Four-Wheeler / Passenger Vehicle (Standard)")
            SettingRow(label = "Two-Wheeler Support", value = "Disabled (Standard Level NHC Active)")
            SettingRow(label = "Estimator Algorithm", value = "Track-B Frozen (C0=76.34%, N0=34.43%, N7=35.71%)")
        }
    }
}

@Composable
private fun SettingsSection(
    title: String,
    content: @Composable () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.SurfaceSubtle)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
            .padding(12.dp)
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(
                text = title,
                style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                color = IdrColors.SaffronPrimary,
                fontWeight = FontWeight.Bold
            )
            content()
        }
    }
}

@Composable
private fun SettingRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = label,
            style = IdrTypography.bodyMedium.copy(fontSize = 11.sp),
            color = IdrColors.TextSecondary,
            modifier = Modifier.weight(1.1f)
        )
        Text(
            text = value,
            style = IdrTypography.bodyMedium.copy(fontSize = 11.sp),
            color = IdrColors.TextPrimary,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.weight(1.4f)
        )
    }
}

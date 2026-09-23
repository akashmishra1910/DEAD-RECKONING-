package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState

@Composable
fun SettingsScreen(
    uiState: NavigationUiState,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Text(
            text = "SYSTEM & HARDWARE SETTINGS",
            style = IdrTypography.headlineMedium,
            color = IdrColors.SaffronAccent,
            fontWeight = FontWeight.Bold
        )

        Box(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(6.dp))
                .background(IdrColors.NavyCard)
                .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(6.dp))
                .padding(12.dp)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                SettingRow(label = "IMU Target Rate", value = "100 Hz (SENSOR_DELAY_FASTEST)")
                SettingRow(label = "UI Presentation Rate", value = "10 Hz (Decoupled StateFlow)")
                SettingRow(label = "Estimator State", value = "15-State Error-State Kalman Filter")
                SettingRow(
                    label = "Geodetic Anchor",
                    value = if (uiState.enuAnchor != null) {
                        String.format("%.6f°, %.6f° (h=%.1fm)", uiState.enuAnchor.latitude, uiState.enuAnchor.longitude, uiState.enuAnchor.altitude)
                    } else "Awaiting 3D GNSS Fix"
                )
                SettingRow(label = "Two-Wheeler Support", value = "Disabled (Car / 4-Wheeler Mode)")
                SettingRow(label = "Target Architecture", value = "arm64-v8a (Release) / x86_64")
            }
        }
    }
}

@Composable
fun SettingRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = label,
            style = IdrTypography.bodyMedium,
            color = IdrColors.TextSecondary
        )
        Text(
            text = value,
            style = IdrTypography.bodyMedium,
            color = IdrColors.TextPrimary,
            fontWeight = FontWeight.Bold
        )
    }
}

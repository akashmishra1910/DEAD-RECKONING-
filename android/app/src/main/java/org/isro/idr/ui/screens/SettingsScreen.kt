package org.isro.idr.ui.screens

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
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
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
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.SubScreen

/**
 * Screen 12: Settings & Preferences.
 * Configures Navigation preferences, Map Appearance, Units & Display,
 * Offline Maps storage, and Subsystem links.
 */
@Composable
fun SettingsScreen(
    uiState: NavigationUiState,
    onFollowVehicleToggled: (Boolean) -> Unit,
    onHeadingUpToggled: (Boolean) -> Unit,
    on3dBuildingsToggled: (Boolean) -> Unit,
    onAutoRecenterToggled: (Boolean) -> Unit,
    onOpenSubScreen: (SubScreen) -> Unit,
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
            text = "Settings",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Subsystems Quick Launch Bar
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
                text = "Subsystems & Instruments",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                SubsystemQuickButton("Sensor Monitor", "📈", modifier = Modifier.weight(1f)) {
                    onOpenSubScreen(SubScreen.SENSOR_MONITOR)
                }
                SubsystemQuickButton("GNSS / NavIC", "🛰", modifier = Modifier.weight(1f)) {
                    onOpenSubScreen(SubScreen.GNSS_NAVIC_SKYVIEW)
                }
                SubsystemQuickButton("Offline Maps", "🗺", modifier = Modifier.weight(1f)) {
                    onOpenSubScreen(SubScreen.OFFLINE_MAPS_MANAGER)
                }
            }
        }

        // Section 1: Navigation Preferences
        SettingsSectionCard(title = "Navigation") {
            SettingsToggleRow("Follow Vehicle", uiState.followVehicle, onFollowVehicleToggled)
            SettingsToggleRow("Heading Up Mode", uiState.headingUpMode, onHeadingUpToggled)
            SettingsToggleRow("Show 3D Buildings", uiState.show3dBuildings, on3dBuildingsToggled)
            SettingsToggleRow("Auto Recenter", uiState.autoRecenter, onAutoRecenterToggled)
        }

        // Section 2: Map Appearance
        SettingsSectionCard(title = "Map Appearance") {
            SettingsValueRow("Theme", "Dark Aerospace")
            SettingsValueRow("Map Style", uiState.mapStyle.label)
            SettingsValueRow("Building Height", "Real Footprint (OSM)")
            SettingsValueRow("Show Terrain", "Enabled")
        }

        // Section 3: Units & Display
        SettingsSectionCard(title = "Units & Display") {
            SettingsValueRow("Speed Unit", uiState.speedUnit)
            SettingsValueRow("Distance Unit", uiState.distanceUnit)
            SettingsValueRow("Coordinate Format", uiState.coordinateFormat)
        }

        // Section 4: Offline Maps & Data
        SettingsSectionCard(title = "Offline Maps & Storage") {
            SettingsActionRow("Manage Offline Regions", "8 packs available") {
                onOpenSubScreen(SubScreen.OFFLINE_MAPS_MANAGER)
            }
            SettingsActionRow("Storage Settings", "1.74 GB used") {
                onOpenSubScreen(SubScreen.OFFLINE_MAPS_MANAGER)
            }
        }
    }
}

@Composable
private fun SubsystemQuickButton(
    title: String,
    icon: String,
    modifier: Modifier = Modifier,
    onClick: () -> Unit
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(10.dp))
            .background(Color(0xFF10263A))
            .border(1.dp, Color(0xFF17324A), RoundedCornerShape(10.dp))
            .clickable { onClick() }
            .padding(vertical = 10.dp, horizontal = 6.dp),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(text = icon, fontSize = 18.sp)
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = title,
                style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.SemiBold),
                color = Color.White
            )
        }
    }
}

@Composable
private fun SettingsSectionCard(
    title: String,
    content: @Composable () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(6.dp, RoundedCornerShape(14.dp))
            .clip(RoundedCornerShape(14.dp))
            .background(IdrColors.Surface)
            .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
            .padding(14.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(
            text = title,
            style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )
        content()
    }
}

@Composable
private fun SettingsToggleRow(
    title: String,
    isChecked: Boolean,
    onCheckedChange: (Boolean) -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = title, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp), color = Color.White)
        Switch(
            checked = isChecked,
            onCheckedChange = onCheckedChange,
            colors = SwitchDefaults.colors(
                checkedThumbColor = Color.White,
                checkedTrackColor = Color(0xFF1688FF),
                uncheckedThumbColor = Color(0xFF94A3B8),
                uncheckedTrackColor = Color(0xFF10263A)
            )
        )
    }
}

@Composable
private fun SettingsValueRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = label, style = IdrTypography.bodySmall.copy(fontSize = 12.sp), color = Color(0xFF94A3B8))
        Text(text = value, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold), color = Color.White)
    }
}

@Composable
private fun SettingsActionRow(label: String, subtitle: String, onClick: () -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clickable { onClick() }
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Column {
            Text(text = label, style = IdrTypography.bodyMedium.copy(fontSize = 12.sp), color = Color.White)
            Text(text = subtitle, style = IdrTypography.labelSmall.copy(fontSize = 10.sp), color = Color(0xFF64748B))
        }
        Text(text = "›", fontSize = 18.sp, color = Color(0xFF94A3B8), fontWeight = FontWeight.Bold)
    }
}

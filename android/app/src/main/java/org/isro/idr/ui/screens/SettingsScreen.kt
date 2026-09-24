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
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
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
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.map.TileCacheManager
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.SubScreen

/**
 * Screen 12: Interactive Settings & Preferences.
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
    var followVehicle by remember { mutableStateOf(uiState.followVehicle) }
    var headingUp by remember { mutableStateOf(uiState.headingUpMode) }
    var show3d by remember { mutableStateOf(uiState.show3dBuildings) }
    var twoWheelerMode by remember { mutableStateOf(false) }
    var navicAiding by remember { mutableStateOf(true) }
    var cacheClearedMessage by remember { mutableStateOf(false) }

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
            text = "Settings & Configuration",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Subsystems Quick Launch Bar
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

        // Section 1: Navigation & Estimator Modes
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
                text = "Navigation & Estimator Modes",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            SettingsSwitchRow("Auto-Follow Vehicle Cursor", followVehicle) {
                followVehicle = it
                onFollowVehicleToggled(it)
            }
            SettingsSwitchRow("Course-Up (Heading Rotation)", headingUp) {
                headingUp = it
                onHeadingUpToggled(it)
            }
            SettingsSwitchRow("3D Extruded Buildings Layer", show3d) {
                show3d = it
                on3dBuildingsToggled(it)
            }
            SettingsSwitchRow("Two-Wheeler Lean Compensation", twoWheelerMode) {
                twoWheelerMode = it
            }
            SettingsSwitchRow("NavIC (IRNSS) Constellation Aiding", navicAiding) {
                navicAiding = it
            }
        }

        // Section 2: Storage & Cache Management
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
                text = "Map Tile & Vector Storage",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Column {
                    Text(text = "Cached Offline Tiles", fontSize = 12.sp, color = Color.White)
                    Text(
                        text = String.format("%.2f MB stored locally", TileCacheManager.getCacheSizeMb()),
                        fontSize = 10.sp,
                        color = Color(0xFF94A3B8)
                    )
                }

                Button(
                    onClick = {
                        TileCacheManager.clearCache()
                        cacheClearedMessage = true
                    },
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF17324A)),
                    shape = RoundedCornerShape(8.dp)
                ) {
                    Text(
                        text = if (cacheClearedMessage) "Cleared ✓" else "Clear Cache",
                        fontSize = 11.sp,
                        color = if (cacheClearedMessage) Color(0xFF18D47B) else Color.White
                    )
                }
            }
        }

        // Section 3: About & PS Attribution
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(Color(0xFF0D2238))
                .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Text(text = "ISRO IDR • Version 2.4.0 (Production)", fontSize = 12.sp, fontWeight = FontWeight.Bold, color = Color.White)
            Text(text = "SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System", fontSize = 10.sp, color = Color(0xFFFFAB40))
            Text(text = "Mentored by SAC / ISRO (Apurv Vasal, Singamaneni Anusha, Ashutosh Dutt)", fontSize = 10.sp, color = Color(0xFF94A3B8))
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
            .background(Color(0xFF17324A))
            .clickable(onClick = onClick)
            .padding(vertical = 10.dp, horizontal = 6.dp),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(text = icon, fontSize = 16.sp)
            Spacer(modifier = Modifier.height(3.dp))
            Text(text = title, fontSize = 10.sp, color = Color.White, fontWeight = FontWeight.SemiBold)
        }
    }
}

@Composable
private fun SettingsSwitchRow(
    title: String,
    checked: Boolean,
    onCheckedChange: (Boolean) -> Unit
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = title, fontSize = 12.sp, color = Color.White)
        Switch(
            checked = checked,
            onCheckedChange = onCheckedChange,
            colors = SwitchDefaults.colors(
                checkedThumbColor = Color.White,
                checkedTrackColor = Color(0xFFFF8A00),
                uncheckedThumbColor = Color(0xFF64748B),
                uncheckedTrackColor = Color(0xFF17324A)
            )
        )
    }
}

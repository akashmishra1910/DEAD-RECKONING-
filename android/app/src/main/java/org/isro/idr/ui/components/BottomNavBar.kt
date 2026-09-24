package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
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
import org.isro.idr.ui.state.ScreenTab

/**
 * 5-Destination Dark Aerospace Bottom Navigation Bar:
 * - Navigate
 * - Map
 * - Trip
 * - Diagnose
 * - Settings
 */
@Composable
fun BottomNavBar(
    selectedTab: ScreenTab,
    onTabSelected: (ScreenTab) -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(16.dp, RoundedCornerShape(topStart = 16.dp, topEnd = 16.dp))
            .clip(RoundedCornerShape(topStart = 16.dp, topEnd = 16.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(topStart = 16.dp, topEnd = 16.dp))
            .padding(horizontal = 8.dp, vertical = 6.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceAround,
            verticalAlignment = Alignment.CenterVertically
        ) {
            NavTabItem(
                title = "Navigate",
                icon = "▲",
                isSelected = selectedTab == ScreenTab.NAVIGATION,
                onClick = { onTabSelected(ScreenTab.NAVIGATION) }
            )
            NavTabItem(
                title = "Map",
                icon = "◫",
                isSelected = selectedTab == ScreenTab.MAP_EXPLORER,
                onClick = { onTabSelected(ScreenTab.MAP_EXPLORER) }
            )
            NavTabItem(
                title = "Trip",
                icon = "∿",
                isSelected = selectedTab == ScreenTab.TRIP_ANALYTICS,
                onClick = { onTabSelected(ScreenTab.TRIP_ANALYTICS) }
            )
            NavTabItem(
                title = "Diagnose",
                icon = "⌖",
                isSelected = selectedTab == ScreenTab.DIAGNOSTICS,
                onClick = { onTabSelected(ScreenTab.DIAGNOSTICS) }
            )
            NavTabItem(
                title = "Settings",
                icon = "⚙",
                isSelected = selectedTab == ScreenTab.SETTINGS,
                onClick = { onTabSelected(ScreenTab.SETTINGS) }
            )
        }
    }
}

@Composable
private fun NavTabItem(
    title: String,
    icon: String,
    isSelected: Boolean,
    onClick: () -> Unit
) {
    val activeColor = Color(0xFF1688FF)
    val inactiveColor = Color(0xFF64748B)

    Column(
        modifier = Modifier
            .clip(RoundedCornerShape(8.dp))
            .clickable { onClick() }
            .padding(horizontal = 10.dp, vertical = 4.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center
    ) {
        Text(
            text = icon,
            fontSize = 18.sp,
            color = if (isSelected) activeColor else inactiveColor,
            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal
        )
        Spacer(modifier = Modifier.height(2.dp))
        Text(
            text = title,
            style = IdrTypography.labelSmall.copy(
                fontSize = 10.sp,
                fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Medium
            ),
            color = if (isSelected) activeColor else inactiveColor
        )
    }
}

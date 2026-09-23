package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
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
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.HealthStatus
import org.isro.idr.ui.state.SystemHealthState

@Composable
fun SystemHealthBar(
    health: SystemHealthState,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
            .padding(horizontal = 8.dp, vertical = 6.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceEvenly,
            verticalAlignment = Alignment.CenterVertically
        ) {
            HealthChip(label = "IMU", status = health.imuStatus)
            HealthChip(label = "GNSS", status = health.gnssStatus)
            HealthChip(label = "NAV", status = health.navStatus)
            HealthChip(label = "MAP", status = health.mapStatus)
            HealthChip(label = "NAVIC", status = health.navicStatus)
            HealthChip(label = "JNI", status = health.jniStatus)
        }
    }
}

@Composable
fun HealthChip(
    label: String,
    status: HealthStatus
) {
    val (dotColor, textColor) = when (status) {
        HealthStatus.HEALTHY -> Pair(IdrColors.StatusGreen, IdrColors.TextPrimary)
        HealthStatus.DEGRADED -> Pair(IdrColors.StatusAmber, IdrColors.StatusAmber)
        HealthStatus.UNAVAILABLE -> Pair(IdrColors.StatusRed, IdrColors.TextTertiary)
    }

    Row(
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(
            modifier = Modifier
                .size(7.dp)
                .clip(CircleShape)
                .background(dotColor)
        )
        Spacer(modifier = Modifier.width(4.dp))
        Text(
            text = label,
            style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
            color = textColor,
            fontWeight = FontWeight.Bold
        )
    }
}

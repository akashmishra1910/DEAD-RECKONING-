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
import androidx.compose.ui.draw.shadow
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
            .shadow(2.dp, RoundedCornerShape(10.dp))
            .clip(RoundedCornerShape(10.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(10.dp))
            .padding(horizontal = 10.dp, vertical = 7.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
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
        HealthStatus.HEALTHY -> Pair(IdrColors.StatusGreen, IdrColors.StatusGreenDark)
        HealthStatus.SEARCHING -> Pair(IdrColors.NavBlue, IdrColors.NavBlueDark)
        HealthStatus.DEGRADED -> Pair(IdrColors.StatusAmber, IdrColors.StatusAmberDark)
        HealthStatus.OFFLINE -> Pair(IdrColors.SlateMuted, IdrColors.TextTertiary)
        HealthStatus.UNAVAILABLE -> Pair(IdrColors.SlateMuted, IdrColors.TextTertiary)
        HealthStatus.ERROR -> Pair(IdrColors.StatusRed, IdrColors.StatusRed)
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
            style = IdrTypography.labelSmall.copy(fontSize = 11.sp),
            color = textColor,
            fontWeight = FontWeight.Bold
        )
    }
}

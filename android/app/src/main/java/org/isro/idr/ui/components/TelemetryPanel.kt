package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
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
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GeoCoordinates
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import kotlin.math.sqrt

@Composable
fun TelemetryPanel(
    telemetry: NavigationTelemetry?,
    enuAnchor: GeoCoordinates.Wgs84Coordinate?,
    gnssAccuracyM: Double,
    modifier: Modifier = Modifier
) {
    val speedKmh = telemetry?.speedKmh ?: 0.0
    val speedMps = if (telemetry != null) {
        sqrt(telemetry.velEnu.x * telemetry.velEnu.x + telemetry.velEnu.y * telemetry.velEnu.y + telemetry.velEnu.z * telemetry.velEnu.z)
    } else 0.0

    val headingDeg = telemetry?.headingDeg ?: 0.0
    val cardinal = getCardinalDirection(headingDeg)
    val pitchDeg = telemetry?.pitchDeg ?: 0.0
    val rollDeg = telemetry?.rollDeg ?: 0.0

    val enuX = telemetry?.posX ?: 0.0
    val enuY = telemetry?.posY ?: 0.0
    val enuZ = telemetry?.posZ ?: 0.0

    // Geodetic position if anchor exists
    val wgs84 = if (enuAnchor != null && telemetry != null) {
        GeoCoordinates.enuToWgs84(enuX, enuY, enuZ, enuAnchor)
    } else null

    Column(
        modifier = modifier
            .fillMaxWidth()
            .shadow(2.dp, RoundedCornerShape(10.dp))
            .clip(RoundedCornerShape(10.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(10.dp))
            .padding(12.dp)
    ) {
        // Section Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "NAVIGATION INSTRUMENTATION",
                style = IdrTypography.labelSmall,
                color = IdrColors.SaffronPrimary,
                fontWeight = FontWeight.Bold
            )
            Text(
                text = "10 Hz TELEMETRY HUD",
                style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                color = IdrColors.TextTertiary
            )
        }

        Spacer(modifier = Modifier.height(8.dp))

        // Row 1: Speed & Heading
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            MetricCard(
                label = "FORWARD SPEED",
                primaryValue = String.format("%.1f", speedKmh),
                primaryUnit = "km/h",
                secondaryValue = String.format("%.2f m/s", speedMps),
                valueColor = IdrColors.NavyDarkest,
                modifier = Modifier.weight(1f)
            )

            MetricCard(
                label = "AZIMUTH / HEADING",
                primaryValue = String.format("%05.1f°", headingDeg),
                primaryUnit = cardinal,
                secondaryValue = String.format("P: %+.1f°  R: %+.1f°", pitchDeg, rollDeg),
                valueColor = IdrColors.SaffronPrimary,
                modifier = Modifier.weight(1f)
            )
        }

        Spacer(modifier = Modifier.height(8.dp))

        // Row 2: Local ENU Position & Geodetic Accuracy
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            MetricCard(
                label = "LOCAL ENU POSITION",
                primaryValue = String.format("E:%+5.1f N:%+5.1f", enuX, enuY),
                primaryUnit = "m",
                secondaryValue = String.format("Up: %+5.1f m", enuZ),
                valueColor = IdrColors.NavBlueDark,
                modifier = Modifier.weight(1.15f)
            )

            MetricCard(
                label = "GNSS ACCURACY (1σ)",
                primaryValue = if (gnssAccuracyM > 0.0) String.format("%.1f", gnssAccuracyM) else "STANDBY",
                primaryUnit = if (gnssAccuracyM > 0.0) "m" else "",
                secondaryValue = if (wgs84 != null) {
                    String.format("%.5f, %.5f", wgs84.latitude, wgs84.longitude)
                } else "Anchor Locked",
                valueColor = if (gnssAccuracyM in 0.001..10.0) IdrColors.StatusGreenDark else IdrColors.StatusAmberDark,
                modifier = Modifier.weight(0.85f)
            )
        }
    }
}

@Composable
fun MetricCard(
    label: String,
    primaryValue: String,
    primaryUnit: String,
    secondaryValue: String? = null,
    valueColor: Color = IdrColors.NavyDarkest,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.SurfaceSubtle)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(8.dp))
            .padding(horizontal = 10.dp, vertical = 8.dp)
    ) {
        Column {
            Text(
                text = label,
                style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                color = IdrColors.TextSecondary
            )
            Spacer(modifier = Modifier.height(3.dp))
            Row(
                verticalAlignment = Alignment.Bottom
            ) {
                Text(
                    text = primaryValue,
                    style = IdrTypography.displayMedium.copy(fontSize = 19.sp),
                    color = valueColor,
                    fontWeight = FontWeight.Bold
                )
                if (primaryUnit.isNotEmpty()) {
                    Spacer(modifier = Modifier.width(4.dp))
                    Text(
                        text = primaryUnit,
                        style = IdrTypography.titleMedium.copy(fontSize = 11.sp),
                        color = IdrColors.TextSecondary,
                        modifier = Modifier.padding(bottom = 2.dp)
                    )
                }
            }
            if (secondaryValue != null) {
                Spacer(modifier = Modifier.height(2.dp))
                Text(
                    text = secondaryValue,
                    style = IdrTypography.bodyMedium.copy(fontSize = 11.sp),
                    color = IdrColors.TextTertiary
                )
            }
        }
    }
}

private fun getCardinalDirection(azimuthDeg: Double): String {
    val norm = ((azimuthDeg % 360.0) + 360.0) % 360.0
    return when {
        norm >= 337.5 || norm < 22.5 -> "N"
        norm >= 22.5 && norm < 67.5 -> "NE"
        norm >= 67.5 && norm < 112.5 -> "E"
        norm >= 112.5 && norm < 157.5 -> "SE"
        norm >= 157.5 && norm < 202.5 -> "S"
        norm >= 202.5 && norm < 247.5 -> "SW"
        norm >= 247.5 && norm < 292.5 -> "NW"
        else -> "NW"
    }
}

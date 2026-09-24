package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
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

/**
 * 4-Metric Instrument Strip:
 * - Speed (e.g. 42 km/h)
 * - Heading (e.g. 087°)
 * - Accuracy / Est. Error (e.g. ±1.6 m or ±8.3 m)
 * - Map Confidence (e.g. HIGH / GOOD)
 */
@Composable
fun SpeedHeadingHud(
    speedKmh: Double,
    headingDeg: Double,
    accuracyOrErrorM: Double,
    isDrMode: Boolean,
    mapConfidence: String,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(12.dp, RoundedCornerShape(14.dp))
            .clip(RoundedCornerShape(14.dp))
            .background(IdrColors.SurfaceElevated.copy(alpha = 0.94f))
            .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
            .padding(horizontal = 14.dp, vertical = 8.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            InstrumentMetricItem(
                value = "${speedKmh.toInt()}",
                unit = "km/h",
                label = "Speed"
            )
            InstrumentMetricItem(
                value = String.format("%03d°", headingDeg.toInt() % 360),
                unit = "",
                label = "Heading"
            )
            InstrumentMetricItem(
                value = "±${String.format("%.1f", accuracyOrErrorM)}",
                unit = "m",
                label = if (isDrMode) "Est. Error" else "Accuracy",
                valueColor = if (isDrMode) Color(0xFFFFB020) else Color(0xFF18D47B)
            )
            InstrumentMetricItem(
                value = mapConfidence,
                unit = "",
                label = "Map Conf.",
                valueColor = if (mapConfidence == "HIGH") Color(0xFF18D47B) else Color(0xFF00B8FF)
            )
        }
    }
}

@Composable
private fun InstrumentMetricItem(
    value: String,
    unit: String,
    label: String,
    valueColor: Color = Color.White
) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = value,
                style = IdrTypography.titleLarge.copy(
                    fontSize = 17.sp,
                    fontWeight = FontWeight.Black
                ),
                color = valueColor
            )
            if (unit.isNotEmpty()) {
                Text(
                    text = " $unit",
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = Color(0xFF94A3B8)
                )
            }
        }
        Text(
            text = label,
            style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
            color = Color(0xFF64748B)
        )
    }
}

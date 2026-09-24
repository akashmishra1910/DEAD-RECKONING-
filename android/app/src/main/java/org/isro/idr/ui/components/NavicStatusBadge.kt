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
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography

private data class NavicStatusStyle(
    val statusText: String,
    val statusColor: Color,
    val statusBg: Color,
    val statusBorder: Color
)

@Composable
fun NavicStatusBadge(
    totalSatellites: Int,
    navicInView: Int,
    navicUsed: Int,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(2.dp, RoundedCornerShape(10.dp))
            .clip(RoundedCornerShape(10.dp))
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(10.dp))
            .padding(10.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column {
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(4.dp))
                            .background(IdrColors.SaffronPrimary)
                            .padding(horizontal = 6.dp, vertical = 2.dp)
                    ) {
                        Text(
                            text = "ISRO / NavIC",
                            style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                            color = Color.White,
                            fontWeight = FontWeight.Bold
                        )
                    }
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(
                        text = "CONSTELLATION MONITOR (IRNSS)",
                        style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                        color = IdrColors.TextSecondary
                    )
                }

                Spacer(modifier = Modifier.height(6.dp))

                Row(
                    horizontalArrangement = Arrangement.spacedBy(14.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "NavIC in View: $navicInView",
                        style = IdrTypography.bodyMedium.copy(fontSize = 12.sp),
                        color = if (navicInView > 0) IdrColors.SaffronPrimary else IdrColors.TextTertiary,
                        fontWeight = if (navicInView > 0) FontWeight.Bold else FontWeight.Normal
                    )
                    Text(
                        text = "NavIC in Fix: $navicUsed",
                        style = IdrTypography.bodyMedium.copy(fontSize = 12.sp),
                        color = if (navicUsed > 0) IdrColors.StatusGreenDark else IdrColors.TextTertiary,
                        fontWeight = if (navicUsed > 0) FontWeight.Bold else FontWeight.Normal
                    )
                    Text(
                        text = "Total Sats: $totalSatellites",
                        style = IdrTypography.bodyMedium.copy(fontSize = 12.sp),
                        color = IdrColors.TextSecondary
                    )
                }
            }

            // Status Indicator Pill
            val style = when {
                navicUsed > 0 -> NavicStatusStyle(
                    "NavIC LOCKED",
                    IdrColors.StatusGreenDark,
                    IdrColors.StatusGreenBg,
                    IdrColors.StatusGreen.copy(alpha = 0.5f)
                )
                navicInView > 0 -> NavicStatusStyle(
                    "NavIC VISIBLE",
                    IdrColors.StatusOrangeDark,
                    IdrColors.StatusOrangeBg,
                    IdrColors.StatusOrange.copy(alpha = 0.5f)
                )
                else -> NavicStatusStyle(
                    "SEARCHING NavIC",
                    IdrColors.NavBlueDark,
                    IdrColors.StatusBlueBg,
                    IdrColors.NavBlue.copy(alpha = 0.5f)
                )
            }

            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(style.statusBg)
                    .border(1.dp, style.statusBorder, RoundedCornerShape(6.dp))
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = style.statusText,
                    color = style.statusColor,
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

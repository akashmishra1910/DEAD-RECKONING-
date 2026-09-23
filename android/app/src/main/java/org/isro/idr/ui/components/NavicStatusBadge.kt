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
import androidx.compose.foundation.layout.width
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
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavySurface)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
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
                            .clip(RoundedCornerShape(3.dp))
                            .background(IdrColors.SaffronAccent)
                            .padding(horizontal = 4.dp, vertical = 1.dp)
                    ) {
                        Text(
                            text = "ISRO / NavIC",
                            style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                            color = IdrColors.NavyDarkest,
                            fontWeight = FontWeight.Bold
                        )
                    }
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(
                        text = "CONSTELLATION MONITOR (IRNSS)",
                        style = IdrTypography.labelSmall,
                        color = IdrColors.TextSecondary
                    )
                }

                Spacer(modifier = Modifier.height(4.dp))

                Row(
                    horizontalArrangement = Arrangement.spacedBy(16.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "NavIC in View: $navicInView",
                        style = IdrTypography.bodyMedium,
                        color = if (navicInView > 0) IdrColors.SaffronLight else IdrColors.TextTertiary
                    )
                    Text(
                        text = "NavIC in Fix: $navicUsed",
                        style = IdrTypography.bodyMedium,
                        color = if (navicUsed > 0) IdrColors.StatusGreen else IdrColors.TextTertiary,
                        fontWeight = FontWeight.Bold
                    )
                    Text(
                        text = "Total Sats: $totalSatellites",
                        style = IdrTypography.bodyMedium,
                        color = IdrColors.TextSecondary
                    )
                }
            }

            // Status Indicator Pill
            val hasNavic = navicUsed > 0 || navicInView > 0
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(if (hasNavic) IdrColors.StatusGreenDim else IdrColors.NavyCard)
                    .border(
                        1.dp,
                        if (hasNavic) IdrColors.StatusGreen else IdrColors.NavyCardBorder,
                        RoundedCornerShape(4.dp)
                    )
                    .padding(horizontal = 6.dp, vertical = 3.dp)
            ) {
                Text(
                    text = if (navicUsed > 0) "NavIC LOCKED" else if (navicInView > 0) "NavIC VISIBLE" else "NO NavIC",
                    color = if (navicUsed > 0) IdrColors.StatusGreen else if (navicInView > 0) IdrColors.StatusAmber else IdrColors.TextTertiary,
                    style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

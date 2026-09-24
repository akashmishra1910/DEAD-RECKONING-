package org.isro.idr.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
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

/**
 * Floating Next Turn Guidance Card (e.g. "350 m Turn right Mahatma Gandhi Marg").
 */
@Composable
fun NextTurnCard(
    distanceMeters: Int,
    turnDirection: String,
    roadName: String,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .shadow(12.dp, RoundedCornerShape(14.dp))
            .clip(RoundedCornerShape(14.dp))
            .background(IdrColors.SurfaceElevated.copy(alpha = 0.92f))
            .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
            .padding(horizontal = 14.dp, vertical = 10.dp)
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            Box(
                modifier = Modifier
                    .size(34.dp)
                    .clip(CircleShape)
                    .background(Color(0xFF1688FF).copy(alpha = 0.2f))
                    .border(1.dp, Color(0xFF1688FF), CircleShape),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = if (turnDirection.contains("left", ignoreCase = true)) "↰" else "↱",
                    fontSize = 18.sp,
                    color = Color(0xFF1688FF),
                    fontWeight = FontWeight.Bold
                )
            }

            Column {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(
                        text = "$distanceMeters m",
                        style = IdrTypography.titleMedium.copy(
                            fontSize = 15.sp,
                            fontWeight = FontWeight.Black
                        ),
                        color = Color.White
                    )
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(
                        text = turnDirection,
                        style = IdrTypography.labelMedium.copy(fontSize = 12.sp),
                        color = Color(0xFF94A3B8)
                    )
                }
                Text(
                    text = roadName,
                    style = IdrTypography.bodySmall.copy(
                        fontSize = 11.sp,
                        fontWeight = FontWeight.SemiBold
                    ),
                    color = Color(0xFFE2E8F0)
                )
            }
        }
    }
}

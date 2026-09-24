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
 * Floating Dynamic Next Turn Guidance & Active Trip Details Card.
 */
@Composable
fun NextTurnCard(
    distanceMeters: Int,
    turnDirection: String,
    roadName: String,
    turnIcon: String = "↱",
    isTripActive: Boolean = false,
    remainingDistanceMeters: Double = 0.0,
    estimatedMinutes: Int = 0,
    onEndTrip: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(12.dp, RoundedCornerShape(14.dp))
            .clip(RoundedCornerShape(14.dp))
            .background(Color(0xFF0D2238).copy(alpha = 0.96f))
            .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
            .padding(horizontal = 14.dp, vertical = 10.dp)
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(10.dp),
                    modifier = Modifier.weight(1f)
                ) {
                    Box(
                        modifier = Modifier
                            .size(36.dp)
                            .clip(CircleShape)
                            .background(Color(0xFF1688FF).copy(alpha = 0.25f))
                            .border(1.2.dp, Color(0xFF1688FF), CircleShape),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = turnIcon,
                            fontSize = 18.sp,
                            color = Color(0xFF1688FF),
                            fontWeight = FontWeight.Bold
                        )
                    }

                    Column {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Text(
                                text = if (distanceMeters >= 1000) String.format("%.1f km", distanceMeters / 1000.0) else "$distanceMeters m",
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
                                color = Color(0xFFCBD5E1),
                                maxLines = 1
                            )
                        }
                        Text(
                            text = roadName,
                            style = IdrTypography.bodySmall.copy(
                                fontSize = 11.sp,
                                fontWeight = FontWeight.SemiBold
                            ),
                            color = Color(0xFFFFAB40),
                            maxLines = 1
                        )
                    }
                }

                if (isTripActive) {
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(6.dp))
                            .background(Color(0xFF4A1F1F))
                            .border(1.dp, Color(0xFFFF4F5E).copy(alpha = 0.6f), RoundedCornerShape(6.dp))
                            .clickable { onEndTrip?.invoke() }
                            .padding(horizontal = 8.dp, vertical = 4.dp)
                    ) {
                        Text(
                            text = "END TRIP ✕",
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color(0xFFFF4F5E)
                        )
                    }
                }
            }

            // If a trip is active, show remaining distance and ETA
            if (isTripActive && remainingDistanceMeters > 0) {
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .clip(RoundedCornerShape(6.dp))
                        .background(Color(0xFF06111F))
                        .padding(horizontal = 8.dp, vertical = 4.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = "Remaining: ${String.format("%.2f km", remainingDistanceMeters / 1000.0)}",
                        fontSize = 10.sp,
                        color = Color(0xFF94A3B8),
                        fontWeight = FontWeight.SemiBold
                    )
                    Text(
                        text = "ETA: ${estimatedMinutes} min",
                        fontSize = 10.sp,
                        color = Color(0xFF18D47B),
                        fontWeight = FontWeight.Bold
                    )
                }
            }
        }
    }
}

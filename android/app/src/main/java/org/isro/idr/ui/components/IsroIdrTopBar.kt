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
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.GnssStatusState
import org.isro.idr.ui.state.NavigationUiState

/**
 * Production Aerospace Top Navigation Bar for ISRO IDR.
 * Redesigned for visual excellence, featuring:
 * - Stylish Orange Emblem Badge ("IDR" on #FF8A00)
 * - "ISRO IDR" Title & Aerospace Subtitle
 * - Live NavIC Satellite & IMU Rate Pills
 * - Dynamic GNSS / Dead Reckoning Status Pill
 */
@Composable
fun IsroIdrTopBar(
    uiState: NavigationUiState,
    onOpenDiagnostics: (() -> Unit)? = null,
    onOpenNavicSkyView: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val isDr = uiState.isDrActive || !uiState.gnssStatus.fixValid
    val isReacquired = uiState.isGnssReacquired

    val statusPillColor = when {
        isDr -> Color(0xFFFF7A00) // Saffron / Orange
        isReacquired -> Color(0xFF00B8FF) // Cyan
        else -> Color(0xFF18D47B) // Green
    }

    val statusPillBg = when {
        isDr -> Color(0xFF4A1F00)
        isReacquired -> Color(0xFF00385A)
        else -> Color(0xFF08331E)
    }

    val statusLabel = when {
        isDr -> "DR ACTIVE"
        isReacquired -> "REACQUIRED"
        else -> "GNSS LOCKED"
    }

    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(12.dp, RoundedCornerShape(14.dp))
            .clip(RoundedCornerShape(14.dp))
            .background(
                Brush.verticalGradient(
                    colors = listOf(
                        Color(0xFF0D2238),
                        Color(0xFF06111F)
                    )
                )
            )
            .border(1.2.dp, Color(0xFF1E3A5A), RoundedCornerShape(14.dp))
            .padding(horizontal = 12.dp, vertical = 9.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            // Left: Stylish Orange "IDR" Emblem Badge + Title & Subtitle
            Row(
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.weight(1f)
            ) {
                // Stylish Orange Badge
                Box(
                    modifier = Modifier
                        .size(38.dp)
                        .shadow(6.dp, RoundedCornerShape(9.dp))
                        .clip(RoundedCornerShape(9.dp))
                        .background(
                            Brush.linearGradient(
                                colors = listOf(
                                    Color(0xFFFF8A00),
                                    Color(0xFFE65100)
                                )
                            )
                        )
                        .border(1.5.dp, Color(0xFFFFD54F), RoundedCornerShape(9.dp)),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = "IDR",
                        color = Color.White,
                        fontSize = 15.sp,
                        fontWeight = FontWeight.Black,
                        fontFamily = FontFamily.SansSerif,
                        letterSpacing = 0.5.sp
                    )
                }

                Spacer(modifier = Modifier.width(10.dp))

                Column {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text(
                            text = "ISRO IDR",
                            style = IdrTypography.titleMedium.copy(
                                fontSize = 16.sp,
                                fontWeight = FontWeight.ExtraBold,
                                letterSpacing = 0.8.sp
                            ),
                            color = Color.White
                        )
                        Spacer(modifier = Modifier.width(6.dp))
                        Box(
                            modifier = Modifier
                                .clip(RoundedCornerShape(3.dp))
                                .background(Color(0xFF10263A))
                                .padding(horizontal = 5.dp, vertical = 1.dp)
                        ) {
                            Text(
                                text = "PS 26168",
                                color = Color(0xFFFFB020),
                                fontSize = 8.sp,
                                fontWeight = FontWeight.Bold
                            )
                        }
                    }
                    Text(
                        text = "Intelligent Dead Reckoning System",
                        style = IdrTypography.bodySmall.copy(fontSize = 10.sp),
                        color = Color(0xFF94A3B8)
                    )
                }
            }

            // Right: Real-Time Live Status Indicators
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                // NavIC / IRNSS Satellite Pill
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(6.dp))
                        .background(Color(0xFF0F2618))
                        .border(1.dp, Color(0xFF18D47B).copy(alpha = 0.6f), RoundedCornerShape(6.dp))
                        .then(if (onOpenNavicSkyView != null) Modifier.clickable { onOpenNavicSkyView() } else Modifier)
                        .padding(horizontal = 7.dp, vertical = 4.dp)
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Box(
                            modifier = Modifier
                                .size(6.dp)
                                .clip(CircleShape)
                                .background(Color(0xFFFF8A00))
                        )
                        Spacer(modifier = Modifier.width(4.dp))
                        Text(
                            text = "NavIC: ${uiState.gnssStatus.navicSatellitesUsed}",
                            style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.Bold),
                            color = Color(0xFFE2E8F0)
                        )
                    }
                }

                // Active Mode Pill (GNSS / DR / RECOVERY)
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(6.dp))
                        .background(statusPillBg)
                        .border(1.dp, statusPillColor.copy(alpha = 0.8f), RoundedCornerShape(6.dp))
                        .then(if (onOpenDiagnostics != null) Modifier.clickable { onOpenDiagnostics() } else Modifier)
                        .padding(horizontal = 8.dp, vertical = 4.dp)
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Box(
                            modifier = Modifier
                                .size(6.dp)
                                .clip(CircleShape)
                                .background(statusPillColor)
                        )
                        Spacer(modifier = Modifier.width(5.dp))
                        Text(
                            text = statusLabel,
                            style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.ExtraBold),
                            color = statusPillColor
                        )
                    }
                }
            }
        }
    }
}

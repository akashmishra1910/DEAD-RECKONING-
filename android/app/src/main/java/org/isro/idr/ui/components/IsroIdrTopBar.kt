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
import org.isro.idr.ui.state.NavigationUiState

/**
 * Production Aerospace Top Navigation Bar for ISRO IDR.
 * Features:
 * - Stylish Orange Emblem Badge ("IDR" on #FF8A00)
 * - "ISRO IDR" Title & Aerospace Subtitle
 * - Live NavIC Satellite & IMU Rate Pills
 * - Seamless Multi-State GNSS / DR Status Indicator
 */
@Composable
fun IsroIdrTopBar(
    uiState: NavigationUiState,
    onOpenDiagnostics: (() -> Unit)? = null,
    onOpenNavicSkyView: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val isSearching = uiState.gnssStatus.isSearching && !uiState.gnssStatus.fixValid
    val isDr = uiState.isDrActive || (!uiState.gnssStatus.fixValid && !isSearching)
    val isReacquired = uiState.isGnssReacquired

    val statusPillColor = when {
        isDr -> Color(0xFFFF7A00) // Saffron / Orange
        isSearching -> Color(0xFF00B8FF) // Cyan
        isReacquired -> Color(0xFF00E5FF) // Electric Cyan
        else -> Color(0xFF18D47B) // Green
    }

    val statusPillBg = when {
        isDr -> Color(0xFF4A1F00)
        isSearching -> Color(0xFF002942)
        isReacquired -> Color(0xFF00385A)
        else -> Color(0xFF08331E)
    }

    val statusLabel = when {
        isDr -> "DR ACTIVE"
        isSearching -> "SEARCHING"
        isReacquired -> "REACQUIRED"
        else -> "GNSS LOCKED"
    }

    Box(
        modifier = modifier
            .fillMaxWidth()
            .shadow(10.dp, RoundedCornerShape(14.dp))
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
            .padding(horizontal = 10.dp, vertical = 8.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            // Left: Orange "IDR" Emblem Badge + Title & Subtitle
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                // Stylish Orange Badge
                Box(
                    modifier = Modifier
                        .size(36.dp)
                        .shadow(6.dp, RoundedCornerShape(9.dp), spotColor = Color(0xFFFF8A00))
                        .clip(RoundedCornerShape(9.dp))
                        .background(
                            Brush.linearGradient(
                                colors = listOf(
                                    Color(0xFFFF8A00),
                                    Color(0xFFFF6F00),
                                    Color(0xFFE65100)
                                )
                            )
                        )
                        .border(1.2.dp, Color.White.copy(alpha = 0.9f), RoundedCornerShape(9.dp)),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = "IDR",
                        fontSize = 14.sp,
                        fontWeight = FontWeight.Black,
                        fontFamily = FontFamily.SansSerif,
                        letterSpacing = 0.8.sp,
                        color = Color.White
                    )
                }

                Column {
                    Row(
                        verticalAlignment = Alignment.CenterVertically,
                        horizontalArrangement = Arrangement.spacedBy(5.dp)
                    ) {
                        Text(
                            text = "ISRO IDR",
                            style = IdrTypography.titleMedium.copy(
                                fontSize = 15.sp,
                                fontWeight = FontWeight.ExtraBold,
                                letterSpacing = 0.5.sp
                            ),
                            color = Color.White
                        )

                        // PS Badge Pill (Single line, compact)
                        Box(
                            modifier = Modifier
                                .clip(RoundedCornerShape(4.dp))
                                .background(Color(0xFF17324A))
                                .border(0.8.dp, Color(0xFF254B6E), RoundedCornerShape(4.dp))
                                .padding(horizontal = 4.dp, vertical = 1.dp)
                        ) {
                            Text(
                                text = "PS 26168",
                                fontSize = 8.5.sp,
                                fontWeight = FontWeight.Bold,
                                color = Color(0xFFFFD54F),
                                fontFamily = FontFamily.Monospace
                            )
                        }
                    }

                    Spacer(modifier = Modifier.height(1.dp))

                    Text(
                        text = "Intelligent Dead Reckoning",
                        style = IdrTypography.bodySmall.copy(
                            fontSize = 9.5.sp,
                            fontWeight = FontWeight.Medium
                        ),
                        color = Color(0xFF94A3B8)
                    )
                }
            }

            // Right: Status Badges (NavIC + Mode Indicator)
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(5.dp)
            ) {
                // NavIC Satellite Pill
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(16.dp))
                        .background(Color(0xFF0D2B1E))
                        .border(0.9.dp, Color(0xFF18D47B).copy(alpha = 0.6f), RoundedCornerShape(16.dp))
                        .clickable { onOpenNavicSkyView?.invoke() }
                        .padding(horizontal = 6.dp, vertical = 3.dp)
                ) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically,
                        horizontalArrangement = Arrangement.spacedBy(3.dp)
                    ) {
                        Box(
                            modifier = Modifier
                                .size(5.dp)
                                .clip(CircleShape)
                                .background(Color(0xFFFFB020))
                        )
                        Text(
                            text = "NavIC: ${uiState.gnssStatus.navicSatellitesUsed}",
                            fontSize = 9.5.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color.White
                        )
                    }
                }

                // Active Mode Pill
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(16.dp))
                        .background(statusPillBg)
                        .border(0.9.dp, statusPillColor.copy(alpha = 0.8f), RoundedCornerShape(16.dp))
                        .clickable { onOpenDiagnostics?.invoke() }
                        .padding(horizontal = 6.dp, vertical = 3.dp)
                ) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically,
                        horizontalArrangement = Arrangement.spacedBy(3.dp)
                    ) {
                        Box(
                            modifier = Modifier
                                .size(5.dp)
                                .clip(CircleShape)
                                .background(statusPillColor)
                        )
                        Text(
                            text = statusLabel,
                            fontSize = 9.5.sp,
                            fontWeight = FontWeight.Bold,
                            color = statusPillColor
                        )
                    }
                }
            }
        }
    }
}

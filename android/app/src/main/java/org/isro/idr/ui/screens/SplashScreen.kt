package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.components.IdrCoverBanner

/**
 * Screen 1: Splash / Welcome.
 * Premium Dark Aerospace Branding & Mission Presentation for ISRO IDR.
 * Features stylish Orange IDR Cover Banner and mission launch trigger.
 */
@Composable
fun SplashScreen(
    onGetStarted: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxSize()
            .background(
                Brush.verticalGradient(
                    colors = listOf(
                        Color(0xFF040B14),
                        Color(0xFF06111F),
                        Color(0xFF0B1B2B),
                        Color(0xFF10263A)
                    )
                )
            )
            .padding(20.dp)
    ) {
        Column(
            modifier = Modifier.fillMaxSize(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.SpaceBetween
        ) {
            // Top Header: Official Cover Banner
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(top = 16.dp)
            ) {
                IdrCoverBanner(
                    modifier = Modifier.fillMaxWidth(),
                    isCompact = false
                )
            }

            // Central Mission Overview Cards
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 4.dp),
                verticalArrangement = Arrangement.spacedBy(10.dp)
            ) {
                Text(
                    text = "Navigate Beyond GNSS",
                    style = IdrTypography.titleLarge.copy(
                        fontSize = 22.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 0.5.sp
                    ),
                    color = Color.White,
                    textAlign = TextAlign.Center
                )

                Text(
                    text = "Autonomous AI/ML Dead Reckoning & 3D Map Matching\nfor seamless navigation through tunnels, flyovers, & urban canyons.",
                    style = IdrTypography.bodyMedium.copy(
                        fontSize = 13.sp,
                        fontWeight = FontWeight.Normal,
                        lineHeight = 18.sp
                    ),
                    color = Color(0xFF94A3B8),
                    textAlign = TextAlign.Center
                )

                Spacer(modifier = Modifier.height(6.dp))

                // Feature Highlights Row
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    FeatureHighlightPill(
                        title = "10 Hz / 200 Hz",
                        subtitle = "Mobile & Edge Core",
                        modifier = Modifier.weight(1f)
                    )
                    FeatureHighlightPill(
                        title = "<10% Drift",
                        subtitle = "PS26168 Benchmark",
                        modifier = Modifier.weight(1f)
                    )
                    FeatureHighlightPill(
                        title = "NavIC / IRNSS",
                        subtitle = "Multi-Constellation",
                        modifier = Modifier.weight(1f)
                    )
                }
            }

            // Bottom CTA Button
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 16.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Button(
                    onClick = onGetStarted,
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(54.dp)
                        .shadow(12.dp, RoundedCornerShape(14.dp)),
                    shape = RoundedCornerShape(14.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFFFF8A00) // Saffron action CTA
                    )
                ) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically,
                        horizontalArrangement = Arrangement.Center
                    ) {
                        Text(
                            text = "Launch Navigation Engine",
                            style = IdrTypography.titleMedium.copy(
                                fontSize = 16.sp,
                                fontWeight = FontWeight.Bold
                            ),
                            color = Color.White
                        )
                        Spacer(modifier = Modifier.width(8.dp))
                        Text(
                            text = "→",
                            style = IdrTypography.titleMedium.copy(
                                fontSize = 18.sp,
                                fontWeight = FontWeight.Bold
                            ),
                            color = Color.White
                        )
                    }
                }

                Spacer(modifier = Modifier.height(10.dp))

                Text(
                    text = "Indian Space Research Organisation • Department of Space",
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = Color(0xFF64748B)
                )
            }
        }
    }
}

@Composable
private fun FeatureHighlightPill(
    title: String,
    subtitle: String,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .shadow(4.dp, RoundedCornerShape(10.dp))
            .background(Color(0xFF0B1B2B))
            .padding(horizontal = 8.dp, vertical = 8.dp),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = title,
                style = IdrTypography.labelMedium.copy(fontSize = 11.sp, fontWeight = FontWeight.ExtraBold),
                color = Color(0xFFFFD54F)
            )
            Text(
                text = subtitle,
                style = IdrTypography.bodySmall.copy(fontSize = 9.sp),
                color = Color(0xFF94A3B8)
            )
        }
    }
}

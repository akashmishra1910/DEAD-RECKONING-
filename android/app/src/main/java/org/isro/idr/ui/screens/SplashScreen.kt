package org.isro.idr.ui.screens

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography

/**
 * Screen 1: Splash / Welcome.
 * Premium Dark Aerospace Branding & Mission Presentation for ISRO IDR.
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
            .padding(24.dp)
    ) {
        // Subtle Aerospace Particle / Orbital Canvas Background
        Canvas(modifier = Modifier.fillMaxSize()) {
            val w = size.width
            val h = size.height
            // Orbit Arc 1
            drawArc(
                color = Color(0xFF1688FF).copy(alpha = 0.15f),
                startAngle = 180f,
                sweepAngle = 140f,
                useCenter = false,
                topLeft = Offset(-w * 0.4f, h * 0.2f),
                size = androidx.compose.ui.geometry.Size(w * 1.8f, h * 0.9f),
                style = Stroke(width = 1.5f)
            )
            // Orbit Arc 2 (Saffron)
            drawArc(
                color = Color(0xFFFF8A00).copy(alpha = 0.18f),
                startAngle = 210f,
                sweepAngle = 100f,
                useCenter = false,
                topLeft = Offset(-w * 0.2f, h * 0.35f),
                size = androidx.compose.ui.geometry.Size(w * 1.4f, h * 0.7f),
                style = Stroke(width = 2.0f)
            )
        }

        Column(
            modifier = Modifier.fillMaxSize(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.SpaceBetween
        ) {
            // Top ISRO Space Emblem Badge
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier.padding(top = 28.dp)
            ) {
                // Stylized ISRO Saffron-Blue Rocket Icon
                Box(
                    modifier = Modifier
                        .size(68.dp)
                        .shadow(12.dp, CircleShape)
                        .clip(CircleShape)
                        .background(Color(0xFF0B1B2B))
                        .border(1.5.dp, Color(0xFF17324A), CircleShape),
                    contentAlignment = Alignment.Center
                ) {
                    Canvas(modifier = Modifier.size(36.dp)) {
                        // Upward rocket / satellite ray
                        val path = Path().apply {
                            moveTo(size.width * 0.5f, 2f)
                            lineTo(size.width * 0.85f, size.height * 0.75f)
                            lineTo(size.width * 0.5f, size.height * 0.55f)
                            lineTo(size.width * 0.15f, size.height * 0.75f)
                            close()
                        }
                        drawPath(path, color = Color(0xFFFF8A00))
                        drawCircle(
                            color = Color(0xFF1688FF),
                            radius = 4f,
                            center = Offset(size.width * 0.5f, size.height * 0.88f)
                        )
                    }
                }

                Spacer(modifier = Modifier.height(10.dp))

                Text(
                    text = "ISRO",
                    style = IdrTypography.titleLarge.copy(
                        fontSize = 18.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 4.sp
                    ),
                    color = Color(0xFFFF8A00)
                )
            }

            // Central Mission Typography & Globe Visual
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier.padding(horizontal = 16.dp)
            ) {
                Text(
                    text = "IDR",
                    style = IdrTypography.displayLarge.copy(
                        fontSize = 54.sp,
                        fontWeight = FontWeight.Black,
                        letterSpacing = 6.sp
                    ),
                    color = Color.White
                )

                Text(
                    text = "Intelligent Dead Reckoning\nNavigation",
                    style = IdrTypography.titleMedium.copy(
                        fontSize = 16.sp,
                        fontWeight = FontWeight.Medium,
                        lineHeight = 22.sp
                    ),
                    color = Color(0xFF94A3B8),
                    textAlign = TextAlign.Center
                )

                Spacer(modifier = Modifier.height(14.dp))

                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(20.dp))
                        .background(Color(0xFF10263A).copy(alpha = 0.85f))
                        .border(1.dp, Color(0xFF17324A), RoundedCornerShape(20.dp))
                        .padding(horizontal = 14.dp, vertical = 6.dp)
                ) {
                    Text(
                        text = "SIH26168",
                        style = IdrTypography.labelMedium.copy(
                            fontSize = 12.sp,
                            fontWeight = FontWeight.Bold,
                            letterSpacing = 2.sp
                        ),
                        color = Color(0xFF00B8FF)
                    )
                }

                Spacer(modifier = Modifier.height(28.dp))

                // Stylized Glowing Earth / Globe Arc
                Box(
                    modifier = Modifier
                        .size(190.dp)
                        .clip(CircleShape)
                        .background(
                            Brush.radialGradient(
                                colors = listOf(
                                    Color(0xFF1688FF).copy(alpha = 0.35f),
                                    Color(0xFF0B1B2B).copy(alpha = 0.9f),
                                    Color(0xFF06111F)
                                )
                            )
                        )
                        .border(1.5.dp, Color(0xFF1688FF).copy(alpha = 0.4f), CircleShape),
                    contentAlignment = Alignment.Center
                ) {
                    Canvas(modifier = Modifier.fillMaxSize()) {
                        val cx = size.width / 2f
                        val cy = size.height / 2f
                        val r = size.width / 2.2f
                        // Latitude lines
                        drawLine(Color(0xFF1688FF).copy(alpha = 0.3f), Offset(cx - r * 0.85f, cy - r * 0.4f), Offset(cx + r * 0.85f, cy - r * 0.4f), 1f)
                        drawLine(Color(0xFF1688FF).copy(alpha = 0.45f), Offset(cx - r * 0.98f, cy), Offset(cx + r * 0.98f, cy), 1.2f)
                        drawLine(Color(0xFF1688FF).copy(alpha = 0.3f), Offset(cx - r * 0.85f, cy + r * 0.4f), Offset(cx + r * 0.85f, cy + r * 0.4f), 1f)
                        // Meridian ellipse
                        drawOval(
                            color = Color(0xFFFF8A00).copy(alpha = 0.35f),
                            topLeft = Offset(cx - r * 0.45f, cy - r),
                            size = androidx.compose.ui.geometry.Size(r * 0.9f, r * 2f),
                            style = Stroke(width = 1.2f)
                        )
                    }
                    Text(
                        text = "GLOBAL\nOFFLINE",
                        style = IdrTypography.labelSmall.copy(
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Bold,
                            letterSpacing = 2.sp
                        ),
                        color = Color(0xFFE2E8F0),
                        textAlign = TextAlign.Center
                    )
                }

                Spacer(modifier = Modifier.height(24.dp))

                Text(
                    text = "Resilient Navigation.\nAnywhere. Anytime.",
                    style = IdrTypography.bodyLarge.copy(
                        fontSize = 15.sp,
                        fontWeight = FontWeight.SemiBold,
                        lineHeight = 20.sp
                    ),
                    color = Color(0xFFE2E8F0),
                    textAlign = TextAlign.Center
                )
            }

            // Bottom CTA Button
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 20.dp),
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
                        containerColor = Color(0xFF1688FF)
                    )
                ) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically,
                        horizontalArrangement = Arrangement.Center
                    ) {
                        Text(
                            text = "Get Started",
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

                Spacer(modifier = Modifier.height(12.dp))

                Text(
                    text = "Autonomous 15-State ESKF + GBDT Speed Inference Engine",
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = Color(0xFF64748B)
                )
            }
        }
    }
}

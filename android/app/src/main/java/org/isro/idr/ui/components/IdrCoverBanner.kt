package org.isro.idr.ui.components

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
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrTypography

/**
 * Official Cover Banner & Logo Badge for ISRO IDR.
 * Features stylish "IDR" typography on rich aerospace orange / saffron background
 * with celestial orbital tracks, satellite nodes, and ISRO metadata.
 */
@Composable
fun IdrCoverBanner(
    modifier: Modifier = Modifier,
    isCompact: Boolean = false
) {
    val bannerHeight = if (isCompact) 110.dp else 190.dp

    Box(
        modifier = modifier
            .fillMaxWidth()
            .height(bannerHeight)
            .shadow(16.dp, RoundedCornerShape(18.dp))
            .clip(RoundedCornerShape(18.dp))
            .background(
                Brush.linearGradient(
                    colors = listOf(
                        Color(0xFFE65100), // Deep aerospace orange
                        Color(0xFFFF8A00), // Vibrant ISRO saffron
                        Color(0xFFFF6D00), // Rich amber
                        Color(0xFFBF360C)  // Dark orbital bronze
                    ),
                    start = Offset(0f, 0f),
                    end = Offset(1000f, 600f)
                )
            )
            .border(
                width = 1.5.dp,
                brush = Brush.linearGradient(
                    colors = listOf(
                        Color(0xFFFFD54F),
                        Color(0xFFFFFFFF).copy(alpha = 0.6f),
                        Color(0xFFFF8A00),
                        Color(0xFFFFD54F)
                    )
                ),
                shape = RoundedCornerShape(18.dp)
            )
    ) {
        // Celestial Orbital Canvas Background
        Canvas(modifier = Modifier.fillMaxSize()) {
            val w = size.width
            val h = size.height

            // Orbital Ring 1
            drawArc(
                color = Color.White.copy(alpha = 0.25f),
                startAngle = -30f,
                sweepAngle = 220f,
                useCenter = false,
                topLeft = Offset(-w * 0.2f, -h * 0.4f),
                size = androidx.compose.ui.geometry.Size(w * 1.4f, h * 1.8f),
                style = Stroke(width = 2f)
            )

            // Orbital Ring 2 (Golden NavIC Ellipse)
            drawArc(
                color = Color(0xFFFFD54F).copy(alpha = 0.45f),
                startAngle = 30f,
                sweepAngle = 190f,
                useCenter = false,
                topLeft = Offset(w * 0.1f, -h * 0.6f),
                size = androidx.compose.ui.geometry.Size(w * 1.1f, h * 2.1f),
                style = Stroke(width = 2.5f)
            )

            // Constellation Satellite Points
            drawCircle(Color.White, radius = 3.5f, center = Offset(w * 0.15f, h * 0.35f))
            drawCircle(Color(0xFFFFD54F), radius = 4.5f, center = Offset(w * 0.85f, h * 0.25f))
            drawCircle(Color(0xFF18D47B), radius = 3.0f, center = Offset(w * 0.72f, h * 0.78f))
            drawCircle(Color.White, radius = 2.5f, center = Offset(w * 0.32f, h * 0.82f))

            // Indian Tricolor Accent Arc
            val path = Path().apply {
                moveTo(w * 0.05f, h - 8f)
                lineTo(w * 0.35f, h - 8f)
            }
            drawPath(path, Color(0xFF18D47B), style = Stroke(width = 4f, cap = StrokeCap.Round))
        }

        // Foreground Content Overlay
        Row(
            modifier = Modifier
                .fillMaxSize()
                .padding(horizontal = if (isCompact) 14.dp else 20.dp, vertical = if (isCompact) 10.dp else 16.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Column(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.Center
            ) {
                // Top Org Pill
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(4.dp))
                            .background(Color(0xFF06111F).copy(alpha = 0.75f))
                            .padding(horizontal = 7.dp, vertical = 3.dp)
                    ) {
                        Text(
                            text = "ISRO • SAC / DEPT OF SPACE",
                            color = Color(0xFFFFD54F),
                            fontSize = 9.sp,
                            fontWeight = FontWeight.ExtraBold,
                            letterSpacing = 1.sp
                        )
                    }
                    Spacer(modifier = Modifier.width(6.dp))
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(4.dp))
                            .background(Color.White.copy(alpha = 0.2f))
                            .padding(horizontal = 6.dp, vertical = 3.dp)
                    ) {
                        Text(
                            text = "PS 26168",
                            color = Color.White,
                            fontSize = 9.sp,
                            fontWeight = FontWeight.Bold
                        )
                    }
                }

                Spacer(modifier = Modifier.height(6.dp))

                // Title Typography
                Text(
                    text = "INTELLIGENT DEAD RECKONING",
                    color = Color.White,
                    style = IdrTypography.titleLarge.copy(
                        fontSize = if (isCompact) 15.sp else 19.sp,
                        fontWeight = FontWeight.Black,
                        letterSpacing = 0.8.sp
                    )
                )

                if (!isCompact) {
                    Spacer(modifier = Modifier.height(3.dp))
                    Text(
                        text = "AI-ML Real-Time Inertial & 3D Map Fusion Engine",
                        color = Color.White.copy(alpha = 0.9f),
                        fontSize = 11.sp,
                        fontWeight = FontWeight.Medium
                    )
                    Spacer(modifier = Modifier.height(4.dp))
                    Text(
                        text = "10 Hz Smartphone • 200 Hz Edge FOG Core • NavIC Aided",
                        color = Color(0xFFFFE082),
                        fontSize = 10.sp,
                        fontWeight = FontWeight.Bold
                    )
                }
            }

            Spacer(modifier = Modifier.width(12.dp))

            // Large Stylish "IDR" Emblem Badge
            Box(
                modifier = Modifier
                    .size(if (isCompact) 64.dp else 84.dp)
                    .shadow(12.dp, CircleShape)
                    .clip(CircleShape)
                    .background(
                        Brush.radialGradient(
                            colors = listOf(
                                Color(0xFF0F2B48),
                                Color(0xFF06111F)
                            )
                        )
                    )
                    .border(2.5.dp, Color(0xFFFFD54F), CircleShape),
                contentAlignment = Alignment.Center
            ) {
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    Text(
                        text = "IDR",
                        color = Color(0xFFFF8A00),
                        fontSize = if (isCompact) 22.sp else 30.sp,
                        fontWeight = FontWeight.Black,
                        fontFamily = FontFamily.SansSerif,
                        letterSpacing = 1.5.sp
                    )
                    Text(
                        text = "NAVIC",
                        color = Color(0xFF18D47B),
                        fontSize = if (isCompact) 7.sp else 9.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 1.2.sp
                    )
                }
            }
        }
    }
}

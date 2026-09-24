package org.isro.idr.ui.screens

import androidx.compose.animation.core.FastOutSlowInEasing
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
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
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.draw.scale
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.delay
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography

/**
 * Screen 1: Splash / Welcome Screen with dynamic animated logo & pipeline initialization sequence.
 */
@Composable
fun SplashScreen(
    onGetStarted: () -> Unit,
    modifier: Modifier = Modifier
) {
    // Pulse and Rotation Animations
    val infiniteTransition = rememberInfiniteTransition(label = "SplashAnimation")

    val pulseScale by infiniteTransition.animateFloat(
        initialValue = 0.95f,
        targetValue = 1.05f,
        animationSpec = infiniteRepeatable(
            animation = tween(1200, easing = FastOutSlowInEasing),
            repeatMode = RepeatMode.Reverse
        ),
        label = "PulseScale"
    )

    val haloAlpha by infiniteTransition.animateFloat(
        initialValue = 0.25f,
        targetValue = 0.70f,
        animationSpec = infiniteRepeatable(
            animation = tween(1400, easing = FastOutSlowInEasing),
            repeatMode = RepeatMode.Reverse
        ),
        label = "HaloAlpha"
    )

    val orbitRotation by infiniteTransition.animateFloat(
        initialValue = 0f,
        targetValue = 360f,
        animationSpec = infiniteRepeatable(
            animation = tween(6000, easing = LinearEasing),
            repeatMode = RepeatMode.Restart
        ),
        label = "OrbitRotation"
    )

    var loadingProgress by remember { mutableStateOf(0.15f) }
    var statusText by remember { mutableStateOf("Initializing 15-State ESKF Core...") }

    LaunchedEffect(Unit) {
        delay(400)
        loadingProgress = 0.40f
        statusText = "Calibrating 100 Hz IMU Pipeline..."
        delay(400)
        loadingProgress = 0.70f
        statusText = "Mounting Valhalla-Aligned OSM Road Network..."
        delay(400)
        loadingProgress = 0.90f
        statusText = "Syncing NavIC / GNSS Satellite Ephemeris..."
        delay(400)
        loadingProgress = 1.0f
        statusText = "IDR Navigation Engine Ready."
        delay(300)
        onGetStarted()
    }

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
        Column(
            modifier = Modifier.fillMaxSize(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.SpaceBetween
        ) {
            Spacer(modifier = Modifier.height(16.dp))

            // Central Animated Aerospace Emblem
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(20.dp)
            ) {
                Box(
                    modifier = Modifier.size(160.dp),
                    contentAlignment = Alignment.Center
                ) {
                    // Outer Pulsing Glow
                    Box(
                        modifier = Modifier
                            .size(150.dp)
                            .scale(pulseScale)
                            .clip(CircleShape)
                            .background(
                                Brush.radialGradient(
                                    colors = listOf(
                                        Color(0xFFFF8A00).copy(alpha = haloAlpha),
                                        Color(0xFFFF6F00).copy(alpha = haloAlpha * 0.4f),
                                        Color.Transparent
                                    )
                                )
                            )
                    )

                    // Rotating Orbit Ring
                    Box(
                        modifier = Modifier
                            .size(136.dp)
                            .rotate(orbitRotation)
                            .border(1.5.dp, Color(0xFFFFAB40).copy(alpha = 0.6f), CircleShape)
                    ) {
                        // NavIC Satellite Node
                        Box(
                            modifier = Modifier
                                .size(10.dp)
                                .align(Alignment.TopCenter)
                                .clip(CircleShape)
                                .background(Color(0xFFFFD54F))
                                .border(1.dp, Color.White, CircleShape)
                        )
                    }

                    // Main IDR Emblem Box
                    Box(
                        modifier = Modifier
                            .size(96.dp)
                            .shadow(16.dp, RoundedCornerShape(26.dp), spotColor = Color(0xFFFF8A00))
                            .clip(RoundedCornerShape(26.dp))
                            .background(
                                Brush.linearGradient(
                                    colors = listOf(
                                        Color(0xFFFF8A00),
                                        Color(0xFFFF6F00),
                                        Color(0xFFE65100)
                                    )
                                )
                            )
                            .border(2.dp, Color.White.copy(alpha = 0.85f), RoundedCornerShape(26.dp)),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = "IDR",
                            fontSize = 32.sp,
                            fontWeight = FontWeight.Black,
                            fontFamily = FontFamily.SansSerif,
                            letterSpacing = 2.sp,
                            color = Color.White
                        )
                    }
                }

                // Title and Subtitle
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.spacedBy(6.dp)
                ) {
                    Text(
                        text = "ISRO IDR",
                        fontSize = 28.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 1.5.sp,
                        color = Color.White
                    )

                    Text(
                        text = "AI-ML INTELLIGENT DEAD RECKONING",
                        fontSize = 11.sp,
                        fontWeight = FontWeight.Bold,
                        letterSpacing = 2.sp,
                        color = Color(0xFFFFAB40)
                    )

                    Text(
                        text = "SIH PS 26168 • SAC / ISRO",
                        fontSize = 11.sp,
                        fontWeight = FontWeight.SemiBold,
                        letterSpacing = 1.sp,
                        color = Color(0xFF94A3B8)
                    )
                }
            }

            // Bottom Progress Sequence & Manual Skip CTA
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 12.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(14.dp)
            ) {
                Column(
                    modifier = Modifier.fillMaxWidth(),
                    verticalArrangement = Arrangement.spacedBy(6.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Text(
                            text = statusText,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Medium,
                            color = Color(0xFFCBD5E1)
                        )
                        Text(
                            text = "${(loadingProgress * 100).toInt()}%",
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color(0xFFFFAB40)
                        )
                    }

                    LinearProgressIndicator(
                        progress = { loadingProgress },
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(5.dp)
                            .clip(RoundedCornerShape(3.dp)),
                        color = Color(0xFFFF8A00),
                        trackColor = Color(0xFF17324A)
                    )
                }

                Button(
                    onClick = onGetStarted,
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(48.dp)
                        .shadow(8.dp, RoundedCornerShape(12.dp)),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFFFF8A00)
                    ),
                    shape = RoundedCornerShape(12.dp)
                ) {
                    Text(
                        text = "START NAVIGATION",
                        fontSize = 14.sp,
                        fontWeight = FontWeight.Bold,
                        letterSpacing = 1.sp,
                        color = Color.White
                    )
                }
            }
        }
    }
}

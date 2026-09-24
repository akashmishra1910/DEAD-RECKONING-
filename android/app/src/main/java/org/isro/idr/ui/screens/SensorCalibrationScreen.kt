package org.isro.idr.ui.screens

import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
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
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState

/**
 * Screen 2: Sensor Calibration & IMU Readiness.
 * Real-time IMU calibration checks, 3D gimbal ring level graphic,
 * 4-sensor checklist (Accel, Gyro, Mag, Baro), progress bar, and readiness CTA.
 */
@Composable
fun SensorCalibrationScreen(
    uiState: NavigationUiState,
    onCalibrationComplete: () -> Unit,
    onCancel: () -> Unit,
    modifier: Modifier = Modifier
) {
    val progress = uiState.calibrationProgress.coerceIn(0f, 1f)
    val isComplete = progress >= 0.99f

    val infiniteTransition = rememberInfiniteTransition(label = "ring_spin")
    val spinAngle by infiniteTransition.animateFloat(
        initialValue = 0f,
        targetValue = 360f,
        animationSpec = infiniteRepeatable(
            animation = tween(4000, easing = LinearEasing)
        ),
        label = "spin_angle"
    )

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .padding(18.dp),
        verticalArrangement = Arrangement.SpaceBetween,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // Top Header
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(top = 12.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(36.dp)
                    .clip(CircleShape)
                    .background(IdrColors.Surface)
                    .border(1.dp, IdrColors.CardBorder, CircleShape)
                    .clickable { onCancel() },
                contentAlignment = Alignment.Center
            ) {
                Text(text = "←", color = Color.White, fontSize = 16.sp, fontWeight = FontWeight.Bold)
            }

            Spacer(modifier = Modifier.weight(1f))

            Text(
                text = "Sensor Calibration",
                style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )

            Spacer(modifier = Modifier.weight(1f))
            Spacer(modifier = Modifier.size(36.dp))
        }

        // Central 3D Gimbal Ring & Device Outline Graphic
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier.padding(vertical = 12.dp)
        ) {
            Box(
                modifier = Modifier
                    .size(200.dp)
                    .clip(CircleShape)
                    .background(IdrColors.Surface)
                    .border(1.5.dp, IdrColors.CardBorder, CircleShape),
                contentAlignment = Alignment.Center
            ) {
                Canvas(modifier = Modifier.fillMaxSize()) {
                    val cx = size.width / 2f
                    val cy = size.height / 2f
                    val radius = size.width * 0.42f

                    // Outer level calibration arc
                    drawArc(
                        color = Color(0xFF18D47B),
                        startAngle = spinAngle,
                        sweepAngle = 260f,
                        useCenter = false,
                        topLeft = Offset(cx - radius, cy - radius),
                        size = androidx.compose.ui.geometry.Size(radius * 2, radius * 2),
                        style = Stroke(width = 3.5f, cap = StrokeCap.Round)
                    )

                    // Secondary Inner Cyan Arc
                    drawArc(
                        color = Color(0xFF00B8FF).copy(alpha = 0.5f),
                        startAngle = -spinAngle * 1.5f,
                        sweepAngle = 180f,
                        useCenter = false,
                        topLeft = Offset(cx - radius * 0.75f, cy - radius * 0.75f),
                        size = androidx.compose.ui.geometry.Size(radius * 1.5f, radius * 1.5f),
                        style = Stroke(width = 2.0f, cap = StrokeCap.Round)
                    )

                    // Center 3D Isometric Phone/Cradle Slate
                    val padW = 48f
                    val padH = 68f
                    val padPath = Path().apply {
                        moveTo(cx - padW, cy - padH * 0.5f)
                        lineTo(cx + padW * 0.3f, cy - padH)
                        lineTo(cx + padW, cy + padH * 0.4f)
                        lineTo(cx - padW * 0.3f, cy + padH * 0.9f)
                        close()
                    }
                    drawPath(padPath, color = Color(0xFF10263A))
                    drawPath(padPath, color = Color(0xFF1688FF), style = Stroke(width = 1.8f))
                    drawCircle(color = Color(0xFF18D47B), radius = 4.5f, center = Offset(cx, cy))
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            Text(
                text = "Place your device on a flat surface\nand keep it still",
                style = IdrTypography.bodyMedium.copy(fontSize = 13.sp, lineHeight = 18.sp),
                color = Color(0xFF94A3B8),
                textAlign = TextAlign.Center
            )
        }

        // 4-Sensor Checklist Panel
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            SensorCheckItem(
                name = "Accelerometer",
                status = if (uiState.isAccelCalibrated) "Ready" else "Calibrating...",
                isReady = uiState.isAccelCalibrated
            )
            SensorCheckItem(
                name = "Gyroscope",
                status = if (uiState.isGyroCalibrated) "Ready" else "Calibrating...",
                isReady = uiState.isGyroCalibrated
            )
            SensorCheckItem(
                name = "Magnetometer",
                status = if (uiState.isMagCalibrated) "Ready" else "Calibrating...",
                isReady = uiState.isMagCalibrated
            )
            SensorCheckItem(
                name = "Barometer",
                status = if (uiState.isBaroCalibrated) "Ready" else "Standby",
                isReady = uiState.isBaroCalibrated
            )
        }

        // Progress Bar & CTA
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 12.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = if (isComplete) "Calibration Complete" else "Calibrating IMU...",
                    style = IdrTypography.labelMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )
                Text(
                    text = "${(progress * 100).toInt()}%",
                    style = IdrTypography.labelMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold),
                    color = if (isComplete) Color(0xFF18D47B) else Color(0xFF00B8FF)
                )
            }

            Spacer(modifier = Modifier.height(6.dp))

            LinearProgressIndicator(
                progress = { progress },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(6.dp)
                    .clip(RoundedCornerShape(3.dp)),
                color = if (isComplete) Color(0xFF18D47B) else Color(0xFF00B8FF),
                trackColor = Color(0xFF10263A)
            )

            Spacer(modifier = Modifier.height(16.dp))

            Button(
                onClick = { if (isComplete) onCalibrationComplete() else onCancel() },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(50.dp)
                    .shadow(8.dp, RoundedCornerShape(12.dp)),
                shape = RoundedCornerShape(12.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (isComplete) Color(0xFF18D47B) else Color(0xFF10263A)
                )
            ) {
                Text(
                    text = if (isComplete) "Proceed to Navigation" else "Cancel",
                    style = IdrTypography.titleMedium.copy(fontSize = 15.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )
            }
        }
    }
}

@Composable
private fun SensorCheckItem(
    name: String,
    status: String,
    isReady: Boolean
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(
            modifier = Modifier
                .size(22.dp)
                .clip(CircleShape)
                .background(if (isReady) Color(0xFF18D47B).copy(alpha = 0.2f) else Color(0xFF00B8FF).copy(alpha = 0.2f))
                .border(1.dp, if (isReady) Color(0xFF18D47B) else Color(0xFF00B8FF), CircleShape),
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = if (isReady) "✓" else "⋯",
                color = if (isReady) Color(0xFF18D47B) else Color(0xFF00B8FF),
                fontSize = 11.sp,
                fontWeight = FontWeight.ExtraBold
            )
        }

        Spacer(modifier = Modifier.width(10.dp))

        Text(
            text = name,
            style = IdrTypography.bodyMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.SemiBold),
            color = Color.White
        )

        Spacer(modifier = Modifier.weight(1f))

        Text(
            text = status,
            style = IdrTypography.labelSmall.copy(fontSize = 11.sp),
            color = if (isReady) Color(0xFF18D47B) else Color(0xFF64748B)
        )
    }
}

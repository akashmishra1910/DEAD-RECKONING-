package org.isro.idr.ui.map

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint
import kotlin.math.cos
import kotlin.math.sin

/**
 * Tactical Grid Map Canvas.
 * Provides high-performance, deterministic, zero-network local ENU map rendering.
 * Draws metric grid, measured GNSS trajectory, estimated IDR dead-reckoning trajectory,
 * and live vehicle marker rotated by telemetry azimuth.
 */
@Composable
fun TacticalGridMapCanvas(
    telemetry: NavigationTelemetry?,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    trajectoryMode: TrajectoryDisplayMode,
    mapDataState: MapDataState,
    modifier: Modifier = Modifier
) {
    val vehicleX = telemetry?.posX ?: 0.0
    val vehicleY = telemetry?.posY ?: 0.0
    val vehicleHeadingDeg = telemetry?.azimuthDeg?.toFloat() ?: 0.0f

    Box(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(8.dp))
            .background(IdrColors.NavyDarkest)
            .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(8.dp))
    ) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val canvasWidth = size.width
            val canvasHeight = size.height
            val centerX = canvasWidth / 2f
            val centerY = canvasHeight / 2f

            // Auto-scale pixels per meter based on trajectory bounds
            val scale = calculateScale(
                vehicleX = vehicleX,
                vehicleY = vehicleY,
                idrTrajectory = idrTrajectory,
                gnssTrajectory = gnssTrajectory,
                canvasWidth = canvasWidth,
                canvasHeight = canvasHeight
            )

            // 1. Draw Grid Lines & Axes
            drawMetricGrid(centerX, centerY, vehicleX, vehicleY, scale, canvasWidth, canvasHeight)

            // 2. Draw GNSS Trajectory (Cyan) if enabled
            if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.GNSS_ONLY) {
                drawTrajectoryPath(
                    points = gnssTrajectory,
                    vehicleX = vehicleX,
                    vehicleY = vehicleY,
                    centerX = centerX,
                    centerY = centerY,
                    scale = scale,
                    color = IdrColors.TrajectoryGnss,
                    strokeWidth = 3f,
                    drawDots = true
                )
            }

            // 3. Draw IDR Trajectory (Orange/Saffron) if enabled
            if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.IDR_ONLY) {
                drawTrajectoryPath(
                    points = idrTrajectory,
                    vehicleX = vehicleX,
                    vehicleY = vehicleY,
                    centerX = centerX,
                    centerY = centerY,
                    scale = scale,
                    color = IdrColors.TrajectoryIdr,
                    strokeWidth = 4f,
                    drawDots = false
                )
            }

            // 4. Draw Vehicle Marker (Chevron rotated to azimuth)
            drawVehicleMarker(
                centerX = centerX,
                centerY = centerY,
                headingDeg = vehicleHeadingDeg
            )

            // 5. Draw North Arrow in top-right corner
            drawNorthIndicator(canvasWidth - 32.dp.toPx(), 32.dp.toPx())
        }

        // Offline Map Status Overlay Banner
        Box(
            modifier = Modifier
                .align(Alignment.BottomStart)
                .padding(8.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(IdrColors.NavySurface.copy(alpha = 0.85f))
                .border(1.dp, IdrColors.NavyCardBorder, RoundedCornerShape(4.dp))
                .padding(horizontal = 8.dp, vertical = 4.dp)
        ) {
            Text(
                text = when (mapDataState) {
                    MapDataState.AVAILABLE -> "MAP LAYER: LOCAL OSM ACTIVE"
                    MapDataState.LOADING -> "MAP LAYER: LOADING TILES..."
                    MapDataState.UNAVAILABLE -> "OFFLINE MAP DATA NOT INSTALLED"
                    MapDataState.OFFLINE -> "OFFLINE TACTICAL GRID (LOCAL ENU)"
                },
                style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                color = if (mapDataState == MapDataState.AVAILABLE) IdrColors.StatusGreen else IdrColors.TextSecondary,
                fontWeight = FontWeight.Bold
            )
        }
    }
}

private fun DrawScope.drawMetricGrid(
    centerX: Float,
    centerY: Float,
    vehicleX: Double,
    vehicleY: Double,
    scale: Float,
    width: Float,
    height: Float
) {
    val gridSizeMeters = when {
        scale > 5.0f -> 10f
        scale > 1.0f -> 50f
        scale > 0.2f -> 100f
        else -> 500f
    }
    val gridPx = gridSizeMeters * scale

    // World origin (0,0 ENU) on screen
    val originScreenX = centerX - (vehicleX * scale).toFloat()
    val originScreenY = centerY + (vehicleY * scale).toFloat() // North is Up (-Y in screen coords)

    val gridColor = IdrColors.NavyCardBorder.copy(alpha = 0.5f)
    val axisColor = IdrColors.TextTertiary.copy(alpha = 0.7f)

    // Vertical grid lines (Constant East)
    var x = originScreenX % gridPx
    if (x < 0) x += gridPx
    while (x < width) {
        drawLine(
            color = gridColor,
            start = Offset(x, 0f),
            end = Offset(x, height),
            strokeWidth = 1f
        )
        x += gridPx
    }

    // Horizontal grid lines (Constant North)
    var y = originScreenY % gridPx
    if (y < 0) y += gridPx
    while (y < height) {
        drawLine(
            color = gridColor,
            start = Offset(0f, y),
            end = Offset(width, y),
            strokeWidth = 1f
        )
        y += gridPx
    }

    // Major origin axes
    if (originScreenX in 0f..width) {
        drawLine(
            color = axisColor,
            start = Offset(originScreenX, 0f),
            end = Offset(originScreenX, height),
            strokeWidth = 1.5f
        )
    }
    if (originScreenY in 0f..height) {
        drawLine(
            color = axisColor,
            start = Offset(0f, originScreenY),
            end = Offset(width, originScreenY),
            strokeWidth = 1.5f
        )
    }
}

private fun DrawScope.drawTrajectoryPath(
    points: List<TrajectoryPoint>,
    vehicleX: Double,
    vehicleY: Double,
    centerX: Float,
    centerY: Float,
    scale: Float,
    color: Color,
    strokeWidth: Float,
    drawDots: Boolean
) {
    if (points.isEmpty()) return

    val path = Path()
    var isFirst = true

    for (pt in points) {
        val screenX = centerX + ((pt.enuX - vehicleX) * scale).toFloat()
        val screenY = centerY - ((pt.enuY - vehicleY) * scale).toFloat() // North up

        if (isFirst) {
            path.moveTo(screenX, screenY)
            isFirst = false
        } else {
            path.lineTo(screenX, screenY)
        }

        if (drawDots) {
            drawCircle(
                color = color,
                radius = 2.5f,
                center = Offset(screenX, screenY)
            )
        }
    }

    drawPath(
        path = path,
        color = color,
        style = Stroke(width = strokeWidth, cap = StrokeCap.Round)
    )
}

private fun DrawScope.drawVehicleMarker(
    centerX: Float,
    centerY: Float,
    headingDeg: Float
) {
    rotate(degrees = headingDeg, pivot = Offset(centerX, centerY)) {
        // Vehicle Chevron / Arrow
        val chevronPath = Path().apply {
            moveTo(centerX, centerY - 18f)        // Tip pointing up (North)
            lineTo(centerX + 12f, centerY + 14f)  // Bottom right wing
            lineTo(centerX, centerY + 6f)         // Center indent
            lineTo(centerX - 12f, centerY + 14f)  // Bottom left wing
            close()
        }

        // Glow ring
        drawCircle(
            color = IdrColors.SaffronAccent.copy(alpha = 0.25f),
            radius = 24f,
            center = Offset(centerX, centerY)
        )

        // Chevron body
        drawPath(
            path = chevronPath,
            color = IdrColors.VehicleMarker
        )
        drawPath(
            path = chevronPath,
            color = IdrColors.NavyDarkest,
            style = Stroke(width = 1.5f)
        )
    }
}

private fun DrawScope.drawNorthIndicator(
    posX: Float,
    posY: Float
) {
    val arrowSize = 16f
    val path = Path().apply {
        moveTo(posX, posY - arrowSize)
        lineTo(posX + 6f, posY + arrowSize * 0.5f)
        lineTo(posX, posY)
        lineTo(posX - 6f, posY + arrowSize * 0.5f)
        close()
    }
    drawPath(path = path, color = IdrColors.StatusRed)
}

private fun calculateScale(
    vehicleX: Double,
    vehicleY: Double,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    canvasWidth: Float,
    canvasHeight: Float
): Float {
    var minX = vehicleX
    var maxX = vehicleX
    var minY = vehicleY
    var maxY = vehicleY

    for (pt in idrTrajectory) {
        if (pt.enuX < minX) minX = pt.enuX
        if (pt.enuX > maxX) maxX = pt.enuX
        if (pt.enuY < minY) minY = pt.enuY
        if (pt.enuY > maxY) maxY = pt.enuY
    }

    val spanX = (maxX - minX).coerceAtLeast(40.0) // min 40m window
    val spanY = (maxY - minY).coerceAtLeast(40.0)

    val scaleX = (canvasWidth * 0.7f) / spanX.toFloat()
    val scaleY = (canvasHeight * 0.7f) / spanY.toFloat()

    return minOf(scaleX, scaleY).coerceIn(0.05f, 20.0f)
}

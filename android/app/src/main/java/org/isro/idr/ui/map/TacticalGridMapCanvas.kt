package org.isro.idr.ui.map

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint

/**
 * Tactical Grid Map Canvas.
 * Provides high-performance, deterministic, zero-network local ENU map rendering in modern light styling.
 * Draws metric grid, measured GNSS trajectory, estimated IDR dead-reckoning trajectory,
 * map-aided snapped position, and live vehicle marker rotated by telemetry azimuth.
 */
@Composable
fun TacticalGridMapCanvas(
    telemetry: NavigationTelemetry?,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    trajectoryMode: TrajectoryDisplayMode,
    mapDataState: MapDataState,
    isNorthUp: Boolean = true,
    userZoomFactor: Float = 1.0f,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val vehicleX = telemetry?.posX ?: 0.0
    val vehicleY = telemetry?.posY ?: 0.0
    val vehicleHeadingDeg = telemetry?.headingDeg?.toFloat() ?: 0.0f

    // Pan offsets in pixels
    var panOffsetX by remember { mutableFloatStateOf(0f) }
    var panOffsetY by remember { mutableFloatStateOf(0f) }

    Box(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.MapCanvasBg)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .pointerInput(Unit) {
                detectDragGestures { change, dragAmount ->
                    change.consume()
                    panOffsetX += dragAmount.x
                    panOffsetY += dragAmount.y
                }
            }
    ) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val canvasWidth = size.width
            val canvasHeight = size.height
            val centerX = canvasWidth / 2f + panOffsetX
            val centerY = canvasHeight / 2f + panOffsetY

            // Calculate scale with user zoom multiplier
            val baseScale = calculateScale(
                vehicleX = vehicleX,
                vehicleY = vehicleY,
                idrTrajectory = idrTrajectory,
                gnssTrajectory = gnssTrajectory,
                canvasWidth = canvasWidth,
                canvasHeight = canvasHeight
            )
            val scale = (baseScale * userZoomFactor).coerceIn(0.05f, 50.0f)

            // Rotation angle: 0 for North-up, or -vehicleHeadingDeg for Heading-up
            val mapRotationDeg = if (isNorthUp) 0f else -vehicleHeadingDeg

            rotate(degrees = mapRotationDeg, pivot = Offset(centerX, centerY)) {
                // 1. Draw Grid Lines & Axes
                drawMetricGrid(centerX, centerY, vehicleX, vehicleY, scale, canvasWidth * 2f, canvasHeight * 2f)

                // 2. Draw GNSS Trajectory (Electric Blue) if enabled
                if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.GNSS_ONLY) {
                    drawTrajectoryPath(
                        points = gnssTrajectory,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        color = IdrColors.TrajectoryGnss,
                        strokeWidth = 3.5f,
                        drawDots = true
                    )
                }

                // 3. Draw IDR Trajectory (Vivid Saffron) if enabled
                if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.IDR_ONLY) {
                    drawTrajectoryPath(
                        points = idrTrajectory,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        color = IdrColors.TrajectoryIdr,
                        strokeWidth = 4.5f,
                        drawDots = false
                    )
                }

                // 4. Draw Map-Aided Trajectory (Royal Purple) if enabled
                if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.MAP_AIDED) {
                    drawTrajectoryPath(
                        points = mapAidedTrajectory,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        color = IdrColors.TrajectoryMap,
                        strokeWidth = 3.5f,
                        drawDots = false
                    )
                }

                // 5. Draw Vehicle Marker (Chevron rotated to azimuth)
                drawVehicleMarker(
                    centerX = centerX,
                    centerY = centerY,
                    headingDeg = if (isNorthUp) vehicleHeadingDeg else 0f
                )
            }

            // 6. Draw North Arrow in top-right corner
            drawNorthIndicator(
                posX = canvasWidth - 36.dp.toPx(),
                posY = 36.dp.toPx(),
                rotationDeg = if (isNorthUp) 0f else -vehicleHeadingDeg
            )

            // 7. Draw Distance Scale Bar at bottom right
            drawScaleBar(
                posX = canvasWidth - 110.dp.toPx(),
                posY = canvasHeight - 20.dp.toPx(),
                scale = scale
            )
        }

        // Top Left Status Badge
        Box(
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(10.dp)
                .shadow(2.dp, RoundedCornerShape(6.dp))
                .clip(RoundedCornerShape(6.dp))
                .background(IdrColors.SurfaceElevated.copy(alpha = 0.92f))
                .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(6.dp))
                .padding(horizontal = 8.dp, vertical = 4.dp)
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(
                    modifier = Modifier
                        .size(7.dp)
                        .clip(CircleShape)
                        .background(if (mapDataState == MapDataState.AVAILABLE || mapDataState == MapDataState.READY) IdrColors.StatusGreen else IdrColors.NavBlue)
                )
                Spacer(modifier = Modifier.width(5.dp))
                Text(
                    text = when (mapDataState) {
                        MapDataState.READY, MapDataState.AVAILABLE -> "LOCAL OSM GRAPH ACTIVE"
                        MapDataState.LOADING -> "LOADING GRAPH..."
                        MapDataState.UNAVAILABLE -> "OFFLINE ENU GRID"
                        MapDataState.OFFLINE -> "OFFLINE TACTICAL GRID"
                    },
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = IdrColors.TextPrimary,
                    fontWeight = FontWeight.Bold
                )
            }
        }

        // Floating Interactive Map Controls (Zoom +, Zoom -, Recenter, Compass)
        Column(
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .padding(end = 10.dp, bottom = 32.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            FloatingMapButton(
                text = "+",
                onClick = { onZoomIn?.invoke() }
            )
            FloatingMapButton(
                text = "−",
                onClick = { onZoomOut?.invoke() }
            )
            FloatingMapButton(
                text = if (isNorthUp) "N" else "H",
                onClick = { onToggleNorthUp?.invoke() }
            )
            FloatingMapButton(
                text = "⌖",
                onClick = {
                    panOffsetX = 0f
                    panOffsetY = 0f
                    onResetMap?.invoke()
                }
            )
        }
    }
}

@Composable
private fun FloatingMapButton(
    text: String,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .size(34.dp)
            .shadow(3.dp, CircleShape)
            .clip(CircleShape)
            .background(IdrColors.SurfaceElevated)
            .border(1.dp, IdrColors.CardBorder, CircleShape)
            .clickable { onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            style = IdrTypography.titleMedium.copy(fontSize = 15.sp),
            color = IdrColors.NavyDarkest,
            fontWeight = FontWeight.Bold
        )
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

    val gridColor = IdrColors.GridLineColor
    val axisColor = IdrColors.GridAxisColor

    val halfW = width
    val halfH = height

    // Vertical grid lines (Constant East)
    var x = originScreenX % gridPx
    while (x > -halfW) x -= gridPx
    while (x < halfW * 2) {
        drawLine(
            color = gridColor,
            start = Offset(x, -halfH),
            end = Offset(x, halfH * 2),
            strokeWidth = 1f
        )
        x += gridPx
    }

    // Horizontal grid lines (Constant North)
    var y = originScreenY % gridPx
    while (y > -halfH) y -= gridPx
    while (y < halfH * 2) {
        drawLine(
            color = gridColor,
            start = Offset(-halfW, y),
            end = Offset(halfW * 2, y),
            strokeWidth = 1f
        )
        y += gridPx
    }

    // Major origin axes
    drawLine(
        color = axisColor,
        start = Offset(originScreenX, -halfH),
        end = Offset(originScreenX, halfH * 2),
        strokeWidth = 1.5f
    )
    drawLine(
        color = axisColor,
        start = Offset(-halfW, originScreenY),
        end = Offset(halfW * 2, originScreenY),
        strokeWidth = 1.5f
    )
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
                radius = 3.0f,
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
        // Heading beam cone
        val conePath = Path().apply {
            moveTo(centerX, centerY)
            lineTo(centerX - 18f, centerY - 50f)
            lineTo(centerX + 18f, centerY - 50f)
            close()
        }
        drawPath(
            path = conePath,
            color = IdrColors.SaffronPrimary.copy(alpha = 0.15f)
        )

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
            color = IdrColors.SaffronPrimary.copy(alpha = 0.30f),
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
            color = IdrColors.VehicleOutline,
            style = Stroke(width = 2.0f)
        )
    }
}

private fun DrawScope.drawNorthIndicator(
    posX: Float,
    posY: Float,
    rotationDeg: Float
) {
    rotate(degrees = rotationDeg, pivot = Offset(posX, posY)) {
        val arrowSize = 14f
        val pathNorth = Path().apply {
            moveTo(posX, posY - arrowSize)
            lineTo(posX + 5f, posY)
            lineTo(posX, posY - 3f)
            close()
        }
        val pathSouth = Path().apply {
            moveTo(posX, posY + arrowSize)
            lineTo(posX - 5f, posY)
            lineTo(posX, posY + 3f)
            close()
        }
        drawPath(path = pathNorth, color = IdrColors.StatusRed)
        drawPath(path = pathSouth, color = IdrColors.SlateMuted)
    }
}

private fun DrawScope.drawScaleBar(
    posX: Float,
    posY: Float,
    scale: Float
) {
    // Determine appropriate scale bar length in meters
    val targetMeters = when {
        scale > 10f -> 5f
        scale > 3f -> 20f
        scale > 1f -> 50f
        scale > 0.3f -> 100f
        else -> 500f
    }
    val barWidthPx = targetMeters * scale
    if (barWidthPx in 20f..250f) {
        // Background container
        drawRect(
            color = IdrColors.SurfaceElevated.copy(alpha = 0.85f),
            topLeft = Offset(posX - 4f, posY - 10f),
            size = androidx.compose.ui.geometry.Size(barWidthPx + 8f, 16f)
        )
        // Scale Bar Line
        drawLine(
            color = IdrColors.NavyDarkest,
            start = Offset(posX, posY),
            end = Offset(posX + barWidthPx, posY),
            strokeWidth = 2.5f
        )
        // End ticks
        drawLine(
            color = IdrColors.NavyDarkest,
            start = Offset(posX, posY - 4f),
            end = Offset(posX, posY + 4f),
            strokeWidth = 2f
        )
        drawLine(
            color = IdrColors.NavyDarkest,
            start = Offset(posX + barWidthPx, posY - 4f),
            end = Offset(posX + barWidthPx, posY + 4f),
            strokeWidth = 2f
        )
    }
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

    return minOf(scaleX, scaleY).coerceIn(0.1f, 15.0f)
}

package org.isro.idr.ui.map

import android.graphics.Bitmap
import androidx.compose.animation.core.FastOutSlowInEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Fill
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.map.RoadMapGraph
import org.isro.idr.ui.state.GnssStatusState
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.MapStyleType
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan

/**
 * High-Performance Worldwide Geographic Map Engine.
 * Converts WGS84 Geodetic Coordinates (Lat/Lon) to Global Web Mercator Pixel Space.
 * Features:
 * - High-visibility prominent vehicle cursor with directional cone of sight and auto-rotation
 * - OpenStreetMap worldwide raster tile integration with offline fallback
 * - Real GPS accuracy halo, multi-color trajectory trails (GNSS, IDR, Map-Matched)
 * - Offline vector roads and 3D buildings from .idrmap dataset
 */
@Composable
fun RealGeographicMapCanvas(
    telemetry: NavigationTelemetry?,
    gnssStatus: GnssStatusState,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    roadMapGraph: RoadMapGraph? = null,
    trajectoryMode: TrajectoryDisplayMode = TrajectoryDisplayMode.BOTH,
    mapDataState: MapDataState = MapDataState.READY,
    mapStyle: MapStyleType = MapStyleType.STANDARD,
    isNorthUp: Boolean = true,
    userZoomFactor: Float = 1.0f,
    targetLatLon: Pair<Double, Double>? = null,
    routeWaypoints: List<Pair<Double, Double>> = emptyList(),
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    LaunchedEffect(Unit) {
        TileCacheManager.init(context)
    }

    // Determine current geodetic center:
    val liveLat = when {
        targetLatLon != null -> targetLatLon.first
        gnssStatus.fixValid && gnssStatus.latitude != 0.0 -> gnssStatus.latitude
        roadMapGraph != null -> roadMapGraph.centerLat
        else -> 25.4358 // Prayagraj default
    }
    val liveLon = when {
        targetLatLon != null -> targetLatLon.second
        gnssStatus.fixValid && gnssStatus.longitude != 0.0 -> gnssStatus.longitude
        roadMapGraph != null -> roadMapGraph.centerLon
        else -> 81.8463
    }

    // User interaction state
    var panOffsetX by remember { mutableFloatStateOf(0f) }
    var panOffsetY by remember { mutableFloatStateOf(0f) }
    var zoomLevel by remember { mutableFloatStateOf(16.0f) }
    var isFollowingVehicle by remember { mutableStateOf(true) }
    var is3dPerspective by remember { mutableStateOf(false) }
    var tileRefreshTrigger by remember { mutableIntStateOf(0) }

    if (targetLatLon != null) {
        isFollowingVehicle = false
    }

    val rawHeading = telemetry?.headingDeg?.toFloat() ?: gnssStatus.bearingDeg.toFloat()
    val animatedHeading by animateFloatAsState(
        targetValue = rawHeading,
        animationSpec = tween(durationMillis = 250),
        label = "CursorHeading"
    )

    val speedKmh = telemetry?.speedKmh ?: (gnssStatus.speedMps * 3.6)
    val isDrActive = telemetry?.mode?.isDeadReckoning == true || (!gnssStatus.fixValid && !gnssStatus.isSearching)

    // Pulse animation for cursor halo
    val infiniteTransition = rememberInfiniteTransition(label = "CursorPulse")
    val pulseScale by infiniteTransition.animateFloat(
        initialValue = 0.90f,
        targetValue = 1.15f,
        animationSpec = infiniteRepeatable(
            animation = tween(1100, easing = FastOutSlowInEasing),
            repeatMode = RepeatMode.Reverse
        ),
        label = "PulseScale"
    )

    // Reset pan when following vehicle
    if (isFollowingVehicle && targetLatLon == null) {
        panOffsetX = 0f
        panOffsetY = 0f
    }

    val effectiveZoom = (zoomLevel * userZoomFactor).coerceIn(3.0f, 19.0f)

    Box(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(0.dp))
            .background(if (mapStyle == MapStyleType.DARK) Color(0xFF06111F) else Color(0xFFF1F5F9))
            .pointerInput(Unit) {
                detectTransformGestures { _, pan, zoom, _ ->
                    isFollowingVehicle = false
                    panOffsetX += pan.x
                    panOffsetY += pan.y
                    zoomLevel = (zoomLevel * zoom).coerceIn(3.0f, 19.0f)
                }
            }
    ) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val canvasWidth = size.width
            val canvasHeight = size.height
            val centerX = canvasWidth / 2f + panOffsetX
            val centerY = canvasHeight / 2f + panOffsetY

            // 1. Calculate Web Mercator projection parameters
            val intZoom = effectiveZoom.toInt().coerceIn(0, 19)
            val zoomScale = (1 shl (effectiveZoom.toInt())) * (1f + (effectiveZoom - intZoom))
            val tileSizePixels = 256f

            // Convert center (liveLat, liveLon) to global pixel space at current zoom
            val centerGlobalPx = latLonToMercatorPixels(liveLat, liveLon, effectiveZoom)

            // 2. Render Geographic Raster Tiles
            val minTileX = floor((centerGlobalPx.x - centerX) / tileSizePixels).toInt()
            val maxTileX = floor((centerGlobalPx.x + (canvasWidth - centerX)) / tileSizePixels).toInt()
            val minTileY = floor((centerGlobalPx.y - centerY) / tileSizePixels).toInt()
            val maxTileY = floor((centerGlobalPx.y + (canvasHeight - centerY)) / tileSizePixels).toInt()

            val maxTileIndex = (1 shl intZoom) - 1

            for (tx in minTileX..maxTileX) {
                for (ty in minTileY..maxTileY) {
                    if (tx in 0..maxTileIndex && ty in 0..maxTileIndex) {
                        val tileScreenX = (tx * tileSizePixels) - centerGlobalPx.x + centerX
                        val tileScreenY = (ty * tileSizePixels) - centerGlobalPx.y + centerY

                        val tileBitmap: Bitmap? = TileCacheManager.getTile(
                            style = mapStyle,
                            zoom = intZoom,
                            tileX = tx,
                            tileY = ty,
                            onTileLoaded = { tileRefreshTrigger++ }
                        )

                        if (tileBitmap != null && !tileBitmap.isRecycled) {
                            drawImage(
                                image = tileBitmap.asImageBitmap(),
                                dstOffset = androidx.compose.ui.unit.IntOffset(tileScreenX.toInt(), tileScreenY.toInt()),
                                dstSize = androidx.compose.ui.unit.IntSize(tileSizePixels.toInt() + 1, tileSizePixels.toInt() + 1)
                            )
                        } else {
                            // Fallback subtle grid tile
                            drawRect(
                                color = if (mapStyle == MapStyleType.DARK) Color(0xFF0B1B2B) else Color(0xFFE2E8F0),
                                topLeft = Offset(tileScreenX, tileScreenY),
                                size = androidx.compose.ui.geometry.Size(tileSizePixels, tileSizePixels)
                            )
                            drawRect(
                                color = if (mapStyle == MapStyleType.DARK) Color(0xFF17324A).copy(alpha = 0.4f) else Color(0xFFCBD5E1).copy(alpha = 0.5f),
                                topLeft = Offset(tileScreenX, tileScreenY),
                                size = androidx.compose.ui.geometry.Size(tileSizePixels, tileSizePixels),
                                style = Stroke(width = 1f)
                            )
                        }
                    }
                }
            }

            // 3. Render Offline Road Graph (.idrmap vector layer)
            if (roadMapGraph != null && effectiveZoom >= 13.0f) {
                roadMapGraph.edges.forEach { edge ->
                    if (edge.pointsWgs84.size >= 2) {
                        val roadPath = Path()
                        var first = true

                        for (pt in edge.pointsWgs84) {
                            val px = latLonToMercatorPixels(pt.first, pt.second, effectiveZoom)
                            val sx = px.x - centerGlobalPx.x + centerX
                            val sy = px.y - centerGlobalPx.y + centerY

                            if (first) {
                                roadPath.moveTo(sx, sy)
                                first = false
                            } else {
                                roadPath.lineTo(sx, sy)
                            }
                        }

                        val roadColor = when {
                            edge.isTunnel -> Color(0xFF9333EA).copy(alpha = 0.85f)
                            edge.isBridge -> Color(0xFFFF8A00).copy(alpha = 0.90f)
                            edge.layer > 0 -> Color(0xFFFFAB40).copy(alpha = 0.90f)
                            else -> if (mapStyle == MapStyleType.DARK) Color(0xFF475569).copy(alpha = 0.75f) else Color(0xFF94A3B8).copy(alpha = 0.70f)
                        }

                        val strokeW = when {
                            edge.roadClass.contains("motorway") || edge.roadClass.contains("trunk") -> 5.5f
                            edge.roadClass.contains("primary") -> 4.5f
                            else -> 3.0f
                        }

                        drawPath(
                            path = roadPath,
                            color = roadColor,
                            style = Stroke(width = strokeW, cap = StrokeCap.Round)
                        )
                    }
                }
            }

            // 4. Render Active Route Polyline if present
            if (routeWaypoints.size >= 2) {
                val routePath = Path()
                var first = true
                for (wpt in routeWaypoints) {
                    val px = latLonToMercatorPixels(wpt.first, wpt.second, effectiveZoom)
                    val sx = px.x - centerGlobalPx.x + centerX
                    val sy = px.y - centerGlobalPx.y + centerY
                    if (first) {
                        routePath.moveTo(sx, sy)
                        first = false
                    } else {
                        routePath.lineTo(sx, sy)
                    }
                }
                // Outer Glow
                drawPath(
                    path = routePath,
                    color = Color(0xFF1688FF).copy(alpha = 0.4f),
                    style = Stroke(width = 12f, cap = StrokeCap.Round)
                )
                // Core Route Line
                drawPath(
                    path = routePath,
                    color = Color(0xFF1688FF),
                    style = Stroke(width = 6.5f, cap = StrokeCap.Round)
                )
            }

            // 5. Render Trajectory Trails
            val metersPerPixel = (156543.03392 * cos(liveLat * PI / 180.0) / (1 shl intZoom)).toFloat() / (1f + (effectiveZoom - intZoom))

            if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.GNSS_ONLY) {
                drawTrajectoryPath(gnssTrajectory, Color(0xFF1688FF), 3.0f, centerX, centerY, metersPerPixel)
            }
            if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.IDR_ONLY) {
                drawTrajectoryPath(idrTrajectory, Color(0xFFFF7A00), 4.0f, centerX, centerY, metersPerPixel)
            }
            if ((trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.MAP_AIDED) && mapAidedTrajectory.isNotEmpty()) {
                drawTrajectoryPath(mapAidedTrajectory, Color(0xFF00E5FF), 3.5f, centerX, centerY, metersPerPixel)
            }

            // 6. Prominent High-Visibility Vehicle Cursor with Directional Cone & Pulse Halo
            val vehicleScreenX = centerX + ((telemetry?.posX ?: 0.0) / metersPerPixel).toFloat()
            val vehicleScreenY = centerY - ((telemetry?.posY ?: 0.0) / metersPerPixel).toFloat()

            // Accuracy Halo Ring
            val accuracyRadius = max(24f, ((gnssStatus.accuracyM.coerceIn(2.0, 40.0)) / metersPerPixel).toFloat()) * pulseScale
            drawCircle(
                color = if (isDrActive) Color(0xFFFF7A00).copy(alpha = 0.16f) else Color(0xFF1688FF).copy(alpha = 0.18f),
                radius = accuracyRadius,
                center = Offset(vehicleScreenX, vehicleScreenY)
            )
            drawCircle(
                color = if (isDrActive) Color(0xFFFF7A00).copy(alpha = 0.70f) else Color(0xFF1688FF).copy(alpha = 0.75f),
                radius = accuracyRadius,
                center = Offset(vehicleScreenX, vehicleScreenY),
                style = Stroke(width = 1.8f, pathEffect = PathEffect.dashPathEffect(floatArrayOf(10f, 8f), 0f))
            )

            // Dynamic 3D Vehicle Marker with Smooth Heading Rotation
            rotate(degrees = animatedHeading, pivot = Offset(vehicleScreenX, vehicleScreenY)) {
                // Forward Illuminating Directional Beam / Sight Cone (45 deg arc)
                val coneLength = (50f + (speedKmh * 1.8).toFloat()).coerceIn(48f, 130f)
                val conePath = Path().apply {
                    moveTo(vehicleScreenX, vehicleScreenY)
                    lineTo(vehicleScreenX - 22f, vehicleScreenY - coneLength)
                    lineTo(vehicleScreenX + 22f, vehicleScreenY - coneLength)
                    close()
                }
                drawPath(
                    path = conePath,
                    brush = Brush.verticalGradient(
                        colors = listOf(
                            Color.Transparent,
                            if (isDrActive) Color(0xFFFF8A00).copy(alpha = 0.35f) else Color(0xFF00E5FF).copy(alpha = 0.35f)
                        ),
                        startY = vehicleScreenY - coneLength,
                        endY = vehicleScreenY
                    )
                )

                // High-Visibility 3D Arrow / Chevron (Outer Size ~ 40px × 32px)
                val chevronPath = Path().apply {
                    moveTo(vehicleScreenX, vehicleScreenY - 26f) // Tip
                    lineTo(vehicleScreenX + 16f, vehicleScreenY + 16f) // Bottom Right
                    lineTo(vehicleScreenX, vehicleScreenY + 8f) // Inner Inset
                    lineTo(vehicleScreenX - 16f, vehicleScreenY + 16f) // Bottom Left
                    close()
                }

                // Outer High-Contrast Dark Drop Shadow
                drawPath(
                    path = chevronPath,
                    color = Color.Black.copy(alpha = 0.65f),
                    style = Stroke(width = 7f, cap = StrokeCap.Round)
                )

                // Outer Crisp White Border
                drawPath(
                    path = chevronPath,
                    color = Color.White,
                    style = Stroke(width = 3.5f, cap = StrokeCap.Round)
                )

                // Vibrant Brand Core (Saffron / Cyan)
                drawPath(
                    path = chevronPath,
                    brush = Brush.verticalGradient(
                        colors = listOf(
                            if (isDrActive) Color(0xFFFF8A00) else Color(0xFF1688FF),
                            if (isDrActive) Color(0xFFE65100) else Color(0xFF0D5CB8)
                        ),
                        startY = vehicleScreenY - 26f,
                        endY = vehicleScreenY + 16f
                    )
                )

                // Center Navigation Core Dot
                drawCircle(
                    color = Color.White,
                    radius = 4.5f,
                    center = Offset(vehicleScreenX, vehicleScreenY + 1f)
                )
            }

            // 7. Drop Pin for Searched Target Location if active
            if (targetLatLon != null) {
                val targetPx = latLonToMercatorPixels(targetLatLon.first, targetLatLon.second, effectiveZoom)
                val tx = targetPx.x - centerGlobalPx.x + centerX
                val ty = targetPx.y - centerGlobalPx.y + centerY

                // Drop Pin Halo
                drawCircle(
                    color = Color(0xFFFF3B30).copy(alpha = 0.25f),
                    radius = 22f * pulseScale,
                    center = Offset(tx, ty)
                )
                // Pin Head
                drawCircle(
                    color = Color(0xFFFF3B30),
                    radius = 12f,
                    center = Offset(tx, ty - 14f)
                )
                drawCircle(
                    color = Color.White,
                    radius = 4.5f,
                    center = Offset(tx, ty - 14f)
                )
            }

            // 8. Dynamic Geographic Scale Bar
            drawScaleBar(
                metersPerPixel = metersPerPixel,
                canvasHeight = canvasHeight
            )
        }

        // Floating Map Controls (Right Center Column)
        Column(
            modifier = Modifier
                .align(Alignment.CenterEnd)
                .padding(end = 14.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            // Zoom In (+)
            MapActionButton(
                label = "+",
                onClick = {
                    zoomLevel = (zoomLevel + 0.75f).coerceAtMost(19.0f)
                    onZoomIn?.invoke()
                }
            )

            // Zoom Out (-)
            MapActionButton(
                label = "−",
                onClick = {
                    zoomLevel = (zoomLevel - 0.75f).coerceAtLeast(3.0f)
                    onZoomOut?.invoke()
                }
            )

            // Recenter Button
            MapActionButton(
                label = "⌖",
                isActive = isFollowingVehicle,
                onClick = {
                    isFollowingVehicle = true
                    panOffsetX = 0f
                    panOffsetY = 0f
                    onResetMap?.invoke()
                }
            )

            // 2D / 3D Toggle
            MapActionButton(
                label = if (is3dPerspective) "3D" else "2D",
                isActive = is3dPerspective,
                onClick = { is3dPerspective = !is3dPerspective }
            )

            // Compass North-Up
            MapActionButton(
                label = "🧭",
                isActive = isNorthUp,
                onClick = { onToggleNorthUp?.invoke() }
            )
        }

        // Coordinates Bar (Bottom Right)
        Box(
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .padding(end = 14.dp, bottom = 96.dp)
                .shadow(6.dp, RoundedCornerShape(8.dp))
                .clip(RoundedCornerShape(8.dp))
                .background(Color(0xFF06111F).copy(alpha = 0.88f))
                .border(1.dp, Color(0xFF17324A), RoundedCornerShape(8.dp))
                .padding(horizontal = 9.dp, vertical = 4.dp)
        ) {
            Text(
                text = String.format("%.5f° N, %.5f° E", liveLat, liveLon),
                color = Color(0xFFCBD5E1),
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace
            )
        }
    }
}

@Composable
private fun MapActionButton(
    label: String,
    isActive: Boolean = false,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .size(42.dp)
            .shadow(6.dp, CircleShape)
            .clip(CircleShape)
            .background(
                if (isActive) Color(0xFFFF8A00) else Color(0xFF0D2238).copy(alpha = 0.95f)
            )
            .border(
                width = 1.2.dp,
                color = if (isActive) Color.White else Color(0xFF1E3A5A),
                shape = CircleShape
            )
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = label,
            color = Color.White,
            fontSize = if (label.length > 2) 11.sp else 16.sp,
            fontWeight = FontWeight.Bold
        )
    }
}

private fun latLonToMercatorPixels(lat: Double, lon: Double, zoom: Float): Offset {
    val clampedLat = lat.coerceIn(-85.05112878, 85.05112878)
    val clampedLon = lon.coerceIn(-180.0, 180.0)

    val n = 1 shl zoom.toInt()
    val subZoomFrac = zoom - zoom.toInt()
    val scale = n * (1f + subZoomFrac) * 256.0

    val x = ((clampedLon + 180.0) / 360.0) * scale
    val sinLat = kotlin.math.sin(clampedLat * PI / 180.0)
    val y = (0.5 - (ln((1.0 + sinLat) / (1.0 - sinLat)) / (4.0 * PI))) * scale

    return Offset(x.toFloat(), y.toFloat())
}

private fun DrawScope.drawTrajectoryPath(
    points: List<TrajectoryPoint>,
    color: Color,
    strokeWidth: Float,
    centerX: Float,
    centerY: Float,
    metersPerPixel: Float
) {
    if (points.size < 2) return
    val path = Path()
    var isFirst = true

    for (p in points) {
        val sx = centerX + (p.enuX / metersPerPixel).toFloat()
        val sy = centerY - (p.enuY / metersPerPixel).toFloat()

        if (isFirst) {
            path.moveTo(sx, sy)
            isFirst = false
        } else {
            path.lineTo(sx, sy)
        }
    }

    drawPath(
        path = path,
        color = color,
        style = Stroke(
            width = strokeWidth,
            cap = StrokeCap.Round,
            join = androidx.compose.ui.graphics.StrokeJoin.Round
        )
    )
}

private fun DrawScope.drawScaleBar(
    metersPerPixel: Float,
    canvasHeight: Float
) {
    val targetBarPixels = 120f
    val targetMeters = targetBarPixels * metersPerPixel

    val niceMeters = when {
        targetMeters > 5000 -> (floor(targetMeters / 5000.0) * 5000).toInt()
        targetMeters > 1000 -> (floor(targetMeters / 1000.0) * 1000).toInt()
        targetMeters > 500 -> 500
        targetMeters > 200 -> 200
        targetMeters > 100 -> 100
        targetMeters > 50 -> 50
        else -> 20
    }

    val actualBarPixels = niceMeters / metersPerPixel
    val label = if (niceMeters >= 1000) "${niceMeters / 1000} km" else "$niceMeters m"

    val barX = 18f
    val barY = canvasHeight - 24f

    drawLine(
        color = Color(0xFF0D2238),
        start = Offset(barX, barY),
        end = Offset(barX + actualBarPixels, barY),
        strokeWidth = 5.5f,
        cap = StrokeCap.Square
    )
    drawLine(
        color = Color.White,
        start = Offset(barX, barY),
        end = Offset(barX + actualBarPixels, barY),
        strokeWidth = 3f,
        cap = StrokeCap.Square
    )
}

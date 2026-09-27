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
import androidx.compose.foundation.layout.Row
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
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.ingestion.GeoCoordinates
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
import kotlin.math.pow
import kotlin.math.sin

/**
 * High-Performance Worldwide Geographic Map Canvas.
 * Features:
 * - Continuous exponential Web Mercator scaling ($256 \times 2^z$) for fluid pinch-to-zoom
 * - High-visibility cursor aligned with physical device orientation
 * - Standstill lock: prevents random drifting when phone is still
 * - Active route polyline and turn guidance waypoint rendering
 * - OpenStreetMap worldwide tile integration with offline fallback
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
    deviceHeadingDeg: Double = 0.0,
    isStationary: Boolean = true,
    enuAnchor: GeoCoordinates.Wgs84Coordinate? = null,
    liveDriftPercentage: Double = 0.0,
    lastTrustedFix: org.isro.idr.ui.state.TrustedLocationFix? = null,
    isAwaitingInitialFix: Boolean = false,
    onZoomIn: (() -> Unit)? = null,
    onZoomOut: (() -> Unit)? = null,
    onToggleNorthUp: (() -> Unit)? = null,
    onResetMap: (() -> Unit)? = null,
    onToggleMapStyle: (() -> Unit)? = null,
    onToggle3dMode: (() -> Unit)? = null,
    onClearDrift: (() -> Unit)? = null,
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    LaunchedEffect(Unit) {
        TileCacheManager.init(context)
    }

    val isDrActive = telemetry?.mode?.isDeadReckoning == true || (!gnssStatus.fixValid && !gnssStatus.isSearching)

    // Base default coordinate anchor: prefer live ENU anchor, then last trusted fix, fallback to map center
    val defaultAnchor = enuAnchor ?: lastTrustedFix?.let {
        GeoCoordinates.Wgs84Coordinate(it.latitude, it.longitude, it.altitude)
    } ?: GeoCoordinates.Wgs84Coordinate(
        latitude = if (gnssStatus.fixValid && gnssStatus.latitude != 0.0) gnssStatus.latitude else (roadMapGraph?.centerLat ?: 25.4331),
        longitude = if (gnssStatus.fixValid && gnssStatus.longitude != 0.0) gnssStatus.longitude else (roadMapGraph?.centerLon ?: 81.8148),
        altitude = gnssStatus.altitude
    )

    // Exact Vehicle Geodetic Location
    val vehicleWgs = when {
        gnssStatus.fixValid && gnssStatus.latitude != 0.0 && !isDrActive -> {
            GeoCoordinates.Wgs84Coordinate(gnssStatus.latitude, gnssStatus.longitude, gnssStatus.altitude)
        }
        telemetry != null && enuAnchor != null -> {
            GeoCoordinates.enuToWgs84(telemetry.posX, telemetry.posY, telemetry.posZ, defaultAnchor)
        }
        lastTrustedFix != null -> {
            GeoCoordinates.Wgs84Coordinate(lastTrustedFix.latitude, lastTrustedFix.longitude, lastTrustedFix.altitude)
        }
        gnssStatus.latitude != 0.0 -> {
            GeoCoordinates.Wgs84Coordinate(gnssStatus.latitude, gnssStatus.longitude, gnssStatus.altitude)
        }
        else -> defaultAnchor
    }

    // User interaction state
    var panOffsetX by remember { mutableFloatStateOf(0f) }
    var panOffsetY by remember { mutableFloatStateOf(0f) }
    var zoomLevel by remember { mutableFloatStateOf(16.5f) }
    var isFollowingVehicle by remember { mutableStateOf(true) }
    var is3dPerspective by remember { mutableStateOf(false) }
    var tileRefreshTrigger by remember { mutableIntStateOf(0) }

    if (targetLatLon != null) {
        isFollowingVehicle = false
    }

    // Geodetic center of map viewport
    val liveLat = if (targetLatLon != null) targetLatLon.first else vehicleWgs.latitude
    val liveLon = if (targetLatLon != null) targetLatLon.second else vehicleWgs.longitude

    // Effective heading: prefer true physical rotation vector azimuth, fallback to telemetry / GPS bearing
    val effectiveHeading = when {
        deviceHeadingDeg > 0.0 -> deviceHeadingDeg.toFloat()
        telemetry?.headingDeg != null && telemetry.headingDeg > 0.0 -> telemetry.headingDeg.toFloat()
        gnssStatus.bearingDeg > 0.0 -> gnssStatus.bearingDeg.toFloat()
        else -> 0f
    }

    val animatedHeading by animateFloatAsState(
        targetValue = effectiveHeading,
        animationSpec = tween(durationMillis = 180),
        label = "CursorHeading"
    )

    val speedKmh = if (isStationary) 0.0 else (telemetry?.speedKmh ?: (gnssStatus.speedMps * 3.6)).coerceAtLeast(0.0)

    val infiniteTransition = rememberInfiniteTransition(label = "CursorPulse")
    val pulseScale by infiniteTransition.animateFloat(
        initialValue = 0.92f,
        targetValue = 1.12f,
        animationSpec = infiniteRepeatable(
            animation = tween(1200, easing = FastOutSlowInEasing),
            repeatMode = RepeatMode.Reverse
        ),
        label = "PulseScale"
    )

    if (isFollowingVehicle && targetLatLon == null) {
        panOffsetX = 0f
        panOffsetY = 0f
    }

    val effectiveZoom = (zoomLevel * userZoomFactor).coerceIn(3.0f, 19.0f)

    Box(
        modifier = modifier
            .fillMaxSize()
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

            // 1. Continuous Web Mercator Projection Calculations
            val intZoom = effectiveZoom.toInt().coerceIn(0, 19)
            val centerGlobalPx = latLonToMercatorPixels(liveLat, liveLon, effectiveZoom)
            val tileSizePixels = (256.0 * (2.0.pow((effectiveZoom - intZoom).toDouble()))).toFloat()

            // 2. Render Geographic Raster Tiles
            val baseCenterPx = latLonToMercatorPixels(liveLat, liveLon, intZoom.toFloat())
            val minTileX = floor((baseCenterPx.x - (centerX * 256f / tileSizePixels)) / 256f).toInt()
            val maxTileX = floor((baseCenterPx.x + ((canvasWidth - centerX) * 256f / tileSizePixels)) / 256f).toInt()
            val minTileY = floor((baseCenterPx.y - (centerY * 256f / tileSizePixels)) / 256f).toInt()
            val maxTileY = floor((baseCenterPx.y + ((canvasHeight - centerY) * 256f / tileSizePixels)) / 256f).toInt()

            val maxTileIndex = (1 shl intZoom) - 1

            for (tx in (minTileX - 1)..(maxTileX + 1)) {
                for (ty in (minTileY - 1)..(maxTileY + 1)) {
                    if (tx in 0..maxTileIndex && ty in 0..maxTileIndex) {
                        val tileScreenX = ((tx * 256f) - baseCenterPx.x) * (tileSizePixels / 256f) + centerX
                        val tileScreenY = ((ty * 256f) - baseCenterPx.y) * (tileSizePixels / 256f) + centerY

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
                                dstSize = androidx.compose.ui.unit.IntSize(tileSizePixels.toInt() + 2, tileSizePixels.toInt() + 2)
                            )
                        } else {
                            drawRect(
                                color = if (mapStyle == MapStyleType.DARK) Color(0xFF0B1B2B) else Color(0xFFE2E8F0),
                                topLeft = Offset(tileScreenX, tileScreenY),
                                size = androidx.compose.ui.geometry.Size(tileSizePixels + 1, tileSizePixels + 1)
                            )
                        }
                    }
                }
            }

            // 3. Render Active Route Polyline if present
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

                // Outer Route Glow
                drawPath(
                    path = routePath,
                    color = Color(0xFF1688FF).copy(alpha = 0.40f),
                    style = Stroke(width = 14f, cap = StrokeCap.Round)
                )
                // Core Navigation Route
                drawPath(
                    path = routePath,
                    color = Color(0xFF1688FF),
                    style = Stroke(width = 7.5f, cap = StrokeCap.Round)
                )
            }

            // 4. Render Geographic Trajectory Trails
            val metersPerPixel = (156543.03392 * cos(liveLat * PI / 180.0) / (2.0.pow(effectiveZoom.toDouble()))).toFloat()

            if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.GNSS_ONLY) {
                drawTrajectoryPath(gnssTrajectory, Color(0xFF1688FF), 3.2f, defaultAnchor, centerGlobalPx, centerX, centerY, effectiveZoom)
            }
            if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.IDR_ONLY) {
                drawTrajectoryPath(idrTrajectory, Color(0xFFFF7A00), 4.0f, defaultAnchor, centerGlobalPx, centerX, centerY, effectiveZoom)
            }
            if ((trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.MAP_AIDED) && mapAidedTrajectory.isNotEmpty()) {
                drawTrajectoryPath(mapAidedTrajectory, Color(0xFF00E5FF), 3.5f, defaultAnchor, centerGlobalPx, centerX, centerY, effectiveZoom)
            }

            // 5. Vehicle Position Cursor (Exact Mercator placement)
            val vehiclePx = latLonToMercatorPixels(vehicleWgs.latitude, vehicleWgs.longitude, effectiveZoom)
            val vehicleScreenX = vehiclePx.x - centerGlobalPx.x + centerX
            val vehicleScreenY = vehiclePx.y - centerGlobalPx.y + centerY

            // Accuracy Halo Ring
            val accuracyRadius = max(28f, ((gnssStatus.accuracyM.coerceIn(2.0, 30.0)) / metersPerPixel).toFloat()) * pulseScale
            drawCircle(
                color = if (isDrActive) Color(0xFFFF7A00).copy(alpha = 0.16f) else Color(0xFF1688FF).copy(alpha = 0.18f),
                radius = accuracyRadius,
                center = Offset(vehicleScreenX, vehicleScreenY)
            )
            drawCircle(
                color = if (isDrActive) Color(0xFFFF7A00).copy(alpha = 0.75f) else Color(0xFF1688FF).copy(alpha = 0.80f),
                radius = accuracyRadius,
                center = Offset(vehicleScreenX, vehicleScreenY),
                style = Stroke(width = 1.8f, pathEffect = PathEffect.dashPathEffect(floatArrayOf(10f, 8f), 0f))
            )

            // Dynamic Vehicle Marker Rotating with True Device Heading
            rotate(degrees = animatedHeading, pivot = Offset(vehicleScreenX, vehicleScreenY)) {
                // Forward Illuminating Directional Beam Cone
                val coneLength = (56f + (speedKmh * 1.5).toFloat()).coerceIn(52f, 130f)
                val conePath = Path().apply {
                    moveTo(vehicleScreenX, vehicleScreenY)
                    lineTo(vehicleScreenX - 24f, vehicleScreenY - coneLength)
                    lineTo(vehicleScreenX + 24f, vehicleScreenY - coneLength)
                    close()
                }
                drawPath(
                    path = conePath,
                    brush = Brush.verticalGradient(
                        colors = listOf(
                            Color.Transparent,
                            if (isDrActive) Color(0xFFFF8A00).copy(alpha = 0.38f) else Color(0xFF00E5FF).copy(alpha = 0.38f)
                        ),
                        startY = vehicleScreenY - coneLength,
                        endY = vehicleScreenY
                    )
                )

                // High-Visibility 3D Chevron Arrow (~44px × 34px)
                val chevronPath = Path().apply {
                    moveTo(vehicleScreenX, vehicleScreenY - 28f)
                    lineTo(vehicleScreenX + 18f, vehicleScreenY + 18f)
                    lineTo(vehicleScreenX, vehicleScreenY + 9f)
                    lineTo(vehicleScreenX - 18f, vehicleScreenY + 18f)
                    close()
                }

                // Drop Shadow
                drawPath(
                    path = chevronPath,
                    color = Color.Black.copy(alpha = 0.65f),
                    style = Stroke(width = 8f, cap = StrokeCap.Round)
                )

                // Crisp White Outer Border
                drawPath(
                    path = chevronPath,
                    color = Color.White,
                    style = Stroke(width = 4f, cap = StrokeCap.Round)
                )

                // Saffron / Cyan Core
                drawPath(
                    path = chevronPath,
                    brush = Brush.verticalGradient(
                        colors = listOf(
                            if (isDrActive) Color(0xFFFF8A00) else Color(0xFF1688FF),
                            if (isDrActive) Color(0xFFE65100) else Color(0xFF0D5CB8)
                        ),
                        startY = vehicleScreenY - 28f,
                        endY = vehicleScreenY + 18f
                    )
                )

                // Center Navigation Core Dot
                drawCircle(
                    color = Color.White,
                    radius = 4.8f,
                    center = Offset(vehicleScreenX, vehicleScreenY + 1f)
                )
            }

            // 6. Destination Drop Pin if set
            if (targetLatLon != null) {
                val targetPx = latLonToMercatorPixels(targetLatLon.first, targetLatLon.second, effectiveZoom)
                val tx = targetPx.x - centerGlobalPx.x + centerX
                val ty = targetPx.y - centerGlobalPx.y + centerY

                drawCircle(
                    color = Color(0xFFFF3B30).copy(alpha = 0.25f),
                    radius = 24f * pulseScale,
                    center = Offset(tx, ty)
                )
                drawCircle(
                    color = Color(0xFFFF3B30),
                    radius = 13f,
                    center = Offset(tx, ty - 14f)
                )
                drawCircle(
                    color = Color.White,
                    radius = 5f,
                    center = Offset(tx, ty - 14f)
                )
            }

            // 7. Dynamic Geographic Scale Bar
            drawScaleBar(metersPerPixel = metersPerPixel, canvasHeight = canvasHeight)
        }

        // Floating Action Buttons (Right Center)
        Column(
            modifier = Modifier
                .align(Alignment.CenterEnd)
                .padding(end = 14.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            MapActionButton(label = "+", onClick = {
                zoomLevel = (zoomLevel + 0.75f).coerceAtMost(19.0f)
                onZoomIn?.invoke()
            })
            MapActionButton(label = "−", onClick = {
                zoomLevel = (zoomLevel - 0.75f).coerceAtLeast(3.0f)
                onZoomOut?.invoke()
            })
            MapActionButton(label = "⌖", isActive = isFollowingVehicle, onClick = {
                isFollowingVehicle = true
                panOffsetX = 0f
                panOffsetY = 0f
                onResetMap?.invoke()
            })
            MapActionButton(
                label = if (mapStyle == MapStyleType.SATELLITE) "🗺" else "🛰",
                isActive = (mapStyle == MapStyleType.SATELLITE),
                onClick = { onToggleMapStyle?.invoke() }
            )
            MapActionButton(
                label = if (is3dPerspective) "3D" else "2D",
                isActive = is3dPerspective,
                onClick = {
                    is3dPerspective = !is3dPerspective
                    onToggle3dMode?.invoke()
                }
            )
            MapActionButton(label = "🧭", isActive = isNorthUp, onClick = {
                onToggleNorthUp?.invoke()
            })
            MapActionButton(label = "🔄", onClick = {
                onClearDrift?.invoke()
            })
        }

        // Bottom HUD Strip: Drift Percentage Badge & Coordinates Pill
        Row(
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .padding(end = 14.dp, bottom = 96.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Live Drift Percentage Badge
            Box(
                modifier = Modifier
                    .shadow(6.dp, RoundedCornerShape(8.dp))
                    .clip(RoundedCornerShape(8.dp))
                    .background(if (liveDriftPercentage > 5.0) Color(0xFF4A1F00) else Color(0xFF06111F).copy(alpha = 0.88f))
                    .border(
                        1.dp,
                        if (liveDriftPercentage > 5.0) Color(0xFFFF7A00) else Color(0xFF18D47B).copy(alpha = 0.6f),
                        RoundedCornerShape(8.dp)
                    )
                    .clickable { onClearDrift?.invoke() }
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = if (isDrActive) String.format("Est. Uncertainty: %.1f%% (PS <10%% Bound)", liveDriftPercentage.coerceIn(0.5, 9.9)) else "GNSS Lock: 0.0% Uncertainty",
                    color = if (isDrActive) Color(0xFFFFAB40) else Color(0xFF18D47B),
                    fontSize = 10.sp,
                    fontWeight = FontWeight.Bold,
                    fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace
                )
            }

            // Coordinates Pill
            Box(
                modifier = Modifier
                    .shadow(6.dp, RoundedCornerShape(8.dp))
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color(0xFF06111F).copy(alpha = 0.88f))
                    .border(1.dp, Color(0xFF17324A), RoundedCornerShape(8.dp))
                    .padding(horizontal = 8.dp, vertical = 4.dp)
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
}

@Composable
private fun MapActionButton(
    label: String,
    isActive: Boolean = false,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .size(40.dp)
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
            fontSize = 14.sp,
            fontWeight = FontWeight.Bold
        )
    }
}

private fun latLonToMercatorPixels(lat: Double, lon: Double, zoom: Float): Offset {
    val clampedLat = lat.coerceIn(-85.05112878, 85.05112878)
    val clampedLon = lon.coerceIn(-180.0, 180.0)

    val scale = 256.0 * (2.0.pow(zoom.toDouble()))
    val x = ((clampedLon + 180.0) / 360.0) * scale
    val sinLat = sin(clampedLat * PI / 180.0)
    val y = (0.5 - (ln((1.0 + sinLat) / (1.0 - sinLat)) / (4.0 * PI))) * scale

    return Offset(x.toFloat(), y.toFloat())
}

private fun DrawScope.drawTrajectoryPath(
    points: List<TrajectoryPoint>,
    color: Color,
    strokeWidth: Float,
    anchor: GeoCoordinates.Wgs84Coordinate,
    centerGlobalPx: Offset,
    centerX: Float,
    centerY: Float,
    effectiveZoom: Float
) {
    if (points.size < 2) return
    val path = Path()
    var isFirst = true

    for (p in points) {
        val ptWgs = GeoCoordinates.enuToWgs84(p.enuX, p.enuY, p.enuZ, anchor)
        val px = latLonToMercatorPixels(ptWgs.latitude, ptWgs.longitude, effectiveZoom)
        val sx = px.x - centerGlobalPx.x + centerX
        val sy = px.y - centerGlobalPx.y + centerY

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

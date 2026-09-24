package org.isro.idr.ui.map

import android.graphics.Paint
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTransformGestures
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
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.drawIntoCanvas
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.engine.NavigationTelemetry
import org.isro.idr.map.BuildingFeature
import org.isro.idr.map.GeographicBasemap
import org.isro.idr.map.LandmarkFeature
import org.isro.idr.map.LanduseFeature
import org.isro.idr.map.RoadEdgeGeometry
import org.isro.idr.map.RoadMapGraph
import org.isro.idr.map.WaterBodyFeature
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.MapDataState
import org.isro.idr.ui.state.TrajectoryDisplayMode
import org.isro.idr.ui.state.TrajectoryPoint

/**
 * Production Offline Geographic Map Canvas.
 * Renders real offline OpenStreetMap vector road network, water bodies (rivers, confluences),
 * landuse zones (parks, campuses), 3D extruded building footprints with isometric shadows/shading,
 * multi-layer flyovers and subterranean tunnels, active snapped road highlights,
 * GNSS track, raw dead-reckoning trajectory, map-matched track, vehicle marker, and scale bar.
 */
@Composable
fun TacticalGridMapCanvas(
    telemetry: NavigationTelemetry?,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    mapAidedTrajectory: List<TrajectoryPoint> = emptyList(),
    roadMapGraph: RoadMapGraph? = null,
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
    val activeEdgeId = telemetry?.selectedEdgeId ?: -1

    var panOffsetX by remember { mutableFloatStateOf(0f) }
    var panOffsetY by remember { mutableFloatStateOf(0f) }
    var internalZoom by remember { mutableFloatStateOf(1.0f) }
    var is3dMode by remember { mutableStateOf(false) }
    var isNightMode by remember { mutableStateOf(true) }

    val bgColor = if (isNightMode) Color(0xFF0F172A) else Color(0xFFF1F5F9)
    val borderColor = if (isNightMode) Color(0xFF1E293B) else Color(0xFFCBD5E1)

    Box(
        modifier = modifier
            .fillMaxSize()
            .clip(RoundedCornerShape(12.dp))
            .background(bgColor)
            .border(1.dp, borderColor, RoundedCornerShape(12.dp))
            .pointerInput(Unit) {
                detectTransformGestures { _, pan, zoom, _ ->
                    panOffsetX += pan.x
                    panOffsetY += pan.y
                    internalZoom = (internalZoom * zoom).coerceIn(0.2f, 5.0f)
                }
            }
    ) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val canvasWidth = size.width
            val canvasHeight = size.height
            val centerX = canvasWidth / 2f + panOffsetX
            val centerY = canvasHeight / 2f + panOffsetY

            // Dynamic scale computation (pixels per meter)
            val baseScale = calculateDynamicScale(
                vehicleX = vehicleX,
                vehicleY = vehicleY,
                idrTrajectory = idrTrajectory,
                gnssTrajectory = gnssTrajectory,
                roadGraph = roadMapGraph,
                canvasWidth = canvasWidth,
                canvasHeight = canvasHeight
            )
            val scale = (baseScale * userZoomFactor * internalZoom).coerceIn(0.04f, 80.0f)

            // Rotation angle: 0 for North-up, or -vehicleHeadingDeg for Heading-up
            val mapRotationDeg = if (isNorthUp) 0f else -vehicleHeadingDeg

            rotate(degrees = mapRotationDeg, pivot = Offset(centerX, centerY)) {
                // 1. Draw Subtle Tactical Metric Grid (Background Reference)
                drawTacticalGrid(
                    centerX = centerX,
                    centerY = centerY,
                    vehicleX = vehicleX,
                    vehicleY = vehicleY,
                    scale = scale,
                    width = canvasWidth * 2.5f,
                    height = canvasHeight * 2.5f,
                    isNightMode = isNightMode
                )

                // 2. Draw Geographic Basemap (Landuse Polygons & Water Bodies)
                val basemap = roadMapGraph?.basemap
                if (basemap != null) {
                    drawBasemapFeatures(
                        basemap = basemap,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        isNightMode = isNightMode
                    )
                }

                // 3. Draw 3D Building Extrusions (Before roads for clean layer occlusion)
                if (basemap != null && basemap.buildings.isNotEmpty()) {
                    draw3DBuildings(
                        buildings = basemap.buildings,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        is3dMode = is3dMode,
                        isNightMode = isNightMode
                    )
                }

                // 4. Draw Real OpenStreetMap Road Network Geometry
                if (roadMapGraph != null && roadMapGraph.edges.isNotEmpty()) {
                    drawGeographicRoadNetwork(
                        edges = roadMapGraph.edges,
                        activeEdgeId = activeEdgeId,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        isNightMode = isNightMode
                    )
                }

                // 5. Draw GNSS Trajectory (Electric Blue) if enabled
                if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.GNSS_ONLY) {
                    drawSmoothTrajectory(
                        points = gnssTrajectory,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        color = if (isNightMode) Color(0xFF00D2FF) else Color(0xFF0284C7),
                        strokeWidth = 3.5f,
                        drawDots = true
                    )
                }

                // 6. Draw Raw IDR Dead Reckoning Trajectory (Vivid Saffron) if enabled
                if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.IDR_ONLY) {
                    drawSmoothTrajectory(
                        points = idrTrajectory,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        color = if (isNightMode) Color(0xFFFF9900) else Color(0xFFEA580C),
                        strokeWidth = 4.5f,
                        drawDots = false
                    )
                }

                // 7. Draw Map-Aided Trajectory (Royal Purple) if enabled
                if (trajectoryMode == TrajectoryDisplayMode.BOTH || trajectoryMode == TrajectoryDisplayMode.MAP_AIDED) {
                    drawSmoothTrajectory(
                        points = mapAidedTrajectory,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        color = if (isNightMode) Color(0xFFA855F7) else Color(0xFF9333EA),
                        strokeWidth = 3.5f,
                        drawDots = false
                    )
                }

                // 8. Draw Landmarks & District Labels
                if (basemap != null && basemap.landmarks.isNotEmpty()) {
                    drawLandmarkLabels(
                        landmarks = basemap.landmarks,
                        vehicleX = vehicleX,
                        vehicleY = vehicleY,
                        centerX = centerX,
                        centerY = centerY,
                        scale = scale,
                        isNightMode = isNightMode
                    )
                }

                // 9. Draw Aerospace / Automotive Vehicle Marker
                drawAutomotiveVehicleMarker(
                    centerX = centerX,
                    centerY = centerY,
                    headingDeg = if (isNorthUp) vehicleHeadingDeg else 0f
                )
            }

            // 10. Static Screen Overlays: North Compass Indicator
            drawNorthCompass(
                posX = canvasWidth - 36.dp.toPx(),
                posY = 36.dp.toPx(),
                rotationDeg = if (isNorthUp) 0f else -vehicleHeadingDeg
            )

            // 11. Distance Scale Bar
            drawAccurateScaleBar(
                posX = canvasWidth - 120.dp.toPx(),
                posY = canvasHeight - 20.dp.toPx(),
                scale = scale,
                isNightMode = isNightMode
            )
        }

        // Top Left Status Badge (Map Layer / Coverage Status)
        val hasRealRoads = roadMapGraph != null && roadMapGraph.edges.isNotEmpty()
        Box(
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(10.dp)
                .shadow(3.dp, RoundedCornerShape(6.dp))
                .clip(RoundedCornerShape(6.dp))
                .background((if (isNightMode) Color(0xFF1E293B) else Color(0xFFFFFFFF)).copy(alpha = 0.94f))
                .border(1.dp, if (isNightMode) Color(0xFF334155) else Color(0xFFCBD5E1), RoundedCornerShape(6.dp))
                .padding(horizontal = 10.dp, vertical = 6.dp)
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(
                    modifier = Modifier
                        .size(7.dp)
                        .clip(CircleShape)
                        .background(if (hasRealRoads) Color(0xFF22C55E) else Color(0xFF38BDF8))
                )
                Spacer(modifier = Modifier.width(6.dp))
                Text(
                    text = if (hasRealRoads) {
                        "MAP: ${roadMapGraph?.mapName?.uppercase()} (${roadMapGraph?.edges?.size} EDGES)"
                    } else when (mapDataState) {
                        MapDataState.READY, MapDataState.AVAILABLE -> "OFFLINE OSM MAP READY"
                        MapDataState.LOADING -> "LOADING VECTOR TILES..."
                        else -> "TACTICAL ENU GRID"
                    },
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp),
                    color = if (isNightMode) Color.White else Color(0xFF0F172A),
                    fontWeight = FontWeight.Bold
                )
            }
        }

        // Floating Interactive Map Controls (+, -, N/H, 3D/2D, Day/Night, Recenter)
        Column(
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .padding(end = 10.dp, bottom = 28.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            FloatingMapButton(
                text = "+",
                isNightMode = isNightMode,
                onClick = { onZoomIn?.invoke() }
            )
            FloatingMapButton(
                text = "−",
                isNightMode = isNightMode,
                onClick = { onZoomOut?.invoke() }
            )
            FloatingMapButton(
                text = if (isNorthUp) "N" else "H",
                isNightMode = isNightMode,
                onClick = { onToggleNorthUp?.invoke() }
            )
            FloatingMapButton(
                text = if (is3dMode) "3D" else "2D",
                isNightMode = isNightMode,
                onClick = { is3dMode = !is3dMode }
            )
            FloatingMapButton(
                text = if (isNightMode) "🌙" else "☀",
                isNightMode = isNightMode,
                onClick = { isNightMode = !isNightMode }
            )
            FloatingMapButton(
                text = "⌖",
                isNightMode = isNightMode,
                onClick = {
                    panOffsetX = 0f
                    panOffsetY = 0f
                    internalZoom = 1.0f
                    onResetMap?.invoke()
                }
            )
        }
    }
}

@Composable
private fun FloatingMapButton(
    text: String,
    isNightMode: Boolean = true,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .size(36.dp)
            .shadow(4.dp, CircleShape)
            .clip(CircleShape)
            .background(if (isNightMode) Color(0xFF1E293B) else Color(0xFFFFFFFF))
            .border(1.dp, if (isNightMode) Color(0xFF475569) else Color(0xFFCBD5E1), CircleShape)
            .clickable { onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            style = IdrTypography.titleMedium.copy(fontSize = 15.sp),
            color = if (isNightMode) Color.White else Color(0xFF0F172A),
            fontWeight = FontWeight.Bold
        )
    }
}

/**
 * Draws the background geographic basemap: landuse zones (parks/campuses) and water bodies (rivers).
 */
private fun DrawScope.drawBasemapFeatures(
    basemap: GeographicBasemap,
    vehicleX: Double,
    vehicleY: Double,
    centerX: Float,
    centerY: Float,
    scale: Float,
    isNightMode: Boolean
) {
    // 1. Draw Landuse Polygons (Parks & Campuses)
    for (land in basemap.landusePolygons) {
        val pts = land.pointsEnu
        if (pts.size < 3) continue
        val path = Path()
        var first = true
        for (p in pts) {
            val sx = centerX + ((p.x - vehicleX) * scale).toFloat()
            val sy = centerY - ((p.y - vehicleY) * scale).toFloat()
            if (first) {
                path.moveTo(sx, sy)
                first = false
            } else {
                path.lineTo(sx, sy)
            }
        }
        path.close()

        val isPark = land.type == "park" || land.type == "forest"
        val fillColor = if (isNightMode) {
            if (isPark) Color(0xFF143022) else Color(0xFF1E293B)
        } else {
            if (isPark) Color(0xFFD1FAE5) else Color(0xFFE2E8F0)
        }
        drawPath(path = path, color = fillColor)
        drawPath(path = path, color = fillColor.copy(alpha = 0.5f), style = Stroke(width = 1.5f))
    }

    // 2. Draw Water Bodies (River Ganga, River Yamuna)
    for (water in basemap.waterBodies) {
        val pts = water.pointsEnu
        if (pts.size < 2) continue
        val path = Path()
        var first = true
        for (p in pts) {
            val sx = centerX + ((p.x - vehicleX) * scale).toFloat()
            val sy = centerY - ((p.y - vehicleY) * scale).toFloat()
            if (first) {
                path.moveTo(sx, sy)
                first = false
            } else {
                path.lineTo(sx, sy)
            }
        }

        val riverWidthPx = (water.widthMeters * scale).coerceIn(12f, 250f)
        val glowColor = if (isNightMode) Color(0xFF1E3A8A).copy(alpha = 0.35f) else Color(0xFFBFDBFE).copy(alpha = 0.5f)
        val bodyColor = if (isNightMode) Color(0xFF0C2444) else Color(0xFF93C5FD)

        // Water Shoreline Glow
        drawPath(
            path = path,
            color = glowColor,
            style = Stroke(width = riverWidthPx + 8f, cap = StrokeCap.Round, join = StrokeJoin.Round)
        )
        // Main River Body
        drawPath(
            path = path,
            color = bodyColor,
            style = Stroke(width = riverWidthPx, cap = StrokeCap.Round, join = StrokeJoin.Round)
        )
    }
}

/**
 * Draws 3D building extrusions (footprints, isometric side walls, and shaded roofs).
 */
private fun DrawScope.draw3DBuildings(
    buildings: List<BuildingFeature>,
    vehicleX: Double,
    vehicleY: Double,
    centerX: Float,
    centerY: Float,
    scale: Float,
    is3dMode: Boolean,
    isNightMode: Boolean
) {
    val tiltFactor = if (is3dMode) 0.85f else 0.25f
    val shadowColor = if (isNightMode) Color(0x60000000) else Color(0x25000000)
    val wallColorDark = if (isNightMode) Color(0xFF1E293B) else Color(0xFF94A3B8)
    val wallColorLight = if (isNightMode) Color(0xFF334155) else Color(0xFFCBD5E1)
    val roofColor = if (isNightMode) Color(0xFF475569) else Color(0xFFE2E8F0)
    val roofBorderColor = if (isNightMode) Color(0xFF64748B) else Color(0xFFFFFFFF)

    for (b in buildings) {
        val pts = b.pointsEnu
        if (pts.size < 3) continue

        val groundPts = pts.map { p ->
            val sx = centerX + ((p.x - vehicleX) * scale).toFloat()
            val sy = centerY - ((p.y - vehicleY) * scale).toFloat()
            Offset(sx, sy)
        }

        val heightPx = (b.heightM * scale * tiltFactor).coerceIn(3f, 80f)
        val roofPts = groundPts.map { g ->
            Offset(g.x, g.y - heightPx)
        }

        // 1. Building Footprint Shadow
        val shadowPath = Path()
        var first = true
        for (g in groundPts) {
            if (first) {
                shadowPath.moveTo(g.x + 4f, g.y + 4f)
                first = false
            } else {
                shadowPath.lineTo(g.x + 4f, g.y + 4f)
            }
        }
        shadowPath.close()
        drawPath(shadowPath, color = shadowColor)

        // 2. Extruded Side Walls (Quad between ground[i], roof[i], roof[i+1], ground[i+1])
        val n = groundPts.size
        for (i in 0 until n) {
            val nextIdx = (i + 1) % n
            val g1 = groundPts[i]
            val g2 = groundPts[nextIdx]
            val r1 = roofPts[i]
            val r2 = roofPts[nextIdx]

            val isFacingLight = (g2.x - g1.x) >= 0

            val wallPath = Path().apply {
                moveTo(g1.x, g1.y)
                lineTo(r1.x, r1.y)
                lineTo(r2.x, r2.y)
                lineTo(g2.x, g2.y)
                close()
            }
            val wallColor = if (isFacingLight) wallColorLight else wallColorDark
            drawPath(wallPath, color = wallColor)
            drawPath(wallPath, color = wallColorDark, style = Stroke(width = 0.8f))
        }

        // 3. Roof Polygon
        val roofPath = Path()
        first = true
        for (r in roofPts) {
            if (first) {
                roofPath.moveTo(r.x, r.y)
                first = false
            } else {
                roofPath.lineTo(r.x, r.y)
            }
        }
        roofPath.close()
        drawPath(roofPath, color = roofColor)
        drawPath(roofPath, color = roofBorderColor, style = Stroke(width = 1.2f))
    }
}

/**
 * Draws the real geographic OpenStreetMap road network with hierarchical styling.
 */
private fun DrawScope.drawGeographicRoadNetwork(
    edges: List<RoadEdgeGeometry>,
    activeEdgeId: Int,
    vehicleX: Double,
    vehicleY: Double,
    centerX: Float,
    centerY: Float,
    scale: Float,
    isNightMode: Boolean
) {
    val motorColor = Color(0xFFF59E0B)       // Warm Amber/Gold for Motorways/Trunk
    val primaryColor = if (isNightMode) Color(0xFFE2E8F0) else Color(0xFFFFFFFF)     // High-contrast primary
    val secondaryColor = if (isNightMode) Color(0xFF94A3B8) else Color(0xFFCBD5E1)   // Slate gray
    val residentialColor = if (isNightMode) Color(0xFF475569) else Color(0xFF94A3B8) // Muted slate
    val bridgeColor = if (isNightMode) Color(0xFFF1F5F9) else Color(0xFFFFFFFF)      // Elevated Bridge white
    val tunnelColor = Color(0xFF64748B)      // Tunnel muted slate
    val activeColor = Color(0xFFFF6F00)      // Vivid ISRO Saffron
    val casingColor = if (isNightMode) Color(0xFF0F172A) else Color(0xFFCBD5E1)

    for (edge in edges) {
        val pts = edge.pointsEnu
        if (pts.size < 2) continue

        val isEdgeActive = (edge.edgeId == activeEdgeId)
        val path = Path()
        var first = true

        for (pt in pts) {
            val sx = centerX + ((pt.x - vehicleX) * scale).toFloat()
            val sy = centerY - ((pt.y - vehicleY) * scale).toFloat()
            if (first) {
                path.moveTo(sx, sy)
                first = false
            } else {
                path.lineTo(sx, sy)
            }
        }

        val baseWidth = when (edge.roadClass) {
            "motorway", "trunk" -> 5.5f
            "primary" -> 4.2f
            "secondary" -> 3.2f
            "tertiary" -> 2.4f
            else -> 1.8f
        }

        if (isEdgeActive) {
            // Snapped Active Matched Road Glow
            drawPath(
                path = path,
                color = activeColor.copy(alpha = 0.35f),
                style = Stroke(width = baseWidth + 10f, cap = StrokeCap.Round, join = StrokeJoin.Round)
            )
            // Active Matched Road Solid Line
            drawPath(
                path = path,
                color = activeColor,
                style = Stroke(width = baseWidth + 2.5f, cap = StrokeCap.Round, join = StrokeJoin.Round)
            )
        } else if (edge.isBridge || edge.layer > 0) {
            // Flyover / Elevated Bridge: Cast shadow + dual-rail casing
            drawPath(
                path = path,
                color = Color(0x60000000),
                style = Stroke(width = baseWidth + 5f, cap = StrokeCap.Square)
            )
            drawPath(
                path = path,
                color = bridgeColor,
                style = Stroke(width = baseWidth + 1.5f, cap = StrokeCap.Round)
            )
        } else if (edge.isTunnel || edge.layer < 0) {
            // Subterranean Tunnel: Dashed path
            drawPath(
                path = path,
                color = tunnelColor,
                style = Stroke(
                    width = baseWidth,
                    cap = StrokeCap.Butt,
                    pathEffect = PathEffect.dashPathEffect(floatArrayOf(8f, 6f), 0f)
                )
            )
        } else {
            // Standard Surface Road styled by hierarchy
            val roadColor = when (edge.roadClass) {
                "motorway", "trunk" -> motorColor
                "primary" -> primaryColor
                "secondary" -> secondaryColor
                else -> residentialColor
            }
            // Road Casing
            drawPath(
                path = path,
                color = casingColor,
                style = Stroke(width = baseWidth + 2f, cap = StrokeCap.Round, join = StrokeJoin.Round)
            )
            // Road Surface
            drawPath(
                path = path,
                color = roadColor,
                style = Stroke(width = baseWidth, cap = StrokeCap.Round, join = StrokeJoin.Round)
            )
        }
    }
}

/**
 * Draws landmark labels and place names on the map.
 */
private fun DrawScope.drawLandmarkLabels(
    landmarks: List<LandmarkFeature>,
    vehicleX: Double,
    vehicleY: Double,
    centerX: Float,
    centerY: Float,
    scale: Float,
    isNightMode: Boolean
) {
    val textColor = if (isNightMode) android.graphics.Color.WHITE else android.graphics.Color.DKGRAY
    val shadowColor = if (isNightMode) android.graphics.Color.BLACK else android.graphics.Color.WHITE

    val paint = Paint().apply {
        color = textColor
        textSize = 28f
        isAntiAlias = true
        setShadowLayer(4f, 0f, 0f, shadowColor)
    }

    drawIntoCanvas { canvas ->
        for (lm in landmarks) {
            val sx = centerX + ((lm.enuX - vehicleX) * scale).toFloat()
            val sy = centerY - ((lm.enuY - vehicleY) * scale).toFloat()
            canvas.nativeCanvas.drawText(lm.name, sx + 8f, sy - 8f, paint)
        }
    }
}

/**
 * Draws subtle tactical metric grid lines.
 */
private fun DrawScope.drawTacticalGrid(
    centerX: Float,
    centerY: Float,
    vehicleX: Double,
    vehicleY: Double,
    scale: Float,
    width: Float,
    height: Float,
    isNightMode: Boolean
) {
    val gridSizeMeters = when {
        scale > 5.0f -> 20f
        scale > 1.0f -> 100f
        scale > 0.2f -> 250f
        else -> 1000f
    }
    val gridPx = gridSizeMeters * scale

    val originScreenX = centerX - (vehicleX * scale).toFloat()
    val originScreenY = centerY + (vehicleY * scale).toFloat()

    val gridColor = if (isNightMode) Color(0xFF1E293B).copy(alpha = 0.5f) else Color(0xFFCBD5E1).copy(alpha = 0.5f)
    val halfW = width
    val halfH = height

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
}

/**
 * Draws smooth trajectories with bounded history.
 */
private fun DrawScope.drawSmoothTrajectory(
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
        val screenY = centerY - ((pt.enuY - vehicleY) * scale).toFloat()

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
        style = Stroke(width = strokeWidth, cap = StrokeCap.Round, join = StrokeJoin.Round)
    )
}

/**
 * Draws a professional automotive/aerospace vehicle marker with forward illuminated beam.
 */
private fun DrawScope.drawAutomotiveVehicleMarker(
    centerX: Float,
    centerY: Float,
    headingDeg: Float
) {
    rotate(degrees = headingDeg, pivot = Offset(centerX, centerY)) {
        // Forward illumination beam cone
        val conePath = Path().apply {
            moveTo(centerX, centerY)
            lineTo(centerX - 24f, centerY - 65f)
            lineTo(centerX + 24f, centerY - 65f)
            close()
        }
        drawPath(
            path = conePath,
            color = Color(0xFFFF6F00).copy(alpha = 0.22f)
        )

        // Vehicle Chevron Icon
        val chevronPath = Path().apply {
            moveTo(centerX, centerY - 18f)        // Tip pointing forward
            lineTo(centerX + 12f, centerY + 14f)  // Right wing
            lineTo(centerX, centerY + 6f)         // Center indent
            lineTo(centerX - 12f, centerY + 14f)  // Left wing
            close()
        }

        // Pulse Glow Ring
        drawCircle(
            color = Color(0xFFFF6F00).copy(alpha = 0.35f),
            radius = 24f,
            center = Offset(centerX, centerY)
        )

        // Chevron body in Saffron with dark border
        drawPath(path = chevronPath, color = Color(0xFFFF6F00))
        drawPath(path = chevronPath, color = Color.White, style = Stroke(width = 1.8f))
    }
}

/**
 * Draws compass north indicator in the top right.
 */
private fun DrawScope.drawNorthCompass(
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
        drawPath(path = pathNorth, color = Color(0xFFEF4444)) // Red North
        drawPath(path = pathSouth, color = Color(0xFF94A3B8)) // Gray South
    }
}

/**
 * Draws accurate distance scale bar.
 */
private fun DrawScope.drawAccurateScaleBar(
    posX: Float,
    posY: Float,
    scale: Float,
    isNightMode: Boolean
) {
    val targetMeters = when {
        scale > 10f -> 5f
        scale > 3f -> 20f
        scale > 1f -> 50f
        scale > 0.3f -> 100f
        scale > 0.08f -> 500f
        else -> 1000f
    }
    val barWidthPx = targetMeters * scale
    if (barWidthPx in 20f..260f) {
        val barBgColor = if (isNightMode) Color(0xFF1E293B).copy(alpha = 0.90f) else Color(0xFFFFFFFF).copy(alpha = 0.90f)
        val barLineColor = if (isNightMode) Color.White else Color(0xFF0F172A)

        drawRect(
            color = barBgColor,
            topLeft = Offset(posX - 6f, posY - 12f),
            size = androidx.compose.ui.geometry.Size(barWidthPx + 12f, 18f)
        )
        drawLine(
            color = barLineColor,
            start = Offset(posX, posY),
            end = Offset(posX + barWidthPx, posY),
            strokeWidth = 2.5f
        )
        drawLine(
            color = barLineColor,
            start = Offset(posX, posY - 5f),
            end = Offset(posX, posY + 5f),
            strokeWidth = 2f
        )
        drawLine(
            color = barLineColor,
            start = Offset(posX + barWidthPx, posY - 5f),
            end = Offset(posX + barWidthPx, posY + 5f),
            strokeWidth = 2f
        )
    }
}

/**
 * Computes scale to frame the active vehicle position, trajectories, and local road graph.
 */
private fun calculateDynamicScale(
    vehicleX: Double,
    vehicleY: Double,
    idrTrajectory: List<TrajectoryPoint>,
    gnssTrajectory: List<TrajectoryPoint>,
    roadGraph: RoadMapGraph?,
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

    val spanX = (maxX - minX).coerceAtLeast(60.0)
    val spanY = (maxY - minY).coerceAtLeast(60.0)

    val scaleX = (canvasWidth * 0.65f) / spanX.toFloat()
    val scaleY = (canvasHeight * 0.65f) / spanY.toFloat()

    return minOf(scaleX, scaleY).coerceIn(0.08f, 12.0f)
}

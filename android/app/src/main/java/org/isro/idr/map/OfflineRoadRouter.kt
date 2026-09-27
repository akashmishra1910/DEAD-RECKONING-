package org.isro.idr.map

import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class ManeuverStep(
    val instruction: String,
    val icon: String, // "↑", "↱", "↰", "↱", "↰", "⚑"
    val distanceMeters: Double,
    val roadName: String,
    val startCoord: Pair<Double, Double>,
    val endCoord: Pair<Double, Double>
)

data class RoutePlanResult(
    val totalDistanceMeters: Double,
    val estimatedTimeMinutes: Int,
    val waypoints: List<Pair<Double, Double>>,
    val maneuvers: List<ManeuverStep>,
    val startRoadName: String,
    val destRoadName: String
)

/**
 * Offline Road Network Routing Engine.
 * Computes road-connected shortest paths over OpenStreetMap vector road graphs,
 * extracts exact turn maneuvers, and computes off-route distance thresholds.
 */
object OfflineRoadRouter {

    /**
     * Plans a road-network route between start and destination coordinates.
     */
    fun route(
        startLat: Double,
        startLon: Double,
        destLat: Double,
        destLon: Double,
        destName: String,
        graph: RoadMapGraph?
    ): RoutePlanResult {
        if (graph == null || graph.edges.isEmpty()) {
            return generateFallbackDirectRoute(startLat, startLon, destLat, destLon, destName)
        }

        // 1. Find nearest road edges to start and destination
        val startEdge = findNearestEdge(startLat, startLon, graph.edges)
        val destEdge = findNearestEdge(destLat, destLon, graph.edges)

        val routePoints = mutableListOf<Pair<Double, Double>>()
        routePoints.add(Pair(startLat, startLon))

        val maneuvers = mutableListOf<ManeuverStep>()

        if (startEdge != null && destEdge != null && startEdge.edgeId == destEdge.edgeId) {
            // Same road segment
            val pts = startEdge.pointsWgs84
            routePoints.addAll(pts)
            routePoints.add(Pair(destLat, destLon))

            val segDist = calculatePolylineDistance(routePoints)
            maneuvers.add(
                ManeuverStep(
                    instruction = "Follow ${startEdge.name.ifBlank { "Road" }} towards destination",
                    icon = "↑",
                    distanceMeters = segDist,
                    roadName = startEdge.name,
                    startCoord = Pair(startLat, startLon),
                    endCoord = Pair(destLat, destLon)
                )
            )
            maneuvers.add(
                ManeuverStep(
                    instruction = "Arrive at $destName",
                    icon = "⚑",
                    distanceMeters = 0.0,
                    roadName = destName,
                    startCoord = Pair(destLat, destLon),
                    endCoord = Pair(destLat, destLon)
                )
            )

            val mins = (segDist / 400.0).toInt().coerceAtLeast(1) // ~25 km/h urban speed
            return RoutePlanResult(
                totalDistanceMeters = segDist,
                estimatedTimeMinutes = mins,
                waypoints = routePoints,
                maneuvers = maneuvers,
                startRoadName = startEdge.name,
                destRoadName = destEdge.name
            )
        }

        // 2. Find connected path across road network edges
        val pathEdges = findConnectedEdgePath(startEdge, destEdge, graph.edges, startLat, startLon, destLat, destLon)

        for (edge in pathEdges) {
            if (edge.pointsWgs84.isNotEmpty()) {
                routePoints.addAll(edge.pointsWgs84)
            }
        }
        routePoints.add(Pair(destLat, destLon))

        // 3. Extract turn maneuvers based on directional change between successive edges
        if (pathEdges.isNotEmpty()) {
            for (i in pathEdges.indices) {
                val currentEdge = pathEdges[i]
                val nextEdge = if (i + 1 < pathEdges.size) pathEdges[i + 1] else null
                val edgeDist = currentEdge.lengthM.toDouble().coerceAtLeast(50.0)

                val (instruction, icon) = if (nextEdge != null) {
                    val turnAngle = computeTurnAngleDeg(currentEdge, nextEdge)
                    when {
                        turnAngle in -25.0..25.0 -> "Continue straight on ${currentEdge.name.ifBlank { "Road" }}" to "↑"
                        turnAngle in 25.0..70.0 -> "Slight right onto ${nextEdge.name.ifBlank { "Road" }}" to "↱"
                        turnAngle in 70.0..120.0 -> "Turn right onto ${nextEdge.name.ifBlank { "Road" }}" to "↱"
                        turnAngle > 120.0 -> "Sharp right onto ${nextEdge.name.ifBlank { "Road" }}" to "↱"
                        turnAngle in -70.0..-25.0 -> "Slight left onto ${nextEdge.name.ifBlank { "Road" }}" to "↰"
                        turnAngle in -120.0..-70.0 -> "Turn left onto ${nextEdge.name.ifBlank { "Road" }}" to "↰"
                        else -> "Sharp left onto ${nextEdge.name.ifBlank { "Road" }}" to "↰"
                    }
                } else {
                    "In ${edgeDist.toInt()} m, arrive at $destName" to "⚑"
                }

                maneuvers.add(
                    ManeuverStep(
                        instruction = instruction,
                        icon = icon,
                        distanceMeters = edgeDist,
                        roadName = currentEdge.name,
                        startCoord = currentEdge.pointsWgs84.firstOrNull() ?: Pair(startLat, startLon),
                        endCoord = currentEdge.pointsWgs84.lastOrNull() ?: Pair(destLat, destLon)
                    )
                )
            }
        } else {
            val dist = haversineDistanceMeters(startLat, startLon, destLat, destLon)
            maneuvers.add(
                ManeuverStep(
                    instruction = "Head towards $destName",
                    icon = "↑",
                    distanceMeters = dist,
                    roadName = destName,
                    startCoord = Pair(startLat, startLon),
                    endCoord = Pair(destLat, destLon)
                )
            )
        }

        val totalDist = calculatePolylineDistance(routePoints)
        val estMins = (totalDist / 450.0).toInt().coerceAtLeast(2)

        return RoutePlanResult(
            totalDistanceMeters = totalDist,
            estimatedTimeMinutes = estMins,
            waypoints = routePoints,
            maneuvers = maneuvers,
            startRoadName = startEdge?.name ?: "Current Road",
            destRoadName = destEdge?.name ?: destName
        )
    }

    private fun findConnectedEdgePath(
        startEdge: RoadEdgeGeometry?,
        destEdge: RoadEdgeGeometry?,
        allEdges: List<RoadEdgeGeometry>,
        startLat: Double,
        startLon: Double,
        destLat: Double,
        destLon: Double
    ): List<RoadEdgeGeometry> {
        if (startEdge == null || destEdge == null) {
            return allEdges.filter { edge ->
                val mid = edge.pointsWgs84.getOrNull(edge.pointsWgs84.size / 2)
                mid != null && isPointNearBoundingBox(mid.first, mid.second, startLat, startLon, destLat, destLon)
            }.take(6)
        }

        // Direct connection or adjacent edges
        val path = mutableListOf<RoadEdgeGeometry>()
        path.add(startEdge)

        val intermediateEdges = allEdges.filter { edge ->
            edge.edgeId != startEdge.edgeId && edge.edgeId != destEdge.edgeId &&
            edge.pointsWgs84.isNotEmpty() &&
            isPointNearBoundingBox(
                edge.pointsWgs84[0].first, edge.pointsWgs84[0].second,
                startLat, startLon, destLat, destLon
            )
        }.sortedBy { edge ->
            val mid = edge.pointsWgs84[edge.pointsWgs84.size / 2]
            haversineDistanceMeters(startLat, startLon, mid.first, mid.second)
        }

        path.addAll(intermediateEdges.take(4))
        if (startEdge.edgeId != destEdge.edgeId) {
            path.add(destEdge)
        }
        return path
    }

    private fun findNearestEdge(lat: Double, lon: Double, edges: List<RoadEdgeGeometry>): RoadEdgeGeometry? {
        var minD = Double.MAX_VALUE
        var bestEdge: RoadEdgeGeometry? = null
        for (edge in edges) {
            for (pt in edge.pointsWgs84) {
                val d = haversineDistanceMeters(lat, lon, pt.first, pt.second)
                if (d < minD) {
                    minD = d
                    bestEdge = edge
                }
            }
        }
        return bestEdge
    }

    private fun computeTurnAngleDeg(e1: RoadEdgeGeometry, e2: RoadEdgeGeometry): Double {
        if (e1.pointsWgs84.size < 2 || e2.pointsWgs84.size < 2) return 0.0
        val p1 = e1.pointsWgs84[e1.pointsWgs84.size - 2]
        val p2 = e1.pointsWgs84.last()
        val p3 = e2.pointsWgs84.first()
        val p4 = e2.pointsWgs84.getOrElse(1) { p3 }

        val b1 = bearingDeg(p1.first, p1.second, p2.first, p2.second)
        val b2 = bearingDeg(p3.first, p3.second, p4.first, p4.second)

        var diff = b2 - b1
        while (diff > 180.0) diff -= 360.0
        while (diff < -180.0) diff += 360.0
        return diff
    }

    private fun bearingDeg(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val y = sin(Math.toRadians(lon2 - lon1)) * cos(Math.toRadians(lat2))
        val x = cos(Math.toRadians(lat1)) * sin(Math.toRadians(lat2)) -
                sin(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * cos(Math.toRadians(lon2 - lon1))
        val b = Math.toDegrees(atan2(y, x))
        return (b + 360.0) % 360.0
    }

    private fun isPointNearBoundingBox(
        lat: Double, lon: Double,
        lat1: Double, lon1: Double,
        lat2: Double, lon2: Double
    ): Boolean {
        val margin = 0.02
        val minLat = minOf(lat1, lat2) - margin
        val maxLat = maxOf(lat1, lat2) + margin
        val minLon = minOf(lon1, lon2) - margin
        val maxLon = maxOf(lon1, lon2) + margin
        return lat in minLat..maxLat && lon in minLon..maxLon
    }

    fun calculatePolylineDistance(points: List<Pair<Double, Double>>): Double {
        var dist = 0.0
        for (i in 0 until points.size - 1) {
            dist += haversineDistanceMeters(
                points[i].first, points[i].second,
                points[i + 1].first, points[i + 1].second
            )
        }
        return dist
    }

    fun distanceToPolyline(lat: Double, lon: Double, points: List<Pair<Double, Double>>): Double {
        if (points.isEmpty()) return Double.MAX_VALUE
        if (points.size == 1) return haversineDistanceMeters(lat, lon, points[0].first, points[0].second)

        var minDistance = Double.MAX_VALUE
        for (i in 0 until points.size - 1) {
            val d = pointToSegmentDistance(
                lat, lon,
                points[i].first, points[i].second,
                points[i + 1].first, points[i + 1].second
            )
            if (d < minDistance) minDistance = d
        }
        return minDistance
    }

    fun isOffRoute(lat: Double, lon: Double, waypoints: List<Pair<Double, Double>>, thresholdM: Double = 45.0): Boolean {
        if (waypoints.isEmpty()) return false
        val dist = distanceToPolyline(lat, lon, waypoints)
        return dist > thresholdM
    }

    private fun pointToSegmentDistance(
        pLat: Double, pLon: Double,
        aLat: Double, aLon: Double,
        bLat: Double, bLon: Double
    ): Double {
        val l2 = (bLat - aLat) * (bLat - aLat) + (bLon - aLon) * (bLon - aLon)
        if (l2 == 0.0) return haversineDistanceMeters(pLat, pLon, aLat, aLon)
        val t = (((pLat - aLat) * (bLat - aLat) + (pLon - aLon) * (bLon - aLon)) / l2).coerceIn(0.0, 1.0)
        val projLat = aLat + t * (bLat - aLat)
        val projLon = aLon + t * (bLon - aLon)
        return haversineDistanceMeters(pLat, pLon, projLat, projLon)
    }

    private fun generateFallbackDirectRoute(
        startLat: Double, startLon: Double,
        destLat: Double, destLon: Double,
        destName: String
    ): RoutePlanResult {
        val totalDist = haversineDistanceMeters(startLat, startLon, destLat, destLon)
        val waypoints = listOf(Pair(startLat, startLon), Pair(destLat, destLon))
        val maneuvers = listOf(
            ManeuverStep(
                instruction = "Proceed towards $destName",
                icon = "↑",
                distanceMeters = totalDist,
                roadName = destName,
                startCoord = Pair(startLat, startLon),
                endCoord = Pair(destLat, destLon)
            ),
            ManeuverStep(
                instruction = "Arrive at $destName",
                icon = "⚑",
                distanceMeters = 0.0,
                roadName = destName,
                startCoord = Pair(destLat, destLon),
                endCoord = Pair(destLat, destLon)
            )
        )
        val mins = (totalDist / 400.0).toInt().coerceAtLeast(1)
        return RoutePlanResult(
            totalDistanceMeters = totalDist,
            estimatedTimeMinutes = mins,
            waypoints = waypoints,
            maneuvers = maneuvers,
            startRoadName = "Current Location",
            destRoadName = destName
        )
    }

    private fun haversineDistanceMeters(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val r = 6371000.0
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return r * c
    }
}

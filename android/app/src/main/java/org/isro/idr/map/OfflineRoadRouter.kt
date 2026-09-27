package org.isro.idr.map

import java.util.PriorityQueue
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
 * Production Offline Road Network Routing Engine.
 * Builds an in-memory topological graph from OpenStreetMap road edges,
 * computes exact shortest paths using Dijkstra's algorithm,
 * connects continuous road geometry without zig-zags,
 * and extracts turn-by-turn maneuvers.
 */
object OfflineRoadRouter {

    private data class GraphEdge(
        val targetNodeId: Int,
        val weightMeters: Double,
        val roadEdge: RoadEdgeGeometry,
        val isForward: Boolean
    )

    private data class PathNode(
        val nodeId: Int,
        val distance: Double
    ) : Comparable<PathNode> {
        override fun compareTo(other: PathNode): Int = distance.compareTo(other.distance)
    }

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

        val validEdges = graph.edges.filter { it.pointsWgs84.size >= 2 }
        if (validEdges.isEmpty()) {
            return generateFallbackDirectRoute(startLat, startLon, destLat, destLon, destName)
        }

        // 1. Find nearest road edges to start and destination
        val startEdge = findNearestEdge(startLat, startLon, validEdges)
        val destEdge = findNearestEdge(destLat, destLon, validEdges)

        if (startEdge == null || destEdge == null) {
            return generateFallbackDirectRoute(startLat, startLon, destLat, destLon, destName)
        }

        // 2. If start and destination snap to the same road edge
        if (startEdge.edgeId == destEdge.edgeId) {
            val pts = extractSubPolyline(startEdge.pointsWgs84, startLat, startLon, destLat, destLon)
            val dist = calculatePolylineDistance(pts)
            val maneuvers = listOf(
                ManeuverStep(
                    instruction = "Follow ${startEdge.name.ifBlank { "Road" }} towards destination",
                    icon = "↑",
                    distanceMeters = dist,
                    roadName = startEdge.name.ifBlank { "Road" },
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
            val mins = (dist / 400.0).toInt().coerceAtLeast(1)
            return RoutePlanResult(
                totalDistanceMeters = dist,
                estimatedTimeMinutes = mins,
                waypoints = pts,
                maneuvers = maneuvers,
                startRoadName = startEdge.name.ifBlank { "Current Location" },
                destRoadName = destName
            )
        }

        // 3. Build topological graph and run Dijkstra shortest path
        val (nodeMap, adjacencyList) = buildTopologicalGraph(validEdges)

        val startNodeIdA = getNodeKey(startEdge.pointsWgs84.first(), nodeMap)
        val startNodeIdB = getNodeKey(startEdge.pointsWgs84.last(), nodeMap)
        val destNodeIdA = getNodeKey(destEdge.pointsWgs84.first(), nodeMap)
        val destNodeIdB = getNodeKey(destEdge.pointsWgs84.last(), nodeMap)

        val destNodeSet = setOf(destNodeIdA, destNodeIdB)
        val pathGraphEdges = runDijkstra(
            startNodes = listOf(startNodeIdA, startNodeIdB),
            destNodeSet = destNodeSet,
            adjacencyList = adjacencyList
        )

        val routePoints = mutableListOf<Pair<Double, Double>>()
        routePoints.add(Pair(startLat, startLon))

        val maneuvers = mutableListOf<ManeuverStep>()

        if (pathGraphEdges.isNotEmpty()) {
            var currentRoadName = ""
            var currentRoadDist = 0.0
            var currentManeuverIcon = "↑"
            var currentRoadStartCoord = Pair(startLat, startLon)

            for (i in pathGraphEdges.indices) {
                val ge = pathGraphEdges[i]
                val edgePts = if (ge.isForward) ge.roadEdge.pointsWgs84 else ge.roadEdge.pointsWgs84.reversed()
                
                // Add points avoiding duplicates
                for (pt in edgePts) {
                    val last = routePoints.lastOrNull()
                    if (last == null || haversineDistanceMeters(last.first, last.second, pt.first, pt.second) > 2.0) {
                        routePoints.add(pt)
                    }
                }

                val edgeRoadName = ge.roadEdge.name.ifBlank { "Main Road" }
                val edgeDist = ge.weightMeters

                if (i == 0) {
                    currentRoadName = edgeRoadName
                    currentRoadDist = edgeDist
                    currentRoadStartCoord = edgePts.first()
                } else {
                    val prevEdge = pathGraphEdges[i - 1]
                    val turnAngle = computeTurnAngle(prevEdge, ge)

                    if (Math.abs(turnAngle) > 25.0 || edgeRoadName != currentRoadName) {
                        val instruction = when {
                            turnAngle in 25.0..65.0 -> "Slight right onto $edgeRoadName"
                            turnAngle in 65.0..120.0 -> "Turn right onto $edgeRoadName"
                            turnAngle > 120.0 -> "Sharp right onto $edgeRoadName"
                            turnAngle in -65.0..-25.0 -> "Slight left onto $edgeRoadName"
                            turnAngle in -120.0..-65.0 -> "Turn left onto $edgeRoadName"
                            turnAngle < -120.0 -> "Sharp left onto $edgeRoadName"
                            else -> "Continue on $edgeRoadName"
                        }
                        val icon = when {
                            turnAngle > 25.0 -> "↱"
                            turnAngle < -25.0 -> "↰"
                            else -> "↑"
                        }

                        maneuvers.add(
                            ManeuverStep(
                                instruction = "In ${currentRoadDist.toInt()} m, $instruction",
                                icon = currentManeuverIcon,
                                distanceMeters = currentRoadDist,
                                roadName = currentRoadName,
                                startCoord = currentRoadStartCoord,
                                endCoord = edgePts.first()
                            )
                        )
                        currentRoadName = edgeRoadName
                        currentRoadDist = edgeDist
                        currentManeuverIcon = icon
                        currentRoadStartCoord = edgePts.first()
                    } else {
                        currentRoadDist += edgeDist
                    }
                }
            }

            // Final arrival maneuver
            maneuvers.add(
                ManeuverStep(
                    instruction = "In ${currentRoadDist.toInt()} m, arrive at $destName",
                    icon = "⚑",
                    distanceMeters = currentRoadDist,
                    roadName = currentRoadName,
                    startCoord = currentRoadStartCoord,
                    endCoord = Pair(destLat, destLon)
                )
            )
        } else {
            // If graph is partitioned, connect startEdge and destEdge smoothly
            val startPts = startEdge.pointsWgs84
            val destPts = destEdge.pointsWgs84
            routePoints.addAll(startPts)
            routePoints.addAll(destPts)
            routePoints.add(Pair(destLat, destLon))

            val dist = calculatePolylineDistance(routePoints)
            maneuvers.add(
                ManeuverStep(
                    instruction = "Proceed on ${startEdge.name.ifBlank { "Road" }} towards $destName",
                    icon = "↑",
                    distanceMeters = dist,
                    roadName = startEdge.name.ifBlank { "Road" },
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
        }

        // Ensure destination coordinate is attached cleanly
        val lastPt = routePoints.lastOrNull()
        if (lastPt == null || haversineDistanceMeters(lastPt.first, lastPt.second, destLat, destLon) > 5.0) {
            routePoints.add(Pair(destLat, destLon))
        }

        val totalDist = calculatePolylineDistance(routePoints)
        val estMins = (totalDist / 450.0).toInt().coerceAtLeast(2)

        return RoutePlanResult(
            totalDistanceMeters = totalDist,
            estimatedTimeMinutes = estMins,
            waypoints = routePoints,
            maneuvers = maneuvers,
            startRoadName = startEdge.name.ifBlank { "Current Location" },
            destRoadName = destEdge.name.ifBlank { destName }
        )
    }

    private fun buildTopologicalGraph(
        edges: List<RoadEdgeGeometry>
    ): Pair<MutableMap<String, Int>, MutableMap<Int, MutableList<GraphEdge>>> {
        val nodeMap = mutableMapOf<String, Int>()
        val adjacencyList = mutableMapOf<Int, MutableList<GraphEdge>>()

        for (edge in edges) {
            if (edge.pointsWgs84.size < 2) continue
            val uKey = getNodeKey(edge.pointsWgs84.first(), nodeMap)
            val vKey = getNodeKey(edge.pointsWgs84.last(), nodeMap)
            val weight = if (edge.lengthM > 1f) edge.lengthM.toDouble() else calculatePolylineDistance(edge.pointsWgs84)

            // Forward direction
            adjacencyList.computeIfAbsent(uKey) { mutableListOf() }
                .add(GraphEdge(vKey, weight, edge, isForward = true))

            // Reverse direction if two-way road
            if (!edge.isOneway) {
                adjacencyList.computeIfAbsent(vKey) { mutableListOf() }
                    .add(GraphEdge(uKey, weight, edge, isForward = false))
            }
        }
        return Pair(nodeMap, adjacencyList)
    }

    private fun getNodeKey(pt: Pair<Double, Double>, nodeMap: MutableMap<String, Int>): Int {
        // Quantize coordinate to 4 decimal places (~11 meters) to merge road junctions
        val latQ = Math.round(pt.first * 10000.0) / 10000.0
        val lonQ = Math.round(pt.second * 10000.0) / 10000.0
        val key = "$latQ,$lonQ"
        return nodeMap.computeIfAbsent(key) { nodeMap.size }
    }

    private fun runDijkstra(
        startNodes: List<Int>,
        destNodeSet: Set<Int>,
        adjacencyList: Map<Int, List<GraphEdge>>
    ): List<GraphEdge> {
        val distMap = mutableMapOf<Int, Double>()
        val prevEdgeMap = mutableMapOf<Int, GraphEdge>()
        val prevNodeMap = mutableMapOf<Int, Int>()
        val pq = PriorityQueue<PathNode>()

        for (s in startNodes) {
            distMap[s] = 0.0
            pq.add(PathNode(s, 0.0))
        }

        var targetFoundNode: Int? = null

        while (pq.isNotEmpty()) {
            val curr = pq.poll() ?: break
            val u = curr.nodeId
            val d = curr.distance

            if (d > (distMap[u] ?: Double.MAX_VALUE)) continue

            if (destNodeSet.contains(u)) {
                targetFoundNode = u
                break
            }

            val neighbors = adjacencyList[u] ?: emptyList()
            for (edge in neighbors) {
                val v = edge.targetNodeId
                val newDist = d + edge.weightMeters
                if (newDist < (distMap[v] ?: Double.MAX_VALUE)) {
                    distMap[v] = newDist
                    prevEdgeMap[v] = edge
                    prevNodeMap[v] = u
                    pq.add(PathNode(v, newDist))
                }
            }
        }

        if (targetFoundNode == null) return emptyList()

        val path = mutableListOf<GraphEdge>()
        var currNode = targetFoundNode
        while (currNode != null && prevEdgeMap.containsKey(currNode)) {
            val edge = prevEdgeMap[currNode]!!
            path.add(0, edge)
            currNode = prevNodeMap[currNode]
        }
        return path
    }

    private fun computeTurnAngle(e1: GraphEdge, e2: GraphEdge): Double {
        val pts1 = if (e1.isForward) e1.roadEdge.pointsWgs84 else e1.roadEdge.pointsWgs84.reversed()
        val pts2 = if (e2.isForward) e2.roadEdge.pointsWgs84 else e2.roadEdge.pointsWgs84.reversed()

        if (pts1.size < 2 || pts2.size < 2) return 0.0
        val p1 = pts1[pts1.size - 2]
        val p2 = pts1.last()
        val p3 = pts2.first()
        val p4 = pts2.getOrElse(1) { p3 }

        val b1 = bearingDeg(p1.first, p1.second, p2.first, p2.second)
        val b2 = bearingDeg(p3.first, p3.second, p4.first, p4.second)

        var diff = b2 - b1
        while (diff > 180.0) diff -= 360.0
        while (diff < -180.0) diff += 360.0
        return diff
    }

    private fun extractSubPolyline(
        points: List<Pair<Double, Double>>,
        startLat: Double,
        startLon: Double,
        destLat: Double,
        destLon: Double
    ): List<Pair<Double, Double>> {
        if (points.size <= 2) return points
        var startIdx = 0
        var minStartD = Double.MAX_VALUE
        var destIdx = points.size - 1
        var minDestD = Double.MAX_VALUE

        for (i in points.indices) {
            val d1 = haversineDistanceMeters(startLat, startLon, points[i].first, points[i].second)
            if (d1 < minStartD) {
                minStartD = d1
                startIdx = i
            }
            val d2 = haversineDistanceMeters(destLat, destLon, points[i].first, points[i].second)
            if (d2 < minDestD) {
                minDestD = d2
                destIdx = i
            }
        }

        return if (startIdx <= destIdx) {
            points.subList(startIdx, destIdx + 1)
        } else {
            points.subList(destIdx, startIdx + 1).reversed()
        }
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

    private fun bearingDeg(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val y = sin(Math.toRadians(lon2 - lon1)) * cos(Math.toRadians(lat2))
        val x = cos(Math.toRadians(lat1)) * sin(Math.toRadians(lat2)) -
                sin(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * cos(Math.toRadians(lon2 - lon1))
        val b = Math.toDegrees(atan2(y, x))
        return (b + 360.0) % 360.0
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

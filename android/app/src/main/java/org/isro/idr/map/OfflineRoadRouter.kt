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
 * performs forward/reverse geometry slicing to eliminate backtrack zig-zags,
 * and extracts turn-by-turn maneuvers along true road centerlines.
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

        // 2. If start and destination snap to the SAME road edge
        if (startEdge.edgeId == destEdge.edgeId) {
            val pts = extractSubPolyline(startEdge.pointsWgs84, startLat, startLon, destLat, destLon)
            val fullRoutePoints = mutableListOf<Pair<Double, Double>>()
            fullRoutePoints.add(Pair(startLat, startLon))
            for (p in pts) {
                if (haversineDistanceMeters(fullRoutePoints.last().first, fullRoutePoints.last().second, p.first, p.second) > 1.0) {
                    fullRoutePoints.add(p)
                }
            }
            if (haversineDistanceMeters(fullRoutePoints.last().first, fullRoutePoints.last().second, destLat, destLon) > 1.0) {
                fullRoutePoints.add(Pair(destLat, destLon))
            }

            val dist = calculatePolylineDistance(fullRoutePoints)
            val maneuvers = listOf(
                ManeuverStep(
                    instruction = "Follow ${startEdge.name.ifBlank { "Road" }} towards $destName",
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
                waypoints = fullRoutePoints,
                maneuvers = maneuvers,
                startRoadName = startEdge.name.ifBlank { "Current Location" },
                destRoadName = destName
            )
        }

        // 3. Build topological graph and find optimal edge sequence
        val (nodeMap, adjacencyList) = buildTopologicalGraph(validEdges)

        data class StartExitOption(val exitNodeId: Int, val dist: Double, val edge: GraphEdge)
        data class DestEntryOption(val entryNodeId: Int, val dist: Double, val edge: GraphEdge)

        val startOptions = mutableListOf<StartExitOption>()
        // Forward exit (at end node)
        val sSliceFwd = slicePolylineFromStartToEnd(startEdge.pointsWgs84, startLat, startLon)
        val fwdDist = calculatePolylineDistance(sSliceFwd)
        val endNodeKey = getNodeKey(startEdge.endNodeId, startEdge.pointsWgs84.last(), nodeMap)
        startOptions.add(StartExitOption(endNodeKey, fwdDist, GraphEdge(endNodeKey, fwdDist, startEdge, isForward = true)))

        // Reverse exit (at start node) if two-way
        if (!startEdge.isOneway) {
            val revPts = startEdge.pointsWgs84.reversed()
            val sSliceRev = slicePolylineFromStartToEnd(revPts, startLat, startLon)
            val revDist = calculatePolylineDistance(sSliceRev)
            val startNodeKey = getNodeKey(startEdge.startNodeId, startEdge.pointsWgs84.first(), nodeMap)
            startOptions.add(StartExitOption(startNodeKey, revDist, GraphEdge(startNodeKey, revDist, startEdge, isForward = false)))
        }

        val destOptions = mutableListOf<DestEntryOption>()
        // Forward entry (at start node)
        val dSliceFwd = slicePolylineFromStartToDest(destEdge.pointsWgs84, destLat, destLon)
        val destFwdDist = calculatePolylineDistance(dSliceFwd)
        val destStartNodeKey = getNodeKey(destEdge.startNodeId, destEdge.pointsWgs84.first(), nodeMap)
        destOptions.add(DestEntryOption(destStartNodeKey, destFwdDist, GraphEdge(getNodeKey(destEdge.endNodeId, destEdge.pointsWgs84.last(), nodeMap), destFwdDist, destEdge, isForward = true)))

        // Reverse entry (at end node) if two-way
        if (!destEdge.isOneway) {
            val revPts = destEdge.pointsWgs84.reversed()
            val dSliceRev = slicePolylineFromStartToDest(revPts, destLat, destLon)
            val destRevDist = calculatePolylineDistance(dSliceRev)
            val destEndNodeKey = getNodeKey(destEdge.endNodeId, destEdge.pointsWgs84.last(), nodeMap)
            destOptions.add(DestEntryOption(destEndNodeKey, destRevDist, GraphEdge(getNodeKey(destEdge.startNodeId, destEdge.pointsWgs84.first(), nodeMap), destRevDist, destEdge, isForward = false)))
        }

        var bestPathEdges: List<GraphEdge> = emptyList()
        var bestTotalDist = Double.MAX_VALUE

        for (sOpt in startOptions) {
            for (dOpt in destOptions) {
                if (sOpt.exitNodeId == dOpt.entryNodeId) {
                    // Direct connection at common node
                    val directDist = sOpt.dist + dOpt.dist
                    if (directDist < bestTotalDist) {
                        bestTotalDist = directDist
                        bestPathEdges = listOf(sOpt.edge, dOpt.edge)
                    }
                } else {
                    // Dijkstra search through intermediate edges
                    val dijkstraResult = runDijkstraFromTo(sOpt.exitNodeId, dOpt.entryNodeId, adjacencyList)
                    if (dijkstraResult != null) {
                        val total = sOpt.dist + dijkstraResult.second + dOpt.dist
                        if (total < bestTotalDist) {
                            bestTotalDist = total
                            bestPathEdges = listOf(sOpt.edge) + dijkstraResult.first + listOf(dOpt.edge)
                        }
                    }
                }
            }
        }

        val routePoints = mutableListOf<Pair<Double, Double>>()
        val maneuvers = mutableListOf<ManeuverStep>()

        if (bestPathEdges.isNotEmpty()) {
            var currentRoadName = ""
            var currentRoadDist = 0.0
            var currentManeuverIcon = "↑"
            var currentRoadStartCoord = Pair(startLat, startLon)

            for (i in bestPathEdges.indices) {
                val ge = bestPathEdges[i]
                val rawPts = if (ge.isForward) ge.roadEdge.pointsWgs84 else ge.roadEdge.pointsWgs84.reversed()

                val slicedPts = when {
                    i == 0 && bestPathEdges.size == 1 -> {
                        extractSubPolyline(rawPts, startLat, startLon, destLat, destLon)
                    }
                    i == 0 -> {
                        slicePolylineFromStartToEnd(rawPts, startLat, startLon)
                    }
                    i == bestPathEdges.size - 1 -> {
                        slicePolylineFromStartToDest(rawPts, destLat, destLon)
                    }
                    else -> rawPts
                }

                for (pt in slicedPts) {
                    val last = routePoints.lastOrNull()
                    if (last == null || haversineDistanceMeters(last.first, last.second, pt.first, pt.second) > 0.5) {
                        routePoints.add(pt)
                    }
                }

                val edgeRoadName = ge.roadEdge.name.ifBlank { "Main Road" }
                val edgeDist = calculatePolylineDistance(slicedPts)

                if (i == 0) {
                    currentRoadName = edgeRoadName
                    currentRoadDist = edgeDist
                    currentRoadStartCoord = slicedPts.firstOrNull() ?: Pair(startLat, startLon)
                } else {
                    val prevEdge = bestPathEdges[i - 1]
                    val turnAngle = computeTurnAngle(prevEdge, ge)

                    if (Math.abs(turnAngle) > 25.0 || edgeRoadName != currentRoadName) {
                        val instruction = when {
                            turnAngle in 25.0..65.0 -> "Slight right onto $edgeRoadName"
                            turnAngle in 65.0..120.0 -> "Turn right onto $edgeRoadName"
                            turnAngle > 120.0 -> "Sharp right onto $edgeRoadName"
                            turnAngle in -65.0..-25.0 -> "Slight left onto $edgeRoadName"
                            turnAngle in -120.0..-65.0 -> "Turn left onto $edgeRoadName"
                            turnAngle < -120.0 -> "Sharp left onto $edgeRoadName"
                            else -> "Continue onto $edgeRoadName"
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
                                endCoord = slicedPts.firstOrNull() ?: Pair(startLat, startLon)
                            )
                        )
                        currentRoadName = edgeRoadName
                        currentRoadDist = edgeDist
                        currentManeuverIcon = icon
                        currentRoadStartCoord = slicedPts.firstOrNull() ?: Pair(startLat, startLon)
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
            // Disconnected fallback
            val sSlice = startEdge.pointsWgs84
            val dSlice = destEdge.pointsWgs84
            routePoints.addAll(sSlice)
            routePoints.addAll(dSlice)
            val dist = calculatePolylineDistance(routePoints)
            maneuvers.add(
                ManeuverStep(
                    instruction = "Follow ${startEdge.name.ifBlank { "Road" }} towards $destName",
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
            val uKey = getNodeKey(edge.startNodeId, edge.pointsWgs84.first(), nodeMap)
            val vKey = getNodeKey(edge.endNodeId, edge.pointsWgs84.last(), nodeMap)
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

    private fun getNodeKey(explicitNodeId: Int, pt: Pair<Double, Double>, nodeMap: MutableMap<String, Int>): Int {
        if (explicitNodeId > 0) {
            val key = "node_$explicitNodeId"
            return nodeMap.computeIfAbsent(key) { nodeMap.size }
        }
        // Spatial clustering to 4 decimal places (~11 meters)
        val latQ = Math.round(pt.first * 10000.0) / 10000.0
        val lonQ = Math.round(pt.second * 10000.0) / 10000.0
        val key = "geo_${latQ}_${lonQ}"
        return nodeMap.computeIfAbsent(key) { nodeMap.size }
    }

    private fun runDijkstraFromTo(
        startNode: Int,
        destNode: Int,
        adjacencyList: Map<Int, List<GraphEdge>>
    ): Pair<List<GraphEdge>, Double>? {
        if (startNode == destNode) return Pair(emptyList(), 0.0)

        val distMap = mutableMapOf<Int, Double>()
        val prevEdgeMap = mutableMapOf<Int, GraphEdge>()
        val prevNodeMap = mutableMapOf<Int, Int>()
        val pq = PriorityQueue<PathNode>()

        distMap[startNode] = 0.0
        pq.add(PathNode(startNode, 0.0))

        var targetFound = false

        while (pq.isNotEmpty()) {
            val curr = pq.poll() ?: break
            val u = curr.nodeId
            val d = curr.distance

            if (d > (distMap[u] ?: Double.MAX_VALUE)) continue

            if (u == destNode) {
                targetFound = true
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

        if (!targetFound) return null

        val path = mutableListOf<GraphEdge>()
        var currNode = destNode
        while (currNode != startNode && prevEdgeMap.containsKey(currNode)) {
            val edge = prevEdgeMap[currNode]!!
            path.add(0, edge)
            currNode = prevNodeMap[currNode]!!
        }
        return Pair(path, distMap[destNode] ?: 0.0)
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

    private fun findNearestPointIndex(lat: Double, lon: Double, points: List<Pair<Double, Double>>): Int {
        var minD = Double.MAX_VALUE
        var bestIdx = 0
        for (i in points.indices) {
            val d = haversineDistanceMeters(lat, lon, points[i].first, points[i].second)
            if (d < minD) {
                minD = d
                bestIdx = i
            }
        }
        return bestIdx
    }

    data class ProjectionResult(val segmentIndex: Int, val t: Double, val param: Double, val projCoord: Pair<Double, Double>)

    private fun projectPointOntoPolyline(
        lat: Double, lon: Double,
        points: List<Pair<Double, Double>>
    ): ProjectionResult {
        if (points.size < 2) {
            val p = points.firstOrNull() ?: Pair(lat, lon)
            return ProjectionResult(0, 0.0, 0.0, p)
        }
        var minD = Double.MAX_VALUE
        var bestSeg = 0
        var bestT = 0.0
        var bestProj = points.first()

        for (i in 0 until points.size - 1) {
            val a = points[i]
            val b = points[i + 1]
            val l2 = (b.first - a.first) * (b.first - a.first) + (b.second - a.second) * (b.second - a.second)
            val t = if (l2 == 0.0) 0.0 else (((lat - a.first) * (b.first - a.first) + (lon - a.second) * (b.second - a.second)) / l2).coerceIn(0.0, 1.0)
            val projLat = a.first + t * (b.first - a.first)
            val projLon = a.second + t * (b.second - a.second)
            val d = haversineDistanceMeters(lat, lon, projLat, projLon)
            if (d < minD) {
                minD = d
                bestSeg = i
                bestT = t
                bestProj = Pair(projLat, projLon)
            }
        }
        val param = bestSeg.toDouble() + bestT
        return ProjectionResult(bestSeg, bestT, param, bestProj)
    }

    private fun extractSubPolyline(
        points: List<Pair<Double, Double>>,
        startLat: Double,
        startLon: Double,
        destLat: Double,
        destLon: Double
    ): List<Pair<Double, Double>> {
        if (points.isEmpty()) return emptyList()
        if (points.size == 1) return points

        val pStart = projectPointOntoPolyline(startLat, startLon, points)
        val pDest = projectPointOntoPolyline(destLat, destLon, points)

        val result = mutableListOf<Pair<Double, Double>>()
        result.add(pStart.projCoord)

        if (pStart.param <= pDest.param) {
            val startSeg = pStart.segmentIndex
            val destSeg = pDest.segmentIndex
            for (j in (startSeg + 1)..destSeg) {
                if (j < points.size) {
                    result.add(points[j])
                }
            }
        } else {
            val startSeg = pStart.segmentIndex
            val destSeg = pDest.segmentIndex
            for (j in startSeg downTo (destSeg + 1)) {
                if (j < points.size) {
                    result.add(points[j])
                }
            }
        }

        result.add(pDest.projCoord)

        val clean = mutableListOf<Pair<Double, Double>>()
        for (pt in result) {
            val last = clean.lastOrNull()
            if (last == null || haversineDistanceMeters(last.first, last.second, pt.first, pt.second) > 0.5) {
                clean.add(pt)
            }
        }
        return clean
    }

    private fun slicePolylineFromStartToEnd(
        points: List<Pair<Double, Double>>,
        startLat: Double,
        startLon: Double
    ): List<Pair<Double, Double>> {
        if (points.isEmpty()) return emptyList()
        if (points.size == 1) return points

        val pStart = projectPointOntoPolyline(startLat, startLon, points)
        val result = mutableListOf<Pair<Double, Double>>()
        result.add(pStart.projCoord)

        for (j in (pStart.segmentIndex + 1) until points.size) {
            result.add(points[j])
        }

        val clean = mutableListOf<Pair<Double, Double>>()
        for (pt in result) {
            val last = clean.lastOrNull()
            if (last == null || haversineDistanceMeters(last.first, last.second, pt.first, pt.second) > 0.5) {
                clean.add(pt)
            }
        }
        return clean
    }

    private fun slicePolylineFromStartToDest(
        points: List<Pair<Double, Double>>,
        destLat: Double,
        destLon: Double
    ): List<Pair<Double, Double>> {
        if (points.isEmpty()) return emptyList()
        if (points.size == 1) return points

        val pDest = projectPointOntoPolyline(destLat, destLon, points)
        val result = mutableListOf<Pair<Double, Double>>()

        for (j in 0..pDest.segmentIndex) {
            if (j < points.size) {
                result.add(points[j])
            }
        }
        result.add(pDest.projCoord)

        val clean = mutableListOf<Pair<Double, Double>>()
        for (pt in result) {
            val last = clean.lastOrNull()
            if (last == null || haversineDistanceMeters(last.first, last.second, pt.first, pt.second) > 0.5) {
                clean.add(pt)
            }
        }
        return clean
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

package org.isro.idr.map

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class OfflineNavigationModulesTest {

    private fun createSyntheticRoadGraph(): RoadMapGraph {
        val edge1 = RoadEdgeGeometry(
            edgeId = 1,
            osmWayId = 101L,
            name = "Mahatma Gandhi Marg",
            roadClass = "primary",
            layer = 0,
            isBridge = false,
            isTunnel = false,
            isOneway = false,
            maxspeedKmh = 50,
            lengthM = 800.0f,
            pointsWgs84 = listOf(
                Pair(25.4350, 81.8280),
                Pair(25.4350, 81.8350),
                Pair(25.4350, 81.8420)
            ),
            startNodeId = 1001,
            endNodeId = 1002
        )

        val edge2 = RoadEdgeGeometry(
            edgeId = 2,
            osmWayId = 102L,
            name = "Sardar Patel Marg",
            roadClass = "secondary",
            layer = 0,
            isBridge = false,
            isTunnel = false,
            isOneway = false,
            maxspeedKmh = 40,
            lengthM = 600.0f,
            pointsWgs84 = listOf(
                Pair(25.4350, 81.8420),
                Pair(25.4400, 81.8420),
                Pair(25.4450, 81.8420)
            ),
            startNodeId = 1002,
            endNodeId = 1003
        )

        val edge3 = RoadEdgeGeometry(
            edgeId = 3,
            osmWayId = 103L,
            name = "Civil Lines Connector",
            roadClass = "residential",
            layer = 0,
            isBridge = false,
            isTunnel = false,
            isOneway = false,
            maxspeedKmh = 30,
            lengthM = 500.0f,
            pointsWgs84 = listOf(
                Pair(25.4450, 81.8420),
                Pair(25.4450, 81.8350),
                Pair(25.4450, 81.8280)
            ),
            startNodeId = 1003,
            endNodeId = 1004
        )

        val landmarks = listOf(
            LandmarkFeature(name = "Sangam Confluence", lat = 25.4290, lon = 81.8840, category = "landmark"),
            LandmarkFeature(name = "Civil Lines Commercial Hub", lat = 25.4520, lon = 81.8340, category = "commercial")
        )

        val basemap = GeographicBasemap(
            name = "Test Basemap",
            minLat = 25.40,
            minLon = 81.80,
            maxLat = 25.50,
            maxLon = 81.90,
            landmarks = landmarks
        )

        return RoadMapGraph(
            mapName = "test_map",
            minLat = 25.40,
            minLon = 81.80,
            maxLat = 25.50,
            maxLon = 81.90,
            nodeCount = 6,
            edgeCount = 3,
            edges = listOf(edge1, edge2, edge3),
            basemap = basemap
        )
    }

    @Test
    fun testOfflineSearchEngineFindsLandmarksAndRoads() {
        val graph = createSyntheticRoadGraph()
        val results = OfflineSearchEngine.search("Gandhi", 25.4350, 81.8280, graph)

        assertTrue("Should find matching road", results.isNotEmpty())
        val first = results.first()
        assertEquals("Mahatma Gandhi Marg", first.title)
        assertEquals("road", first.category)
    }

    @Test
    fun testOfflineSearchEngineEmptyQueryReturnsSortedByDistance() {
        val graph = createSyntheticRoadGraph()
        val results = OfflineSearchEngine.search("", 25.4350, 81.8280, graph)

        assertTrue("Should return all landmarks/regional POIs", results.size >= 2)
        for (i in 0 until results.size - 1) {
            assertTrue("Results must be sorted ascending by distance", results[i].distanceMeters <= results[i + 1].distanceMeters)
        }
    }

    // 10 Comprehensive Route Acceptance Tests
    @Test
    fun testRoute1_SameRoadFullForward() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(25.4350, 81.8280, 25.4350, 81.8420, "East MG Marg", graph)
        assertNotNull(plan)
        assertTrue(plan.waypoints.size >= 3)
        assertTrue(plan.totalDistanceMeters in 1200.0..1600.0)
    }

    @Test
    fun testRoute2_SameRoadMidSegmentSlice() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(25.4350, 81.8300, 25.4350, 81.8380, "Mid MG Marg", graph)
        assertNotNull(plan)
        assertTrue(plan.waypoints.size >= 2)
        assertTrue(plan.totalDistanceMeters in 600.0..1000.0)
    }

    @Test
    fun testRoute3_SameRoadReverseDirection() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(25.4350, 81.8420, 25.4350, 81.8280, "West MG Marg", graph)
        assertNotNull(plan)
        assertTrue(plan.waypoints.size >= 3)
        assertTrue(plan.totalDistanceMeters in 1200.0..1600.0)
    }

    @Test
    fun testRoute4_TwoRoadRightTurnIntersection() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(25.4350, 81.8280, 25.4450, 81.8420, "North Patel Marg", graph)
        assertNotNull(plan)
        assertTrue("Route must contain both road segments", plan.waypoints.size >= 4)
        assertTrue(plan.maneuvers.any { it.instruction.contains("Turn") || it.instruction.contains("Patel") })
    }

    @Test
    fun testRoute5_TwoRoadLeftTurnReverseIntersection() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(25.4450, 81.8420, 25.4350, 81.8280, "West MG Marg", graph)
        assertNotNull(plan)
        assertTrue(plan.waypoints.size >= 4)
        assertTrue(plan.totalDistanceMeters in 2200.0..2800.0)
    }

    @Test
    fun testRoute6_ThreeRoadSequenceUturn() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(25.4350, 81.8280, 25.4450, 81.8280, "Civil Lines West", graph)
        assertNotNull(plan)
        assertTrue("Must traverse all 3 edges", plan.waypoints.size >= 6)
        assertTrue(plan.totalDistanceMeters in 3500.0..4500.0)
    }

    @Test
    fun testRoute7_OneWayForwardTraversal() {
        val onewayEdge = RoadEdgeGeometry(
            edgeId = 10, osmWayId = 201L, name = "One Way Express",
            roadClass = "motorway", layer = 0, isBridge = false, isTunnel = false,
            isOneway = true, maxspeedKmh = 80, lengthM = 1000f,
            pointsWgs84 = listOf(Pair(25.4100, 81.8100), Pair(25.4200, 81.8100)),
            startNodeId = 2001, endNodeId = 2002
        )
        val graph = RoadMapGraph(
            mapName = "oneway_map", minLat = 25.40, minLon = 81.80, maxLat = 25.50, maxLon = 81.90,
            nodeCount = 2, edgeCount = 1, edges = listOf(onewayEdge)
        )
        val forwardPlan = OfflineRoadRouter.route(25.4100, 81.8100, 25.4200, 81.8100, "North End", graph)
        assertNotNull(forwardPlan)
        assertTrue(forwardPlan.waypoints.size >= 2)
    }

    @Test
    fun testRoute8_SnappingWithOffset() {
        val graph = createSyntheticRoadGraph()
        // Start 15m south of MG Marg
        val plan = OfflineRoadRouter.route(25.4348, 81.8300, 25.4452, 81.8420, "North Patel", graph)
        assertNotNull(plan)
        assertTrue(plan.waypoints.size >= 4)
    }

    @Test
    fun testRoute9_OffRouteDetectionOnLine() {
        val waypoints = listOf(
            Pair(25.4350, 81.8280),
            Pair(25.4350, 81.8350),
            Pair(25.4350, 81.8420)
        )
        val onLine = OfflineRoadRouter.isOffRoute(25.4350, 81.8300, waypoints, thresholdM = 30.0)
        assertFalse("Point on route should not be off-route", onLine)
    }

    @Test
    fun testRoute10_OffRouteDetectionDeviation() {
        val waypoints = listOf(
            Pair(25.4350, 81.8280),
            Pair(25.4350, 81.8350),
            Pair(25.4350, 81.8420)
        )
        val offLine = OfflineRoadRouter.isOffRoute(25.4400, 81.8300, waypoints, thresholdM = 30.0)
        assertTrue("Point 500m away must be off-route", offLine)
    }
}

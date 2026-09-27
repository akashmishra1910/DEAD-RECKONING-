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
            )
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
            )
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
            edgeCount = 2,
            edges = listOf(edge1, edge2),
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

    @Test
    fun testOfflineRoadRouterPlansRoadConnectedRoute() {
        val graph = createSyntheticRoadGraph()
        val plan = OfflineRoadRouter.route(
            startLat = 25.4350,
            startLon = 81.8280,
            destLat = 25.4450,
            destLon = 81.8420,
            destName = "Sardar Patel Junction",
            graph = graph
        )

        assertNotNull("Plan must not be null", plan)
        assertTrue("Route must have waypoints", plan.waypoints.size >= 4)
        assertTrue("Total distance must be positive", plan.totalDistanceMeters > 500.0)
        assertTrue("Maneuvers must contain turn steps", plan.maneuvers.isNotEmpty())
    }

    @Test
    fun testOffRouteDetection() {
        val waypoints = listOf(
            Pair(25.4350, 81.8280),
            Pair(25.4350, 81.8350),
            Pair(25.4350, 81.8420)
        )

        // Point right on line
        val onLine = OfflineRoadRouter.isOffRoute(25.4350, 81.8300, waypoints, thresholdM = 30.0)
        assertFalse("Point on route should not be off-route", onLine)

        // Point 200m away
        val offLine = OfflineRoadRouter.isOffRoute(25.4400, 81.8300, waypoints, thresholdM = 30.0)
        assertTrue("Point 500m away must be off-route", offLine)
    }

    @Test
    fun testOneWayRestrictionRouting() {
        val onewayEdge = RoadEdgeGeometry(
            edgeId = 10,
            osmWayId = 201L,
            name = "One Way Express",
            roadClass = "motorway",
            layer = 0,
            isBridge = false,
            isTunnel = false,
            isOneway = true,
            maxspeedKmh = 80,
            lengthM = 1000f,
            pointsWgs84 = listOf(Pair(25.4100, 81.8100), Pair(25.4200, 81.8100))
        )
        val graph = RoadMapGraph(
            mapName = "oneway_map",
            minLat = 25.40, minLon = 81.80, maxLat = 25.50, maxLon = 81.90,
            nodeCount = 2, edgeCount = 1, edges = listOf(onewayEdge)
        )

        val forwardPlan = OfflineRoadRouter.route(25.4100, 81.8100, 25.4200, 81.8100, "North End", graph)
        assertNotNull(forwardPlan)
        assertTrue(forwardPlan.waypoints.size >= 2)
    }
}

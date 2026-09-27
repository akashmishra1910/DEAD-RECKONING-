package org.isro.idr.map

import org.isro.idr.engine.Vector3D
import org.isro.idr.ingestion.GeoCoordinates

/**
 * Geometric and topological representation of an OpenStreetMap road edge
 * extracted from an offline .idrmap binary dataset.
 */
data class RoadEdgeGeometry(
    val edgeId: Int,
    val osmWayId: Long,
    val name: String,
    val roadClass: String,
    val layer: Int, // 0=ground, >0=bridge/flyover, <0=tunnel
    val isBridge: Boolean,
    val isTunnel: Boolean,
    val isOneway: Boolean,
    val maxspeedKmh: Int,
    val lengthM: Float,
    val pointsWgs84: List<Pair<Double, Double>>, // (lat, lon) pairs
    val pointsEnu: List<Vector3D> = emptyList(),   // (enuX, enuY, enuZ) in meters relative to anchor
    val startNodeId: Int = 0,
    val endNodeId: Int = 0
)

/**
 * Geographic water feature (e.g. River Ganga, River Yamuna) for visual basemap rendering.
 */
data class WaterBodyFeature(
    val name: String,
    val type: String, // "river", "lake", "canal"
    val coordinatesWgs84: List<Pair<Double, Double>>,
    val pointsEnu: List<Vector3D> = emptyList(),
    val widthMeters: Float = 150f
)

/**
 * Geographic landuse polygon (e.g. parks, universities, airports) for visual basemap rendering.
 */
data class LanduseFeature(
    val name: String,
    val type: String, // "park", "university", "forest", "industrial"
    val coordinatesWgs84: List<Pair<Double, Double>>,
    val pointsEnu: List<Vector3D> = emptyList()
)

/**
 * Geographic landmark / district label marker for visual basemap orientation.
 */
data class LandmarkFeature(
    val name: String,
    val lat: Double,
    val lon: Double,
    val category: String, // "district", "confluence", "transit", "civic", "park", "bridge"
    val enuX: Double = 0.0,
    val enuY: Double = 0.0
)

/**
 * 3D Building feature for realistic offline vector extrusion.
 */
data class BuildingFeature(
    val id: String,
    val name: String,
    val heightM: Float = 15f,
    val minHeightM: Float = 0f,
    val levels: Int = 3,
    val type: String = "civic",
    val coordinatesWgs84: List<Pair<Double, Double>>,
    val pointsEnu: List<Vector3D> = emptyList()
)

/**
 * Offline visual geographic basemap containing background terrain, water bodies,
 * landuse zones, 3D building footprints, and landmark place markers.
 */
data class GeographicBasemap(
    val name: String,
    val minLat: Double,
    val minLon: Double,
    val maxLat: Double,
    val maxLon: Double,
    val waterBodies: List<WaterBodyFeature> = emptyList(),
    val landusePolygons: List<LanduseFeature> = emptyList(),
    val landmarks: List<LandmarkFeature> = emptyList(),
    val buildings: List<BuildingFeature> = emptyList()
) {
    fun withEnuProjection(anchor: GeoCoordinates): GeographicBasemap {
        val projWater = waterBodies.map { w ->
            val enuPts = w.coordinatesWgs84.map { (lat, lon) ->
                anchor.geodeticToEnu(lat, lon, 0.0)
            }
            w.copy(pointsEnu = enuPts)
        }
        val projLand = landusePolygons.map { l ->
            val enuPts = l.coordinatesWgs84.map { (lat, lon) ->
                anchor.geodeticToEnu(lat, lon, 0.0)
            }
            l.copy(pointsEnu = enuPts)
        }
        val projLandmarks = landmarks.map { lm ->
            val enu = anchor.geodeticToEnu(lm.lat, lm.lon, 0.0)
            lm.copy(enuX = enu.x, enuY = enu.y)
        }
        val projBuildings = buildings.map { b ->
            val enuPts = b.coordinatesWgs84.map { (lat, lon) ->
                anchor.geodeticToEnu(lat, lon, 0.0)
            }
            b.copy(pointsEnu = enuPts)
        }
        return copy(
            waterBodies = projWater,
            landusePolygons = projLand,
            landmarks = projLandmarks,
            buildings = projBuildings
        )
    }
}

/**
 * In-memory topological road graph loaded from an offline .idrmap v1.0 binary dataset,
 * paired with an optional offline visual geographic basemap.
 */
data class RoadMapGraph(
    val mapName: String,
    val minLat: Double,
    val minLon: Double,
    val maxLat: Double,
    val maxLon: Double,
    val nodeCount: Int,
    val edgeCount: Int,
    val edges: List<RoadEdgeGeometry>,
    val attribution: String = "OpenStreetMap contributors / ISRO IDR",
    val basemap: GeographicBasemap? = null,
    val currentAnchor: GeoCoordinates? = null
) {
    val centerLat: Double get() = (minLat + maxLat) / 2.0
    val centerLon: Double get() = (minLon + maxLon) / 2.0

    /**
     * Projects all edge WGS-84 polyline coordinates and basemap features
     * to local tangent ENU coordinates using the specified geodetic anchor.
     */
    fun withEnuProjection(anchor: GeoCoordinates): RoadMapGraph {
        val projectedEdges = edges.map { edge ->
            val enuPts = edge.pointsWgs84.map { (lat, lon) ->
                anchor.geodeticToEnu(lat, lon, 0.0)
            }
            edge.copy(pointsEnu = enuPts)
        }
        val projectedBasemap = basemap?.withEnuProjection(anchor)
        return copy(
            edges = projectedEdges,
            basemap = projectedBasemap,
            currentAnchor = anchor
        )
    }

    /**
     * Checks if a geodetic coordinate (lat, lon) falls inside the bounding box of this map.
     */
    fun contains(lat: Double, lon: Double): Boolean {
        return lat in minLat..maxLat && lon in minLon..maxLon
    }
}

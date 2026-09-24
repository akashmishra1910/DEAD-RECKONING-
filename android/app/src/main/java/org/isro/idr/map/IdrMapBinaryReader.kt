package org.isro.idr.map

import org.json.JSONObject
import java.io.File
import java.io.InputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Fast, memory-efficient binary reader for compiled .idrmap v1.0 offline road datasets
 * and associated geographic basemap definitions.
 */
object IdrMapBinaryReader {

    private const val HEADER_SIZE = 128
    private const val EDGE_RECORD_SIZE = 40
    private const val GEOM_RECORD_SIZE = 16

    private val ROAD_CLASSES = arrayOf(
        "unknown", "motorway", "trunk", "primary", "secondary",
        "tertiary", "residential", "unclassified", "service", "living_street",
        "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"
    )

    fun readFromBytes(buffer: ByteArray, mapName: String = "Offline Road Map", basemap: GeographicBasemap? = null): RoadMapGraph? {
        if (buffer.size < HEADER_SIZE) return null

        val byteBuffer = ByteBuffer.wrap(buffer).order(ByteOrder.LITTLE_ENDIAN)

        // 1. Validate magic: 'IDRMAP\x01\x00'
        val magic = ByteArray(8)
        byteBuffer.get(magic)
        if (magic[0] != 'I'.code.toByte() || magic[1] != 'D'.code.toByte() ||
            magic[2] != 'R'.code.toByte() || magic[3] != 'M'.code.toByte() ||
            magic[4] != 'A'.code.toByte() || magic[5] != 'P'.code.toByte() ||
            magic[6] != 1.toByte() || magic[7] != 0.toByte()) {
            return null
        }

        byteBuffer.short // formatVersion
        byteBuffer.short // generatorVersion
        byteBuffer.int   // flags
        val minLat = byteBuffer.double
        val minLon = byteBuffer.double
        val maxLat = byteBuffer.double
        val maxLon = byteBuffer.double
        val nodeCount = byteBuffer.int
        val edgeCount = byteBuffer.int
        byteBuffer.int   // geomPointCount
        byteBuffer.int   // stringCount
        val stringTableBytes = byteBuffer.int
        byteBuffer.short // gridCols
        byteBuffer.short // gridRows
        byteBuffer.int   // gridCellCount
        byteBuffer.int   // gridIndexEntryCount
        byteBuffer.int   // offsetNodes
        val offsetEdges = byteBuffer.int
        val offsetGeometry = byteBuffer.int
        byteBuffer.int   // offsetSpatialIndex
        val offsetStringTable = byteBuffer.int
        val offsetAttribution = byteBuffer.int
        val attributionBytes = byteBuffer.int

        if (offsetEdges + edgeCount * EDGE_RECORD_SIZE > buffer.size) {
            return null // Truncated
        }

        // Helper to extract string from string table
        fun getString(offset: Int): String {
            if (offset < 0 || offset >= stringTableBytes) return ""
            val start = offsetStringTable + offset
            var end = start
            while (end < buffer.size && end < offsetStringTable + stringTableBytes && buffer[end] != 0.toByte()) {
                end++
            }
            return if (end > start) String(buffer, start, end - start, Charsets.UTF_8) else ""
        }

        // 2. Read Edge Geometry Records
        val edges = ArrayList<RoadEdgeGeometry>(edgeCount)

        for (i in 0 until edgeCount) {
            val edgeOffset = offsetEdges + i * EDGE_RECORD_SIZE
            byteBuffer.position(edgeOffset)

            val edgeId = byteBuffer.int
            val osmWayId = byteBuffer.long
            byteBuffer.int // startNodeId
            byteBuffer.int // endNodeId
            val geomStartIdx = byteBuffer.int
            val geomCount = byteBuffer.short.toInt() and 0xFFFF
            val roadClassId = byteBuffer.get().toInt() and 0xFF
            val oneway = byteBuffer.get().toInt()
            val layer = byteBuffer.get().toInt()
            val edgeFlags = byteBuffer.get().toInt() and 0xFF
            val maxspeed = byteBuffer.get().toInt() and 0xFF
            byteBuffer.get() // reserved
            val nameOffset = byteBuffer.int
            val lengthM = byteBuffer.float

            val roadClassName = if (roadClassId in ROAD_CLASSES.indices) ROAD_CLASSES[roadClassId] else "road"
            val roadName = getString(nameOffset).ifEmpty { "$roadClassName (ID $edgeId)" }
            val isBridge = (edgeFlags and 0x01) != 0 || layer > 0
            val isTunnel = (edgeFlags and 0x02) != 0 || layer < 0
            val isOneway = (edgeFlags and 0x04) != 0 || oneway != 0

            // Read Polyline Points
            val pointsWgs84 = ArrayList<Pair<Double, Double>>(geomCount)
            for (g in 0 until geomCount) {
                val geomOffset = offsetGeometry + (geomStartIdx + g) * GEOM_RECORD_SIZE
                if (geomOffset + GEOM_RECORD_SIZE <= buffer.size) {
                    byteBuffer.position(geomOffset)
                    val lat = byteBuffer.double
                    val lon = byteBuffer.double
                    pointsWgs84.add(Pair(lat, lon))
                }
            }

            edges.add(
                RoadEdgeGeometry(
                    edgeId = edgeId,
                    osmWayId = osmWayId,
                    name = roadName,
                    roadClass = roadClassName,
                    layer = layer,
                    isBridge = isBridge,
                    isTunnel = isTunnel,
                    isOneway = isOneway,
                    maxspeedKmh = maxspeed,
                    lengthM = lengthM,
                    pointsWgs84 = pointsWgs84
                )
            )
        }

        // Attribution JSON
        val attribution = if (attributionBytes > 0 && offsetAttribution + attributionBytes <= buffer.size) {
            String(buffer, offsetAttribution, attributionBytes, Charsets.UTF_8)
        } else "OpenStreetMap contributors / ISRO IDR"

        return RoadMapGraph(
            mapName = mapName,
            minLat = minLat,
            minLon = minLon,
            maxLat = maxLat,
            maxLon = maxLon,
            nodeCount = nodeCount,
            edgeCount = edgeCount,
            edges = edges,
            attribution = attribution,
            basemap = basemap
        )
    }

    fun readFromStream(stream: InputStream, mapName: String = "Offline Road Map", basemap: GeographicBasemap? = null): RoadMapGraph? {
        return stream.use {
            val bytes = it.readBytes()
            readFromBytes(bytes, mapName, basemap)
        }
    }

    fun readFromFile(file: File, basemap: GeographicBasemap? = null): RoadMapGraph? {
        if (!file.exists() || !file.canRead()) return null
        return file.inputStream().use { readFromStream(it, file.nameWithoutExtension, basemap) }
    }

    /**
     * Reads a companion geographic vector basemap JSON definition (rivers, landuse, landmarks).
     */
    fun readBasemapFromJsonStream(stream: InputStream): GeographicBasemap? {
        return try {
            val jsonStr = stream.bufferedReader().use { it.readText() }
            val root = JSONObject(jsonStr)
            val name = root.optString("name", "Offline Basemap")
            val boundsArr = root.optJSONArray("bounds")
            val minLat = boundsArr?.optDouble(0, 0.0) ?: 0.0
            val minLon = boundsArr?.optDouble(1, 0.0) ?: 0.0
            val maxLat = boundsArr?.optDouble(2, 0.0) ?: 0.0
            val maxLon = boundsArr?.optDouble(3, 0.0) ?: 0.0

            val waterList = mutableListOf<WaterBodyFeature>()
            val waterArr = root.optJSONArray("water_bodies") ?: root.optJSONArray("rivers")
            if (waterArr != null) {
                for (i in 0 until waterArr.length()) {
                    val wObj = waterArr.getJSONObject(i)
                    val wName = wObj.optString("name", "Water Body")
                    val wType = wObj.optString("type", "river")
                    val widthM = wObj.optDouble("width_meters", 150.0).toFloat()
                    val coordsArr = wObj.optJSONArray("coordinates")
                    val coords = mutableListOf<Pair<Double, Double>>()
                    if (coordsArr != null) {
                        for (c in 0 until coordsArr.length()) {
                            val pt = coordsArr.getJSONArray(c)
                            coords.add(Pair(pt.getDouble(0), pt.getDouble(1)))
                        }
                    }
                    waterList.add(WaterBodyFeature(wName, wType, coords, emptyList(), widthM))
                }
            }

            val landList = mutableListOf<LanduseFeature>()
            val landArr = root.optJSONArray("landuse_polygons") ?: root.optJSONArray("landuse") ?: root.optJSONArray("parks")
            if (landArr != null) {
                for (i in 0 until landArr.length()) {
                    val lObj = landArr.getJSONObject(i)
                    val lName = lObj.optString("name", "Landuse Area")
                    val lType = lObj.optString("type", "park")
                    val coordsArr = lObj.optJSONArray("coordinates")
                    val coords = mutableListOf<Pair<Double, Double>>()
                    if (coordsArr != null) {
                        for (c in 0 until coordsArr.length()) {
                            val pt = coordsArr.getJSONArray(c)
                            coords.add(Pair(pt.getDouble(0), pt.getDouble(1)))
                        }
                    }
                    landList.add(LanduseFeature(lName, lType, coords, emptyList()))
                }
            }

            val landmarkList = mutableListOf<LandmarkFeature>()
            val marksArr = root.optJSONArray("landmarks")
            if (marksArr != null) {
                for (i in 0 until marksArr.length()) {
                    val mObj = marksArr.getJSONObject(i)
                    val mName = mObj.optString("name", "Landmark")
                    val lat = mObj.optDouble("lat", 0.0)
                    val lon = mObj.optDouble("lon", 0.0)
                    val cat = mObj.optString("category", "district")
                    landmarkList.add(LandmarkFeature(mName, lat, lon, cat))
                }
            }

            val buildingList = mutableListOf<BuildingFeature>()
            val buildingsArr = root.optJSONArray("buildings_3d") ?: root.optJSONArray("buildings")
            if (buildingsArr != null) {
                for (i in 0 until buildingsArr.length()) {
                    val bObj = buildingsArr.getJSONObject(i)
                    val bId = bObj.optString("id", "b$i")
                    val bName = bObj.optString("name", "Building")
                    val heightM = bObj.optDouble("height_m", 15.0).toFloat()
                    val minHeightM = bObj.optDouble("min_height_m", 0.0).toFloat()
                    val levels = bObj.optInt("levels", 3)
                    val bType = bObj.optString("type", "civic")
                    val coordsArr = bObj.optJSONArray("coordinates")
                    val coords = mutableListOf<Pair<Double, Double>>()
                    if (coordsArr != null) {
                        for (c in 0 until coordsArr.length()) {
                            val pt = coordsArr.getJSONArray(c)
                            coords.add(Pair(pt.getDouble(0), pt.getDouble(1)))
                        }
                    }
                    buildingList.add(
                        BuildingFeature(
                            id = bId,
                            name = bName,
                            heightM = heightM,
                            minHeightM = minHeightM,
                            levels = levels,
                            type = bType,
                            coordinatesWgs84 = coords
                        )
                    )
                }
            }

            GeographicBasemap(
                name = name,
                minLat = minLat,
                minLon = minLon,
                maxLat = maxLat,
                maxLon = maxLon,
                waterBodies = waterList,
                landusePolygons = landList,
                landmarks = landmarkList,
                buildings = buildingList
            )
        } catch (e: Exception) {
            null
        }
    }
}

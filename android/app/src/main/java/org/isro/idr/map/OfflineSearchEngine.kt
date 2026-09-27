package org.isro.idr.map

import org.isro.idr.ingestion.GeoCoordinates
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class SearchPoiResult(
    val title: String,
    val subtitle: String,
    val lat: Double,
    val lon: Double,
    val category: String = "Place",
    val distanceMeters: Double = 0.0
)

/**
 * High-Performance Offline Geographic Search Engine.
 * Operates purely on locally cached/loaded OpenStreetMap vector road graphs,
 * landmarks, districts, and POI feature layers with zero network requirement.
 */
object OfflineSearchEngine {

    /**
     * Searches places, roads, landmarks, and POIs in the active offline map graph.
     */
    fun search(
        query: String,
        userLat: Double,
        userLon: Double,
        graph: RoadMapGraph?
    ): List<SearchPoiResult> {
        val results = mutableListOf<SearchPoiResult>()
        val seen = mutableSetOf<String>()

        // 1. Search in Basemap Landmarks & POIs
        graph?.basemap?.landmarks?.forEach { lm ->
            val dist = haversineDistanceMeters(userLat, userLon, lm.lat, lm.lon)
            if (query.isBlank() || lm.name.contains(query, ignoreCase = true) || lm.category.contains(query, ignoreCase = true)) {
                val key = "${lm.name}_${lm.lat}_${lm.lon}"
                if (seen.add(key)) {
                    results.add(
                        SearchPoiResult(
                            title = lm.name,
                            subtitle = "${lm.category.replaceFirstChar { it.uppercase() }} · ${formatDistance(dist)}",
                            lat = lm.lat,
                            lon = lm.lon,
                            category = lm.category,
                            distanceMeters = dist
                        )
                    )
                }
            }
        }

        // 2. Search in Basemap 3D Buildings / Civic Centers
        graph?.basemap?.buildings?.forEach { bld ->
            if (bld.name.isNotBlank() && bld.coordinatesWgs84.isNotEmpty()) {
                val centerLat = bld.coordinatesWgs84.map { it.first }.average()
                val centerLon = bld.coordinatesWgs84.map { it.second }.average()
                val dist = haversineDistanceMeters(userLat, userLon, centerLat, centerLon)
                if (query.isBlank() || bld.name.contains(query, ignoreCase = true) || bld.type.contains(query, ignoreCase = true)) {
                    val key = "${bld.name}_$centerLat"
                    if (seen.add(key)) {
                        results.add(
                            SearchPoiResult(
                                title = bld.name,
                                subtitle = "Civic Building · ${formatDistance(dist)}",
                                lat = centerLat,
                                lon = centerLon,
                                category = "building",
                                distanceMeters = dist
                            )
                        )
                    }
                }
            }
        }

        // 3. Search in Road Network Edges
        graph?.edges?.forEach { edge ->
            if (edge.name.isNotBlank() && edge.pointsWgs84.isNotEmpty()) {
                if (query.isNotBlank() && edge.name.contains(query, ignoreCase = true)) {
                    val midPoint = edge.pointsWgs84[edge.pointsWgs84.size / 2]
                    val dist = haversineDistanceMeters(userLat, userLon, midPoint.first, midPoint.second)
                    val key = "road_${edge.name}"
                    if (seen.add(key)) {
                        results.add(
                            SearchPoiResult(
                                title = edge.name,
                                subtitle = "${edge.roadClass.replaceFirstChar { it.uppercase() }} Road · ${formatDistance(dist)}",
                                lat = midPoint.first,
                                lon = midPoint.second,
                                category = "road",
                                distanceMeters = dist
                            )
                        )
                    }
                }
            }
        }

        // 4. Bundled Offline Regional Points for Prayagraj & Uttar Pradesh Corridor
        val regionalPois = getBundledRegionalPois()
        regionalPois.forEach { poi ->
            val dist = haversineDistanceMeters(userLat, userLon, poi.lat, poi.lon)
            if (query.isBlank() || poi.title.contains(query, ignoreCase = true) || poi.subtitle.contains(query, ignoreCase = true) || poi.category.contains(query, ignoreCase = true)) {
                val key = "${poi.title}_${poi.lat}_${poi.lon}"
                if (seen.add(key)) {
                    results.add(
                        poi.copy(
                            subtitle = "${poi.category.replaceFirstChar { it.uppercase() }} · ${formatDistance(dist)}",
                            distanceMeters = dist
                        )
                    )
                }
            }
        }

        // Sort by distance from current user position
        return results.sortedBy { it.distanceMeters }
    }

    private fun getBundledRegionalPois(): List<SearchPoiResult> = listOf(
        SearchPoiResult("Sangam Confluence (Triveni Sangam)", "Prayagraj, Uttar Pradesh", 25.4290, 81.8840, "Landmark"),
        SearchPoiResult("Civil Lines Commercial Centre", "Mahatma Gandhi Marg, Prayagraj", 25.4520, 81.8340, "Commercial"),
        SearchPoiResult("New Yamuna Bridge (Naini Bridge)", "NH-27, Prayagraj", 25.4260, 81.8540, "Bridge"),
        SearchPoiResult("Prayagraj Junction Railway Station", "Leader Road, Civil Lines", 25.4450, 81.8260, "Transit"),
        SearchPoiResult("Prayagraj Airport (IXD)", "Bamrauli, Prayagraj", 25.4402, 81.7342, "Airport"),
        SearchPoiResult("Motilal Nehru National Institute of Technology (MNNIT)", "Teliarganj, Prayagraj", 25.4925, 81.8638, "Education"),
        SearchPoiResult("University of Allahabad", "Senate House, Prayagraj", 25.4587, 81.8585, "Education"),
        SearchPoiResult("Allahabad High Court", "Nyaya Marg, Cantonment", 25.4480, 81.8280, "Civic"),
        SearchPoiResult("All Saints Cathedral", "Sarojini Naidu Marg, Civil Lines", 25.4533, 81.8354, "Monument"),
        SearchPoiResult("Anand Bhavan", "Tagore Town, Prayagraj", 25.4608, 81.8612, "Museum"),
        SearchPoiResult("Chandra Shekhar Azad Park (Company Bagh)", "Civil Lines, Prayagraj", 25.4560, 81.8520, "Park"),
        SearchPoiResult("Swaroop Rani Nehru Hospital", "Medical College, Prayagraj", 25.4420, 81.8480, "Hospital"),
        SearchPoiResult("Subedarganj Railway Station", "Subedarganj, Prayagraj", 25.4440, 81.7980, "Transit"),
        SearchPoiResult("Shastri Bridge (Ganga River Crossing)", "GT Road, Jhunsi", 25.4410, 81.8920, "Bridge"),
        SearchPoiResult("ISRO Headquarters (Antariksh Bhavan)", "New BEL Road, Bengaluru", 13.0334, 77.5645, "Aerospace"),
        SearchPoiResult("Space Applications Centre (SAC / ISRO)", "Jodhpur Tekra, Ahmedabad", 23.0225, 72.5167, "Aerospace")
    )

    private fun haversineDistanceMeters(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val r = 6371000.0 // Earth radius in meters
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return r * c
    }

    private fun formatDistance(distM: Double): String {
        return if (distM >= 1000.0) {
            String.format("%.1f km", distM / 1000.0)
        } else {
            String.format("%.0f m", distM)
        }
    }
}

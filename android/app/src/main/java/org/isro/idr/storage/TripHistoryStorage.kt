package org.isro.idr.storage

import android.content.Context
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

data class TripHistoryItem(
    val id: String,
    val timestampMs: Long,
    val dateString: String,
    val startLocation: String,
    val destination: String,
    val distanceMeters: Double,
    val durationSeconds: Double,
    val avgSpeedKmh: Double,
    val maxSpeedKmh: Double,
    val drDurationSeconds: Double,
    val driftPercentage: Double,
    val passStatus: String = "PASS (<10%)"
)

/**
 * Local persistent storage for completed Dead Reckoning & Navigation trips.
 * Ensures trip logs survive application restarts and offline testing sessions.
 */
object TripHistoryStorage {

    private const val FILE_NAME = "idr_trip_history.json"
    private val cachedTrips = mutableListOf<TripHistoryItem>()
    private var isInitialized = false

    fun init(context: Context) {
        if (isInitialized) return
        val file = File(context.filesDir, FILE_NAME)
        if (file.exists()) {
            try {
                val jsonStr = file.readText()
                val array = JSONArray(jsonStr)
                cachedTrips.clear()
                for (i in 0 until array.length()) {
                    val obj = array.getJSONObject(i)
                    cachedTrips.add(
                        TripHistoryItem(
                            id = obj.optString("id", "trip_$i"),
                            timestampMs = obj.optLong("timestampMs", System.currentTimeMillis()),
                            dateString = obj.optString("dateString", "Recent Trip"),
                            startLocation = obj.optString("startLocation", "Current Location"),
                            destination = obj.optString("destination", "Destination"),
                            distanceMeters = obj.optDouble("distanceMeters", 0.0),
                            durationSeconds = obj.optDouble("durationSeconds", 0.0),
                            avgSpeedKmh = obj.optDouble("avgSpeedKmh", 0.0),
                            maxSpeedKmh = obj.optDouble("maxSpeedKmh", 0.0),
                            drDurationSeconds = obj.optDouble("drDurationSeconds", 0.0),
                            driftPercentage = obj.optDouble("driftPercentage", 0.0),
                            passStatus = obj.optString("passStatus", "PASS (<10%)")
                        )
                    )
                }
            } catch (_: Exception) {}
        }
        isInitialized = true
    }

    fun getTrips(context: Context): List<TripHistoryItem> {
        init(context)
        return cachedTrips.toList()
    }

    fun saveTrip(context: Context, trip: TripHistoryItem) {
        init(context)
        cachedTrips.add(0, trip)
        persistToFile(context)
    }

    fun clearTrips(context: Context) {
        init(context)
        cachedTrips.clear()
        persistToFile(context)
    }

    private fun persistToFile(context: Context) {
        try {
            val array = JSONArray()
            for (trip in cachedTrips) {
                val obj = JSONObject().apply {
                    put("id", trip.id)
                    put("timestampMs", trip.timestampMs)
                    put("dateString", trip.dateString)
                    put("startLocation", trip.startLocation)
                    put("destination", trip.destination)
                    put("distanceMeters", trip.distanceMeters)
                    put("durationSeconds", trip.durationSeconds)
                    put("avgSpeedKmh", trip.avgSpeedKmh)
                    put("maxSpeedKmh", trip.maxSpeedKmh)
                    put("drDurationSeconds", trip.drDurationSeconds)
                    put("driftPercentage", trip.driftPercentage)
                    put("passStatus", trip.passStatus)
                }
                array.put(obj)
            }
            val file = File(context.filesDir, FILE_NAME)
            file.writeText(array.toString(2))
        } catch (_: Exception) {}
    }
}

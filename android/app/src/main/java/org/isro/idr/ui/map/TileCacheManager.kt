package org.isro.idr.ui.map

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.LruCache
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.isro.idr.ui.state.MapStyleType
import java.io.File
import java.io.FileOutputStream
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.ConcurrentHashMap

/**
 * High-Performance Offline/Online Tile Cache Manager.
 * Fetches and caches worldwide OpenStreetMap / CartoDB geographic tiles.
 * Provides instant in-memory LRU and persistent disk cache with zero UI stutter.
 */
object TileCacheManager {

    private val memoryCache: LruCache<String, Bitmap> = object : LruCache<String, Bitmap>(64 * 1024 * 1024) { // 64 MB
        override fun sizeOf(key: String, value: Bitmap): Int {
            return value.byteCount
        }
    }

    private val inFlightRequests = ConcurrentHashMap<String, Job>()
    private val scope = CoroutineScope(Dispatchers.IO)
    private var cacheDir: File? = null

    fun init(context: Context) {
        if (cacheDir == null) {
            val dir = File(context.cacheDir, "idr_geo_tiles")
            if (!dir.exists()) dir.mkdirs()
            cacheDir = dir
        }
    }

    fun getTile(
        style: MapStyleType,
        zoom: Int,
        tileX: Int,
        tileY: Int,
        onTileLoaded: () -> Unit
    ): Bitmap? {
        val maxTile = (1 shl zoom) - 1
        if (tileX < 0 || tileX > maxTile || tileY < 0 || tileY > maxTile || zoom < 0 || zoom > 19) {
            return null
        }

        val key = "${style.name}_${zoom}_${tileX}_${tileY}"

        // 1. Check in-memory LRU cache
        memoryCache.get(key)?.let { return it }

        // 2. Check disk cache
        val localDir = cacheDir
        if (localDir != null) {
            val diskFile = File(localDir, "$key.png")
            if (diskFile.exists() && diskFile.length() > 0) {
                try {
                    val bitmap = BitmapFactory.decodeFile(diskFile.absolutePath)
                    if (bitmap != null) {
                        memoryCache.put(key, bitmap)
                        return bitmap
                    }
                } catch (_: Exception) {}
            }
        }

        // 3. Asynchronously fetch from tile server if not already requested
        if (!inFlightRequests.containsKey(key)) {
            val job = scope.launch {
                val downloaded = fetchTileFromNetwork(style, zoom, tileX, tileY, key)
                if (downloaded != null) {
                    memoryCache.put(key, downloaded)
                    withContext(Dispatchers.Main) {
                        onTileLoaded()
                    }
                }
                inFlightRequests.remove(key)
            }
            inFlightRequests[key] = job
        }

        return null
    }

    private fun fetchTileFromNetwork(
        style: MapStyleType,
        zoom: Int,
        tileX: Int,
        tileY: Int,
        key: String
    ): Bitmap? {
        val urlStr = when (style) {
            MapStyleType.DARK -> "https://a.basemaps.cartocdn.com/dark_all/$zoom/$tileX/$tileY.png"
            MapStyleType.STANDARD -> "https://tile.openstreetmap.org/$zoom/$tileX/$tileY.png"
            MapStyleType.SATELLITE -> "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/$zoom/$tileY/$tileX"
            MapStyleType.TERRAIN -> "https://a.basemaps.cartocdn.com/rastertiles/voyager/$zoom/$tileX/$tileY.png"
        }

        return try {
            val url = URL(urlStr)
            val conn = url.openConnection() as HttpURLConnection
            conn.connectTimeout = 3500
            conn.readTimeout = 4000
            conn.setRequestProperty("User-Agent", "ISRO-IDR-Navigation-Engine/1.0 (PS26168; Android)")
            conn.connect()

            if (conn.responseCode == HttpURLConnection.HTTP_OK) {
                val input = conn.inputStream
                val bytes = input.readBytes()
                input.close()

                val bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
                if (bitmap != null && cacheDir != null) {
                    try {
                        val diskFile = File(cacheDir, "$key.png")
                        FileOutputStream(diskFile).use { fos ->
                            fos.write(bytes)
                        }
                    } catch (_: Exception) {}
                }
                bitmap
            } else null
        } catch (_: Exception) {
            null
        }
    }
}

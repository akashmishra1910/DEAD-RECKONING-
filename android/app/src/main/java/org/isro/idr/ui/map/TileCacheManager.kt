package org.isro.idr.ui.map

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.ColorMatrix
import android.graphics.ColorMatrixColorFilter
import android.graphics.Paint
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
 * Fetches and caches worldwide OpenStreetMap geographic tiles with 100% free access (no API keys required).
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

    fun getCacheSizeMb(): Double {
        val dir = cacheDir ?: return 0.0
        val bytes = dir.walkTopDown().filter { it.isFile }.map { it.length() }.sum()
        return bytes / (1024.0 * 1024.0)
    }

    fun clearCache() {
        memoryCache.evictAll()
        cacheDir?.listFiles()?.forEach { file ->
            try { file.delete() } catch (_: Exception) {}
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
        // Use 100% free OSM tile servers and OpenTopoMap without API key restrictions
        val mirror = listOf("a", "b", "c")[(tileX + tileY) % 3]
        val urlStr = when (style) {
            MapStyleType.STANDARD, MapStyleType.DARK -> "https://$mirror.tile.openstreetmap.org/$zoom/$tileX/$tileY.png"
            MapStyleType.SATELLITE -> "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/$zoom/$tileY/$tileX"
            MapStyleType.TERRAIN -> "https://tile.opentopomap.org/$zoom/$tileX/$tileY.png"
        }

        return try {
            val url = URL(urlStr)
            val conn = url.openConnection() as HttpURLConnection
            conn.connectTimeout = 3500
            conn.readTimeout = 4000
            conn.setRequestProperty("User-Agent", "ISRO-IDR-Navigation-Engine/1.0 (PS26168; Android; idr.isro.gov.in)")
            conn.connect()

            if (conn.responseCode == HttpURLConnection.HTTP_OK) {
                val input = conn.inputStream
                val bytes = input.readBytes()
                input.close()

                var bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
                if (bitmap != null) {
                    // If Dark mode requested, convert clean OSM tile using aerospace high-contrast dark color matrix
                    if (style == MapStyleType.DARK) {
                        bitmap = applyDarkAerospaceFilter(bitmap)
                    }

                    if (cacheDir != null) {
                        try {
                            val diskFile = File(cacheDir, "$key.png")
                            FileOutputStream(diskFile).use { fos ->
                                bitmap.compress(Bitmap.CompressFormat.PNG, 95, fos)
                            }
                        } catch (_: Exception) {}
                    }
                }
                bitmap
            } else null
        } catch (_: Exception) {
            null
        }
    }

    private fun applyDarkAerospaceFilter(src: Bitmap): Bitmap {
        val darkBitmap = Bitmap.createBitmap(src.width, src.height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(darkBitmap)
        val paint = Paint(Paint.ANTI_ALIAS_FLAG)

        // Aerospace Night Filter: Invert RGB, reduce lightness, boost cyan/blue tone
        val matrix = ColorMatrix(floatArrayOf(
            -0.85f,  0.00f,  0.00f, 0f, 210f,
             0.00f, -0.85f,  0.00f, 0f, 215f,
             0.00f,  0.00f, -0.75f, 0f, 240f,
             0.00f,  0.00f,  0.00f, 1f,   0f
        ))
        paint.colorFilter = ColorMatrixColorFilter(matrix)
        canvas.drawBitmap(src, 0f, 0f, paint)
        return darkBitmap
    }
}

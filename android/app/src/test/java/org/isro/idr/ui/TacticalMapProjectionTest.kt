package org.isro.idr.ui

import org.isro.idr.ui.state.TrajectoryPoint
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class TacticalMapProjectionTest {

    @Test
    fun testScaleCalculationBounds() {
        val points = listOf(
            TrajectoryPoint(enuX = 0.0, enuY = 0.0),
            TrajectoryPoint(enuX = 100.0, enuY = 100.0)
        )

        val canvasWidth = 800f
        val canvasHeight = 600f

        var minX = 0.0
        var maxX = 100.0
        var minY = 0.0
        var maxY = 100.0

        val spanX = (maxX - minX).coerceAtLeast(40.0)
        val spanY = (maxY - minY).coerceAtLeast(40.0)

        val scaleX = (canvasWidth * 0.7f) / spanX.toFloat()
        val scaleY = (canvasHeight * 0.7f) / spanY.toFloat()
        val scale = minOf(scaleX, scaleY).coerceIn(0.05f, 20.0f)

        assertEquals(4.2f, scale, 0.05f)
        assertTrue(scale in 0.05f..20.0f)
    }

    @Test
    fun testOriginScreenProjection() {
        val centerX = 400f
        val centerY = 300f
        val scale = 2.0f
        val vehicleX = 50.0
        val vehicleY = 50.0

        // In TacticalGridMapCanvas, originScreenX = centerX - (vehicleX * scale)
        val originX = centerX - (vehicleX * scale).toFloat()
        val originY = centerY + (vehicleY * scale).toFloat()

        assertEquals(300f, originX, 1e-4f)
        assertEquals(400f, originY, 1e-4f)
    }

    @Test
    fun testHeadingWrapping() {
        fun normalizeHeading(azimuthDeg: Double): Double {
            return ((azimuthDeg % 360.0) + 360.0) % 360.0
        }

        assertEquals(0.0, normalizeHeading(0.0), 1e-6)
        assertEquals(90.0, normalizeHeading(90.0), 1e-6)
        assertEquals(270.0, normalizeHeading(-90.0), 1e-6)
        assertEquals(45.0, normalizeHeading(405.0), 1e-6)
    }
}

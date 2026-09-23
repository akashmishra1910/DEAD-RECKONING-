package org.isro.idr.engine

import java.nio.ByteBuffer

/**
 * Thread-safe Kotlin wrapper around the native IDR C++ Navigation Engine.
 * Manages native lifecycle, zero-allocation step execution, and JNI data translation.
 */
class NativeNavigationEngine private constructor(
    @Volatile private var nativeHandle: Long
) : AutoCloseable {

    private val lock = Any()
    private val flatTelemetryBuffer = DoubleArray(NavigationTelemetry.FLAT_ARRAY_SIZE)

    val isValid: Boolean
        get() = nativeHandle != 0L

    /**
     * Executes a single navigation step.
     */
    fun processStep(
        timeSeconds: Double,
        accel: Vector3D,
        gyro: Vector3D,
        baroAlt: Double = 0.0,
        gnssValid: Boolean = false,
        gnssPos: Vector3D = Vector3D(),
        gnssVel: Vector3D = Vector3D(),
        aiSpeedMps: Double = -1.0
    ): Pair<NativeStatus, NavigationTelemetry?> = synchronized(lock) {
        if (!isValid) {
            return NativeStatus.INVALID_STATE to null
        }

        val statusCode = nativeProcessStep(
            nativeHandle,
            timeSeconds,
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            baroAlt,
            if (gnssValid) 1 else 0,
            gnssPos.x, gnssPos.y, gnssPos.z,
            gnssVel.x, gnssVel.y, gnssVel.z,
            aiSpeedMps,
            flatTelemetryBuffer
        )

        val status = NativeStatus.fromCode(statusCode)
        return if (status == NativeStatus.OK) {
            status to NavigationTelemetry.fromFlatArray(flatTelemetryBuffer)
        } else {
            status to null
        }
    }

    /**
     * Executes an extended navigation step including magnetometer and map constraint toggles.
     */
    fun processStepExt(
        timeSeconds: Double,
        accel: Vector3D,
        gyro: Vector3D,
        baroAlt: Double = 0.0,
        gnssValid: Boolean = false,
        gnssPos: Vector3D = Vector3D(),
        gnssVel: Vector3D = Vector3D(),
        aiSpeedMps: Double = -1.0,
        magValid: Boolean = false,
        mag: Vector3D = Vector3D(),
        enableRoadConstraints: Boolean = true
    ): Pair<NativeStatus, NavigationTelemetry?> = synchronized(lock) {
        if (!isValid) {
            return NativeStatus.INVALID_STATE to null
        }

        val statusCode = nativeProcessStepExt(
            nativeHandle,
            timeSeconds,
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            baroAlt,
            if (gnssValid) 1 else 0,
            gnssPos.x, gnssPos.y, gnssPos.z,
            gnssVel.x, gnssVel.y, gnssVel.z,
            aiSpeedMps,
            if (magValid) 1 else 0,
            mag.x, mag.y, mag.z,
            if (enableRoadConstraints) 1 else 0,
            flatTelemetryBuffer
        )

        val status = NativeStatus.fromCode(statusCode)
        return if (status == NativeStatus.OK) {
            status to NavigationTelemetry.fromFlatArray(flatTelemetryBuffer)
        } else {
            status to null
        }
    }

    /**
     * Zero-allocation direct ByteBuffer step execution.
     */
    fun processStepDirect(
        timeSeconds: Double,
        accel: Vector3D,
        gyro: Vector3D,
        baroAlt: Double,
        gnssValid: Boolean,
        gnssPos: Vector3D,
        gnssVel: Vector3D,
        aiSpeedMps: Double,
        magValid: Boolean,
        mag: Vector3D,
        enableRoadConstraints: Boolean,
        directBuffer: ByteBuffer
    ): NativeStatus = synchronized(lock) {
        if (!isValid) return NativeStatus.INVALID_STATE

        val statusCode = nativeProcessStepDirectBuffer(
            nativeHandle,
            timeSeconds,
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            baroAlt,
            if (gnssValid) 1 else 0,
            gnssPos.x, gnssPos.y, gnssPos.z,
            gnssVel.x, gnssVel.y, gnssVel.z,
            aiSpeedMps,
            if (magValid) 1 else 0,
            mag.x, mag.y, mag.z,
            if (enableRoadConstraints) 1 else 0,
            directBuffer
        )
        return NativeStatus.fromCode(statusCode)
    }

    /**
     * OSM Road Graph topological management.
     */
    fun clearOsmGraph() = synchronized(lock) {
        if (isValid) nativeClearOsmGraph(nativeHandle)
    }

    fun addOsmNode(nodeId: Int, lat: Double, lon: Double, alt: Double, enuX: Double, enuY: Double, enuZ: Double) = synchronized(lock) {
        if (isValid) nativeAddOsmNode(nativeHandle, nodeId, lat, lon, alt, enuX, enuY, enuZ)
    }

    fun addOsmEdge(
        edgeId: Int,
        sourceId: Int,
        targetId: Int,
        name: String,
        roadType: String,
        layer: Int,
        width: Double,
        speedLimit: Double,
        numPts: Int,
        polylineXyz: DoubleArray
    ) = synchronized(lock) {
        if (isValid) {
            nativeAddOsmEdge(
                nativeHandle, edgeId, sourceId, targetId,
                name, roadType, layer, width, speedLimit, numPts, polylineXyz
            )
        }
    }

    override fun close() {
        synchronized(lock) {
            if (nativeHandle != 0L) {
                nativeDestroy(nativeHandle)
                nativeHandle = 0L
            }
        }
    }

    companion object {
        init {
            try {
                System.loadLibrary("idr_jni")
            } catch (e: UnsatisfiedLinkError) {
                System.err.println("Failed to load libidr_jni.so: ${e.message}")
            }
        }

        fun apiVersion(): Int = nativeApiVersion()
        fun apiVersionString(): String = nativeApiVersionString()
        fun statusToString(status: NativeStatus): String = nativeStatusToString(status.code)
        fun navModeToString(mode: NavigationMode): String = nativeNavModeToString(mode.code)

        /**
         * Factory: Standard engine instance.
         */
        fun create(dt: Double = 0.1): NativeNavigationEngine? {
            val handle = nativeCreate(dt)
            return if (handle != 0L) NativeNavigationEngine(handle) else null
        }

        /**
         * Factory: Engine with explicit initial states.
         */
        fun createWithState(
            dt: Double,
            initPos: Vector3D,
            initVel: Vector3D,
            initEuler: Vector3D
        ): NativeNavigationEngine? {
            val handle = nativeCreateWithState(
                dt,
                initPos.x, initPos.y, initPos.z,
                initVel.x, initVel.y, initVel.z,
                initEuler.x, initEuler.y, initEuler.z
            )
            return if (handle != 0L) NativeNavigationEngine(handle) else null
        }

        /**
         * Factory: Fully configured engine instance.
         */
        fun createFull(
            dt: Double,
            initPos: Vector3D,
            initVel: Vector3D,
            initEuler: Vector3D,
            enableHeadingSupervisor: Boolean = true,
            enableMapMatching: Boolean = true,
            enableSpeedCalibration: Boolean = true,
            enableLongitudinalMapConstraint: Boolean = true
        ): NativeNavigationEngine? {
            val handle = nativeCreateFull(
                dt,
                initPos.x, initPos.y, initPos.z,
                initVel.x, initVel.y, initVel.z,
                initEuler.x, initEuler.y, initEuler.z,
                if (enableHeadingSupervisor) 1 else 0,
                if (enableMapMatching) 1 else 0,
                if (enableSpeedCalibration) 1 else 0,
                if (enableLongitudinalMapConstraint) 1 else 0
            )
            return if (handle != 0L) NativeNavigationEngine(handle) else null
        }

        // ====================================================================
        // Private JNI Native Methods
        // ====================================================================

        @JvmStatic private external fun nativeApiVersion(): Int
        @JvmStatic private external fun nativeApiVersionString(): String
        @JvmStatic private external fun nativeStatusToString(statusCode: Int): String
        @JvmStatic private external fun nativeNavModeToString(modeCode: Int): String

        @JvmStatic private external fun nativeCreate(dt: Double): Long
        @JvmStatic private external fun nativeCreateWithState(
            dt: Double,
            posX: Double, posY: Double, posZ: Double,
            velX: Double, velY: Double, velZ: Double,
            eulerRoll: Double, eulerPitch: Double, eulerYaw: Double
        ): Long
        @JvmStatic private external fun nativeCreateFull(
            dt: Double,
            posX: Double, posY: Double, posZ: Double,
            velX: Double, velY: Double, velZ: Double,
            eulerRoll: Double, eulerPitch: Double, eulerYaw: Double,
            enableHeadingSupervisor: Int,
            enableMapMatching: Int,
            enableSpeedCalibration: Int,
            enableLongitudinalMapConstraint: Int
        ): Long

        @JvmStatic private external fun nativeDestroy(handle: Long)

        @JvmStatic private external fun nativeProcessStep(
            handle: Long,
            timeS: Double,
            ax: Double, ay: Double, az: Double,
            gx: Double, gy: Double, gz: Double,
            baroAlt: Double,
            gnssValid: Int,
            gnssPosX: Double, gnssPosY: Double, gnssPosZ: Double,
            gnssVelX: Double, gnssVelY: Double, gnssVelZ: Double,
            aiSpeed: Double,
            outTelemBuffer: DoubleArray
        ): Int

        @JvmStatic private external fun nativeProcessStepExt(
            handle: Long,
            timeS: Double,
            ax: Double, ay: Double, az: Double,
            gx: Double, gy: Double, gz: Double,
            baroAlt: Double,
            gnssValid: Int,
            gnssPosX: Double, gnssPosY: Double, gnssPosZ: Double,
            gnssVelX: Double, gnssVelY: Double, gnssVelZ: Double,
            aiSpeed: Double,
            magValid: Int,
            magX: Double, magY: Double, magZ: Double,
            enableRoadConstraints: Int,
            outTelemBuffer: DoubleArray
        ): Int

        @JvmStatic private external fun nativeProcessStepDirectBuffer(
            handle: Long,
            timeS: Double,
            ax: Double, ay: Double, az: Double,
            gx: Double, gy: Double, gz: Double,
            baroAlt: Double,
            gnssValid: Int,
            gnssPosX: Double, gnssPosY: Double, gnssPosZ: Double,
            gnssVelX: Double, gnssVelY: Double, gnssVelZ: Double,
            aiSpeed: Double,
            magValid: Int,
            magX: Double, magY: Double, magZ: Double,
            enableRoadConstraints: Int,
            directByteBuffer: ByteBuffer
        ): Int

    fun loadIdrmap(path: String, lat0: Double, lon0: Double, alt0: Double): Boolean = synchronized(lock) {
        if (!isValid) return false
        val rc = nativeLoadIdrmap(nativeHandle, path, lat0, lon0, alt0)
        return rc == 0
    }

    fun reset(): Boolean = synchronized(lock) {
        if (!isValid) return false
        val rc = nativeReset(nativeHandle)
        return rc == 0
    }

    companion object {
        init {
            try {
                System.loadLibrary("idr-jni")
            } catch (e: UnsatisfiedLinkError) {
                System.err.println("Failed to load native library idr-jni: ${e.message}")
            }
        }

        @JvmStatic private external fun nativeApiVersion(): Int
        @JvmStatic private external fun nativeApiVersionString(): String
        @JvmStatic private external fun nativeStatusToString(statusCode: Int): String
        @JvmStatic private external fun nativeNavModeToString(modeCode: Int): String

        @JvmStatic private external fun nativeCreate(dt: Double): Long
        @JvmStatic private external fun nativeCreateWithState(
            dt: Double,
            posX: Double, posY: Double, posZ: Double,
            velX: Double, velY: Double, velZ: Double,
            eulerRoll: Double, eulerPitch: Double, eulerYaw: Double
        ): Long
        @JvmStatic private external fun nativeCreateFull(
            dt: Double,
            posX: Double, posY: Double, posZ: Double,
            velX: Double, velY: Double, velZ: Double,
            eulerRoll: Double, eulerPitch: Double, eulerYaw: Double,
            enableHeading: Int,
            enableMap: Int,
            enableSpeedCalib: Int,
            enableLongConstraint: Int
        ): Long

        @JvmStatic private external fun nativeDestroy(handle: Long)

        @JvmStatic private external fun nativeProcessStep(
            handle: Long,
            timeS: Double,
            ax: Double, ay: Double, az: Double,
            gx: Double, gy: Double, gz: Double,
            baroAlt: Double,
            gnssValid: Int,
            gnssPosX: Double, gnssPosY: Double, gnssPosZ: Double,
            gnssVelX: Double, gnssVelY: Double, gnssVelZ: Double,
            aiSpeed: Double,
            outTelemBuffer: DoubleArray
        ): Int

        @JvmStatic private external fun nativeProcessStepExt(
            handle: Long,
            timeS: Double,
            ax: Double, ay: Double, az: Double,
            gx: Double, gy: Double, gz: Double,
            baroAlt: Double,
            gnssValid: Int,
            gnssPosX: Double, gnssPosY: Double, gnssPosZ: Double,
            gnssVelX: Double, gnssVelY: Double, gnssVelZ: Double,
            aiSpeed: Double,
            magValid: Int,
            magX: Double, magY: Double, magZ: Double,
            enableRoadConstraints: Int,
            outTelemBuffer: DoubleArray
        ): Int

        @JvmStatic private external fun nativeProcessStepDirectBuffer(
            handle: Long,
            timeS: Double,
            ax: Double, ay: Double, az: Double,
            gx: Double, gy: Double, gz: Double,
            baroAlt: Double,
            gnssValid: Int,
            gnssPosX: Double, gnssPosY: Double, gnssPosZ: Double,
            gnssVelX: Double, gnssVelY: Double, gnssVelZ: Double,
            aiSpeed: Double,
            magValid: Int,
            magX: Double, magY: Double, magZ: Double,
            enableRoadConstraints: Int,
            directByteBuffer: ByteBuffer
        ): Int

        @JvmStatic private external fun nativeClearOsmGraph(handle: Long)
        @JvmStatic private external fun nativeAddOsmNode(
            handle: Long,
            nodeId: Int,
            lat: Double, lon: Double, alt: Double,
            enuX: Double, enuY: Double, enuZ: Double
        )
        @JvmStatic private external fun nativeAddOsmEdge(
            handle: Long,
            edgeId: Int,
            sourceId: Int,
            targetId: Int,
            name: String?,
            roadType: String?,
            layer: Int,
            width: Double,
            speedLimit: Double,
            numPts: Int,
            polylineXyz: DoubleArray
        )

        @JvmStatic private external fun nativeLoadIdrmap(
            handle: Long,
            path: String,
            lat0: Double,
            lon0: Double,
            alt0: Double
        ): Int

        @JvmStatic private external fun nativeReset(handle: Long): Int
    }
}


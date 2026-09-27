package org.isro.idr.ingestion

import android.os.SystemClock
import org.isro.idr.engine.*
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * Causal Sensor and GNSS Navigation Pipeline Synchronizer.
 * Ingests rate-controlled IMU samples (~100 Hz), coordinates with latest GNSS fix,
 * executes native ESKF steps, and publishes real-time navigation telemetry and diagnostics.
 */
class NavigationPipelineSynchronizer(
    val engine: NativeNavigationEngine,
    val sensorManager: SensorIngestionManager,
    val gnssManager: GnssIngestionManager,
    val maxGnssAgeNanos: Long = 1_500_000_000L // 1.5 seconds maximum GNSS validity window
) {
    private val isRunning = AtomicBoolean(false)
    private val totalSteps = AtomicLong(0)
    private val lastStepDurationUs = AtomicLong(0)

    @Volatile private var latestTelemetry: NavigationTelemetry? = null
    @Volatile private var latestMagSample: MagnetometerSample? = null

    var onTelemetryListener: ((NavigationTelemetry) -> Unit)? = null
    var onImuSampleListener: ((ImuSample) -> Unit)? = null
    var onMagSampleListener: ((MagnetometerSample) -> Unit)? = null

    init {
        // Hook direct listeners
        sensorManager.onImuSampleListener = { imuSample ->
            if (isRunning.get()) {
                // Dequeue from imuQueue so the bounded queue stays clean (0..1 occupancy)
                sensorManager.imuQueue.poll()
                processSingleImuSample(imuSample)
                onImuSampleListener?.invoke(imuSample)
            }
        }
        sensorManager.onMagSampleListener = { magSample ->
            latestMagSample = magSample
            onMagSampleListener?.invoke(magSample)
        }
    }

    fun start(): Boolean {
        if (isRunning.getAndSet(true)) return true

        totalSteps.set(0)
        lastStepDurationUs.set(0)
        latestTelemetry = null

        val sensorStarted = sensorManager.start()
        val gnssStarted = gnssManager.start()

        return sensorStarted && gnssStarted
    }

    fun stop() {
        if (!isRunning.getAndSet(false)) return

        sensorManager.stop()
        gnssManager.stop()
    }

    /**
     * Causal step execution for a single IMU sample.
     */
    fun processSingleImuSample(imu: ImuSample): Pair<NativeStatus, NavigationTelemetry?> {
        val t0 = System.nanoTime()

        // 1. Causal GNSS availability check
        val gnss = gnssManager.latestGnssSample
        val nowElapsedNanos = SystemClock.elapsedRealtimeNanos()
        val isGnssFresh = gnss != null && gnss.isValid &&
                (nowElapsedNanos - gnss.timestampNanos in -500_000_000L..maxGnssAgeNanos)

        val gnssPos = if (isGnssFresh && gnss != null) gnss.posEnu else Vector3D()
        val gnssVel = if (isGnssFresh && gnss != null) gnss.velEnu else Vector3D()

        // 2. Magnetometer availability check
        val mag = latestMagSample
        val isMagValid = mag?.isValid ?: false
        val magVec = mag?.magUteslas ?: Vector3D()

        // 3. Native Step Execution
        val (status, telemetry) = engine.processStepExt(
            timeSeconds = imu.timeSeconds,
            accel = imu.accel,
            gyro = imu.gyro,
            baroAlt = imu.baroAltMeters,
            gnssValid = isGnssFresh,
            gnssPos = gnssPos,
            gnssVel = gnssVel,
            aiSpeedMps = -1.0, // AI speed estimated from IMU features in C++ engine
            magValid = isMagValid,
            mag = magVec,
            enableRoadConstraints = true
        )

        val durationUs = (System.nanoTime() - t0) / 1000
        lastStepDurationUs.set(durationUs)

        if (status == NativeStatus.OK && telemetry != null) {
            latestTelemetry = telemetry
            totalSteps.incrementAndGet()
            onTelemetryListener?.invoke(telemetry)
        }

        return status to telemetry
    }

    /**
     * Produces an instantaneous diagnostics snapshot of the entire ingestion pipeline.
     */
    fun getDiagnosticsSnapshot(): IngestionDiagnostics {
        val gnss = gnssManager.latestGnssSample
        val fixAgeMs = if (gnss != null && gnss.timestampNanos > 0) {
            val nowNanos = SystemClock.elapsedRealtimeNanos()
            maxOf(0L, (nowNanos - gnss.timestampNanos) / 1_000_000L)
        } else -1L

        val fixState = when {
            gnss == null || !gnss.isValid -> if (gnssManager.isRunning) "SEARCHING" else "STANDBY"
            fixAgeMs in 0..1500 -> "FIXED"
            fixAgeMs in 1501..5000 -> "STALE"
            else -> "OUTAGE"
        }

        return IngestionDiagnostics(
            isAccelerometerAvailable = sensorManager.isAccelerometerAvailable,
            isGyroscopeAvailable = sensorManager.isGyroscopeAvailable,
            isMagnetometerAvailable = sensorManager.isMagnetometerAvailable,
            isBarometerAvailable = sensorManager.isBarometerAvailable,

            accelCallbackHz = sensorManager.accelPolicy.callbackHz,
            gyroCallbackHz = sensorManager.gyroPolicy.callbackHz,
            accelCallbackCount = sensorManager.accelPolicy.callbackCount,
            gyroCallbackCount = sensorManager.gyroPolicy.callbackCount,

            accelConsumedHz = sensorManager.accelPolicy.consumedHz,
            gyroConsumedHz = sensorManager.gyroPolicy.consumedHz,
            accelConsumedCount = sensorManager.accelPolicy.consumedCount,
            gyroConsumedCount = sensorManager.gyroPolicy.consumedCount,

            accelDroppedCount = sensorManager.accelPolicy.duplicateTimestampCount + sensorManager.accelPolicy.backwardsTimestampCount,
            gyroDroppedCount = sensorManager.gyroPolicy.duplicateTimestampCount + sensorManager.gyroPolicy.backwardsTimestampCount,
            accelLargeGapCount = sensorManager.accelPolicy.largeGapCount,
            gyroLargeGapCount = sensorManager.gyroPolicy.largeGapCount,
            accelMeanDtMs = sensorManager.accelPolicy.meanDtMs,
            gyroMeanDtMs = sensorManager.gyroPolicy.meanDtMs,
            accelMinDtMs = sensorManager.accelPolicy.minDtMs,
            accelMaxDtMs = sensorManager.accelPolicy.maxDtMs,
            accelP95DtMs = sensorManager.accelPolicy.p95DtMs,

            isGnssAvailable = gnssManager.isGnssAvailable,
            gnssFixState = fixState,
            gnssFixAgeMs = fixAgeMs,
            latitudeDeg = gnss?.latitudeDeg ?: 0.0,
            longitudeDeg = gnss?.longitudeDeg ?: 0.0,
            altitudeM = gnss?.altitudeM ?: 0.0,
            horizontalAccuracyM = gnss?.horizontalAccuracyMeters ?: 0.0f,
            groundSpeedMps = gnss?.groundSpeedMps ?: 0.0,
            navicSatCount = gnssManager.navicSatCount,
            navicInViewCount = gnssManager.navicInViewCount,
            totalSatCount = gnssManager.totalSatCount,
            totalInViewCount = gnssManager.totalInViewCount,

            queueOccupancy = sensorManager.imuQueue.size,
            queueCapacity = sensorManager.imuQueue.capacity,
            queueOverflowDrops = sensorManager.imuQueue.totalDroppedOnOverflow,
            totalStepsProcessed = totalSteps.get(),
            lastStepDurationMicros = lastStepDurationUs.get(),
            pipelineRunning = isRunning.get()
        )
    }
}

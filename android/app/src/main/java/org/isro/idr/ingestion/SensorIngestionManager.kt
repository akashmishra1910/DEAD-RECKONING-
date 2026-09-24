package org.isro.idr.ingestion

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import org.isro.idr.engine.ImuSample
import org.isro.idr.engine.MagnetometerSample
import org.isro.idr.engine.Vector3D
import kotlin.math.pow

/**
 * Thread-Safe Sensor Ingestion Layer for Physical Android Devices.
 * Manages raw Android HAL sensor event callbacks (accelerometer, gyroscope,
 * magnetometer, barometer) on a dedicated high-priority background HandlerThread.
 *
 * Implements deterministic phase-accumulator causal rate resampling to downsample
 * raw hardware interrupts (~468.8 Hz on physical devices) down to exactly 100.0 Hz
 * for the dead-reckoning fusion pipeline without phase drift or queue saturation.
 */
class SensorIngestionManager(
    context: Context,
    private val targetDelayUs: Int = SensorManager.SENSOR_DELAY_FASTEST
) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val accelSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val magSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val baroSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_PRESSURE)

    private var sensorThread: HandlerThread? = null
    private var sensorHandler: Handler? = null

    val accelPolicy = TimestampPolicy(sensorName = "accel", nominalRateHz = 100.0)
    val gyroPolicy = TimestampPolicy(sensorName = "gyro", nominalRateHz = 100.0)

    val imuQueue = BoundedSensorQueue<ImuSample>(capacity = 500)

    @Volatile
    var isRunning: Boolean = false
        private set

    private var baseTimestampNanos: Long = -1L
    private var lastConsumedAccelTimestampNanos: Long = -1L
    private var lastCallbackAccelTimestampNanos: Long = -1L
    private var accelPhaseAccumulatorNanos: Long = 0L

    // Nominal 100 Hz step period: 10,000,000 ns = 10.0 ms
    private val targetStepPeriodNanos = 10_000_000L

    @Volatile
    private var latestGyro: Vector3D = Vector3D(0.0, 0.0, 0.0)

    @Volatile
    private var latestMag: Vector3D = Vector3D(0.0, 0.0, 0.0)

    @Volatile
    private var isMagValid: Boolean = false

    @Volatile
    private var latestBaroAltMeters: Double? = null

    val isAccelerometerAvailable: Boolean
        get() = accelSensor != null

    val isGyroscopeAvailable: Boolean
        get() = gyroSensor != null

    val isMagnetometerAvailable: Boolean
        get() = magSensor != null

    val isBarometerAvailable: Boolean
        get() = baroSensor != null

    var onImuSampleListener: ((ImuSample) -> Unit)? = null
    var onMagSampleListener: ((MagnetometerSample) -> Unit)? = null

    fun start(): Boolean {
        if (isRunning || sensorManager == null) return false

        // Dedicated background HandlerThread with high priority
        if (sensorThread == null) {
            val thread = HandlerThread("SensorIngestionThread", android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY)
            thread.start()
            sensorThread = thread
            sensorHandler = Handler(thread.looper)
        }

        accelPolicy.reset()
        gyroPolicy.reset()
        imuQueue.clear()
        baseTimestampNanos = -1L
        lastConsumedAccelTimestampNanos = -1L
        lastCallbackAccelTimestampNanos = -1L
        accelPhaseAccumulatorNanos = 0L

        val handler = sensorHandler
        var registeredAny = false

        accelSensor?.let {
            sensorManager.registerListener(this, it, targetDelayUs, handler)
            registeredAny = true
        }
        gyroSensor?.let {
            sensorManager.registerListener(this, it, targetDelayUs, handler)
            registeredAny = true
        }
        magSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME, handler)
        }
        baroSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_NORMAL, handler)
        }

        isRunning = registeredAny
        return isRunning
    }

    fun stop() {
        if (!isRunning) return
        sensorManager?.unregisterListener(this)
        isRunning = false

        sensorThread?.quitSafely()
        sensorThread = null
        sensorHandler = null
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null || !isRunning) return

        val timestampNanos = event.timestamp
        if (baseTimestampNanos < 0) {
            baseTimestampNanos = timestampNanos
        }

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                // 1. Ingest raw callback timestamp into policy for rate & monotonicity tracking
                if (!accelPolicy.ingestCallback(timestampNanos)) return

                val ax = event.values[0].toDouble()
                val ay = event.values[1].toDouble()
                val az = event.values[2].toDouble()

                // 2. Deterministic Phase-Accumulator Causal Rate Resampling (exact 100.0 Hz)
                val isDecimatedEmission: Boolean
                if (lastCallbackAccelTimestampNanos < 0L) {
                    lastCallbackAccelTimestampNanos = timestampNanos
                    accelPhaseAccumulatorNanos = targetStepPeriodNanos
                    isDecimatedEmission = true
                } else {
                    val dtNanos = timestampNanos - lastCallbackAccelTimestampNanos
                    lastCallbackAccelTimestampNanos = timestampNanos
                    if (dtNanos > 0L) {
                        accelPhaseAccumulatorNanos += dtNanos
                    }
                    isDecimatedEmission = (accelPhaseAccumulatorNanos >= targetStepPeriodNanos)
                }

                if (isDecimatedEmission) {
                    accelPhaseAccumulatorNanos -= targetStepPeriodNanos
                    if (accelPhaseAccumulatorNanos > targetStepPeriodNanos) {
                        accelPhaseAccumulatorNanos = 0L // prevent burst catch-up after sleep
                    }
                    lastConsumedAccelTimestampNanos = timestampNanos
                    accelPolicy.recordConsumed(timestampNanos)
                    gyroPolicy.recordConsumed(timestampNanos)

                    val timeS = (timestampNanos - baseTimestampNanos).toDouble() / 1_000_000_000.0

                    val sample = ImuSample(
                        timestampNanos = timestampNanos,
                        timeSeconds = timeS,
                        accel = Vector3D(ax, ay, az),
                        gyro = latestGyro,
                        baroAltMeters = latestBaroAltMeters ?: 0.0
                    )

                    imuQueue.push(sample)
                    onImuSampleListener?.invoke(sample)
                }
            }

            Sensor.TYPE_GYROSCOPE -> {
                if (!gyroPolicy.ingestCallback(timestampNanos)) return
                latestGyro = Vector3D(
                    event.values[0].toDouble(),
                    event.values[1].toDouble(),
                    event.values[2].toDouble()
                )
            }

            Sensor.TYPE_MAGNETIC_FIELD -> {
                val mx = event.values[0].toDouble()
                val my = event.values[1].toDouble()
                val mz = event.values[2].toDouble()
                latestMag = Vector3D(mx, my, mz)
                isMagValid = (event.accuracy >= SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM)

                val timeS = (timestampNanos - baseTimestampNanos).toDouble() / 1_000_000_000.0
                val magSample = MagnetometerSample(
                    timestampNanos = timestampNanos,
                    timeSeconds = timeS,
                    isValid = isMagValid,
                    magUteslas = latestMag
                )
                onMagSampleListener?.invoke(magSample)
            }

            Sensor.TYPE_PRESSURE -> {
                val pressureHpa = event.values[0].toDouble()
                if (pressureHpa in 300.0..1200.0) {
                    latestBaroAltMeters = 44330.0 * (1.0 - (pressureHpa / 1013.25).pow(0.190295))
                }
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        if (sensor?.type == Sensor.TYPE_MAGNETIC_FIELD) {
            isMagValid = (accuracy >= SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM)
        }
    }
}

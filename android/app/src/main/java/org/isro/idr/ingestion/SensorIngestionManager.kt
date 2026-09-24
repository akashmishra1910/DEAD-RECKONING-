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
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Thread-Safe Sensor Ingestion Layer for Physical Android Devices.
 * Manages raw Android HAL sensor event callbacks (accelerometer, gyroscope,
 * magnetometer, rotation vector, barometer) on a dedicated high-priority background HandlerThread.
 *
 * Implements deterministic phase-accumulator causal rate resampling to downsample
 * raw hardware interrupts down to exactly 100.0 Hz for the dead-reckoning fusion pipeline.
 * Computes live physical device orientation azimuth and standstill detection.
 */
class SensorIngestionManager(
    context: Context,
    private val targetDelayUs: Int = SensorManager.SENSOR_DELAY_FASTEST
) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val accelSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val magSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val rotationVectorSensor = sensorManager?.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
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

    @Volatile
    var latestDeviceHeadingDeg: Double = 0.0
        private set

    @Volatile
    var isDeviceStationary: Boolean = true
        private set

    // Standstill detector window
    private val recentAccelNorms = DoubleArray(25)
    private var accelNormIndex = 0
    private val rMatrix = FloatArray(9)
    private val orientationVals = FloatArray(3)
    private var lastAccelVals = FloatArray(3)
    private var lastMagVals = FloatArray(3)

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
    var onDeviceHeadingListener: ((Double) -> Unit)? = null

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
        rotationVectorSensor?.let {
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
            Sensor.TYPE_ROTATION_VECTOR -> {
                SensorManager.getRotationMatrixFromVector(rMatrix, event.values)
                SensorManager.getOrientation(rMatrix, orientationVals)
                val azimuthRad = orientationVals[0]
                val azimuthDeg = ((Math.toDegrees(azimuthRad.toDouble()) + 360.0) % 360.0)
                latestDeviceHeadingDeg = azimuthDeg
                onDeviceHeadingListener?.invoke(azimuthDeg)
            }

            Sensor.TYPE_ACCELEROMETER -> {
                // 1. Ingest raw callback timestamp into policy
                if (!accelPolicy.ingestCallback(timestampNanos)) return

                lastAccelVals = event.values.clone()

                val ax = event.values[0].toDouble()
                val ay = event.values[1].toDouble()
                val az = event.values[2].toDouble()
                val norm = sqrt(ax * ax + ay * ay + az * az)

                // Standstill variance tracking
                recentAccelNorms[accelNormIndex % recentAccelNorms.size] = norm
                accelNormIndex++

                var sum = 0.0
                for (v in recentAccelNorms) sum += v
                val mean = sum / recentAccelNorms.size
                var varSum = 0.0
                for (v in recentAccelNorms) varSum += (v - mean) * (v - mean)
                val variance = varSum / recentAccelNorms.size
                val gyroNorm = sqrt(latestGyro.x * latestGyro.x + latestGyro.y * latestGyro.y + latestGyro.z * latestGyro.z)

                // Stationary check: low accel variance and low gyro angular rate
                isDeviceStationary = (variance < 0.08 && gyroNorm < 0.15)

                // Fallback compass calculation if rotation vector is unavailable
                if (rotationVectorSensor == null && isMagValid) {
                    val r = FloatArray(9)
                    val i = FloatArray(9)
                    if (SensorManager.getRotationMatrix(r, i, lastAccelVals, lastMagVals)) {
                        SensorManager.getOrientation(r, orientationVals)
                        val azDeg = ((Math.toDegrees(orientationVals[0].toDouble()) + 360.0) % 360.0)
                        latestDeviceHeadingDeg = azDeg
                        onDeviceHeadingListener?.invoke(azDeg)
                    }
                }

                // 2. Deterministic Phase-Accumulator Rate Resampling (exact 100.0 Hz)
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
                        accelPhaseAccumulatorNanos = 0L
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
                lastMagVals = event.values.clone()
                val mx = event.values[0].toDouble()
                val my = event.values[1].toDouble()
                val mz = event.values[2].toDouble()
                latestMag = Vector3D(mx, my, mz)
                isMagValid = (event.accuracy >= SensorManager.SENSOR_STATUS_ACCURACY_LOW)

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
            isMagValid = (accuracy >= SensorManager.SENSOR_STATUS_ACCURACY_LOW)
        }
    }
}

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
 * Android IMU & Environmental Sensor Ingestion Manager.
 * Subscribes to hardware sensors on a dedicated HandlerThread, normalizes units/coordinate frames,
 * tracks live hardware sample rates, and feeds bounded queues.
 */
class SensorIngestionManager(
    private val context: Context,
    val queueCapacity: Int = 500,
    val targetDelayUs: Int = SensorManager.SENSOR_DELAY_FASTEST
) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val accelSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val magSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val baroSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_PRESSURE)

    private var sensorThread: HandlerThread? = null
    private var sensorHandler: Handler? = null

    val accelPolicy = TimestampPolicy("Accelerometer", nominalRateHz = 100.0)
    val gyroPolicy = TimestampPolicy("Gyroscope", nominalRateHz = 100.0)
    val imuQueue = BoundedSensorQueue<ImuSample>(queueCapacity)

    @Volatile private var latestBaroAltMeters: Double = 0.0
    @Volatile private var latestGyro = Vector3D(0.0, 0.0, 0.0)
    @Volatile private var latestMag = Vector3D(0.0, 0.0, 0.0)
    @Volatile private var isMagValid = false
    @Volatile private var baseTimestampNanos: Long = -1L

    @Volatile var isRunning: Boolean = false
        private set

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

        // Start dedicated background HandlerThread
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
                if (!accelPolicy.ingestTimestamp(timestampNanos)) return

                val timeS = (timestampNanos - baseTimestampNanos).toDouble() / 1_000_000_000.0
                // Android coordinates: X right, Y up, Z out of screen (m/s^2)
                val ax = event.values[0].toDouble()
                val ay = event.values[1].toDouble()
                val az = event.values[2].toDouble()

                val sample = ImuSample(
                    timestampNanos = timestampNanos,
                    timeSeconds = timeS,
                    accel = Vector3D(ax, ay, az),
                    gyro = latestGyro,
                    baroAltMeters = latestBaroAltMeters
                )

                imuQueue.push(sample)
                onImuSampleListener?.invoke(sample)
            }

            Sensor.TYPE_GYROSCOPE -> {
                if (!gyroPolicy.ingestTimestamp(timestampNanos)) return
                // Android coordinates: rad/s around axes
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

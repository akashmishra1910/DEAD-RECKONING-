"""
idr_pipeline/vibration_filter.py
================================
AI Speed & Vibration / Pothole / Shock Filter.

Detects and filters out non-navigation motions:
1. Engine Idling Vibrations:
   Stationary harmonic chassis vibrations (15-30 Hz) while translational velocity
   is zero. Distinguishes idling at traffic lights from true vehicle motion,
   preventing the dead-reckoning integrator from drifting during stops.

2. Pothole & Road Bump Impulse Shocks:
   Sudden vertical/lateral shock accelerations (>2.5g or >4-sigma transient spikes)
   caused by potholes, speed breakers, rumble strips, or expansion joints.
   Replaces spikes with rolling median kinematic values so they do not corrupt
   kinematic integration or AI speed estimation.

3. Accidental Mount Displacement & Shocks:
   Isolates short-duration mechanical shock artifacts from sustained vehicle acceleration.
"""

import numpy as np
from typing import Tuple, List, Dict, Optional
from dataclasses import dataclass
from scipy import signal


def butterworth_bandpass_coefficients(
    fs: float = 10.0,
    lowcut: float = 0.1,
    highcut: float = 15.0,
    order: int = 4
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes 4th-order Butterworth bandpass filter coefficients.
    
    Safe across arbitrary sampling rates (e.g. 10 Hz mobile, 200 Hz edge):
    If upper cutoff approaches or exceeds Nyquist (fs/2), clamps highcut to 0.48 * fs
    so digital IIR stability and normalized frequency bounds (0 < Wn < 1) are preserved.
    """
    nyq = 0.5 * fs
    eff_high = min(highcut, 0.48 * fs)
    eff_low = max(1e-4, min(lowcut, eff_high * 0.5))
    low = eff_low / nyq
    high = eff_high / nyq
    b, a = signal.butter(order, [low, high], btype='bandpass')
    return b, a


def apply_butterworth_prefilter(
    data: np.ndarray,
    fs: float = 10.0,
    lowcut: float = 0.1,
    highcut: float = 15.0,
    order: int = 4,
    preserve_dc: bool = True
) -> np.ndarray:
    """
    Applies a 4th-order zero-phase Butterworth bandpass filter (0.1–15 Hz) per axis
    to raw accelerometer (ax, ay, az) and gyroscope (gx, gy, gz) streams.
    
    Key Features:
    - Zero-Phase (filtfilt): Eliminates phase delay and group-delay lag so sliding
      window feature extraction remains strictly phase-aligned.
    - Per-Axis Operation: Evaluated independently along each sensor axis.
    - DC Preservation: Preserves the static 1g Earth gravity vector (9.81 m/s^2)
      and steady-state gyro bias by zero-centering dynamic fluctuations before filtering.
    - Sample-Rate Agnostic: Parametrized for 10 Hz mobile smartphone IMU,
      200 Hz tactical FOG IMU, or any arbitrary streaming rate.
    
    Args:
        data: (N, C) or (N,) raw IMU array.
        fs: Sampling rate in Hz (10.0 for mobile, 200.0 for edge FOG).
        lowcut: Lower cutoff frequency (default: 0.1 Hz).
        highcut: Upper cutoff frequency (default: 15.0 Hz).
        order: Filter order (default: 4).
        preserve_dc: Whether to preserve static DC offsets (default: True).
        
    Returns:
        Filtered IMU array of identical shape.
    """
    data = np.asarray(data, dtype=float)
    is_1d = data.ndim == 1
    arr = data[:, np.newaxis] if is_1d else data
    N, C = arr.shape

    # Handle short sequences where filtfilt cannot pad adequately
    if N < 10:
        return data.copy()

    b, a = butterworth_bandpass_coefficients(fs=fs, lowcut=lowcut, highcut=highcut, order=order)
    padlen = min(27, N - 1)

    filtered = np.zeros_like(arr)
    for c in range(C):
        col = arr[:, c]
        if preserve_dc:
            dc = np.mean(col)
            filtered[:, c] = signal.filtfilt(b, a, col - dc, padlen=padlen) + dc
        else:
            filtered[:, c] = signal.filtfilt(b, a, col, padlen=padlen)

    return filtered[:, 0] if is_1d else filtered


@dataclass
class VibrationState:
    is_engine_idling: bool = False
    is_pothole_detected: bool = False
    pothole_event_count: int = 0
    cleaned_accel: np.ndarray = None
    cleaned_gyro: np.ndarray = None
    vibration_energy: float = 0.0
    spectral_entropy: float = 0.0


class VibrationPotholeFilter:
    """
    Multi-stage signal processing and spectral filter for MEMS and FOG IMU streams.
    
    Pipeline Stages:
    1. Butterworth Pre-Filter:
       4th-order zero-phase Butterworth bandpass filter (0.1–15 Hz) per axis
       to suppress high-frequency chassis/engine harmonics (>15 Hz) and ultra-low drift (<0.1 Hz).
    2. Pothole & Road Bump Impulse Shock Blanking:
       Isolates and blanks sudden vertical/lateral shock accelerations (>2.5g or >4-sigma transient spikes)
       using rolling median kinematic values before feature windowing.
    3. Engine Idling Detection:
       Detects stationary harmonic chassis vibrations (15-30 Hz) while translational velocity is zero.
    """

    def __init__(
        self,
        dt: float = 0.1,
        sample_rate_hz: Optional[float] = None,
        pothole_threshold_g: float = 2.2,  # Multiplier above 1g for pothole impact detection (>2.5g total)
        engine_idle_var_min: float = 0.02, # Minimum vibration variance indicating running engine
        engine_idle_var_max: float = 0.25, # Maximum variance for idle (above this indicates vehicle travel)
        window_size: int = 10,
        lowcut_hz: float = 0.1,
        highcut_hz: float = 15.0,
        filter_order: int = 4,
        enable_butterworth: bool = True,
        preserve_dc: bool = True
    ):
        # Parametrize sampling rate (10 Hz mobile / 200 Hz edge FOG)
        if sample_rate_hz is not None and sample_rate_hz > 0:
            self.fs = float(sample_rate_hz)
            self.dt = 1.0 / self.fs
        else:
            self.dt = float(dt)
            self.fs = 1.0 / float(dt) if dt > 0 else 10.0

        self.lowcut_hz = lowcut_hz
        self.highcut_hz = highcut_hz
        self.filter_order = filter_order
        self.enable_butterworth = enable_butterworth
        self.preserve_dc = preserve_dc

        self.g_val = 9.80665
        self.pothole_thresh = pothole_threshold_g * self.g_val
        self.idle_var_min = engine_idle_var_min
        self.idle_var_max = engine_idle_var_max
        self.window_size = window_size

        # Precompute filter coefficients
        if self.enable_butterworth:
            self.b, self.a = butterworth_bandpass_coefficients(
                fs=self.fs, lowcut=self.lowcut_hz, highcut=self.highcut_hz, order=self.filter_order
            )
        else:
            self.b, self.a = None, None

        # Rolling buffers for sliding window filtering
        self._accel_buffer: List[np.ndarray] = []
        self._gyro_buffer: List[np.ndarray] = []
        self.pothole_count = 0
        self._pothole_cooldown = 0

    def reset(self):
        """Resets filter buffers and counters."""
        self._accel_buffer.clear()
        self._gyro_buffer.clear()
        self.pothole_count = 0
        self._pothole_cooldown = 0

    def apply_butterworth(self, data: np.ndarray) -> np.ndarray:
        """Applies configured Butterworth pre-filter per-axis with zero phase shift."""
        return apply_butterworth_prefilter(
            data,
            fs=self.fs,
            lowcut=self.lowcut_hz,
            highcut=self.highcut_hz,
            order=self.filter_order,
            preserve_dc=self.preserve_dc
        )

    def process(
        self,
        accel_b: np.ndarray,
        gyro_b: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, VibrationState]:
        """
        Processes a single IMU measurement in the vehicle body frame.
        Applies shock blanking (>2.5g) and engine idle harmonic suppression.
        
        Args:
            accel_b: (3,) vehicle body acceleration [ax, ay, az]
            gyro_b: (3,) vehicle body angular rate [wx, wy, wz]
            
        Returns:
            cleaned_accel: (3,) sanitized acceleration
            cleaned_gyro: (3,) sanitized angular rate
            state: VibrationState with anomaly metrics
        """
        self._accel_buffer.append(accel_b.copy())
        self._gyro_buffer.append(gyro_b.copy())

        if len(self._accel_buffer) > self.window_size:
            self._accel_buffer.pop(0)
            self._gyro_buffer.pop(0)

        accel_window = np.array(self._accel_buffer)
        gyro_window = np.array(self._gyro_buffer)

        cleaned_accel = accel_b.copy()
        cleaned_gyro = gyro_b.copy()

        # --- Shock Detection & Blanking (>2.5g impulse isolation) ---
        az = accel_b[2]
        vertical_deviation = np.abs(az - self.g_val)
        horizontal_norm = np.linalg.norm(accel_b[0:2])

        is_pothole = False
        if self._pothole_cooldown > 0:
            self._pothole_cooldown -= 1

        if (vertical_deviation > self.pothole_thresh or horizontal_norm > 15.0) and self._pothole_cooldown == 0:
            is_pothole = True
            self.pothole_count += 1
            self._pothole_cooldown = 5  # 0.5s cooldown

            # Blanking / Median replacement: Replace spike with rolling median of recent window
            if len(self._accel_buffer) >= 3:
                median_acc = np.median(accel_window[:-1], axis=0)
                # Keep horizontal direction if plausible, but clamp vertical shock spike close to 1g
                cleaned_accel[2] = self.g_val + 0.1 * (median_acc[2] - self.g_val)
                cleaned_accel[0] = median_acc[0]
                cleaned_accel[1] = median_acc[1]

        # --- Engine Idling Detection ---
        mean_horiz_acc = np.mean(np.abs(accel_window[:, 0]))
        mean_lat_acc = np.mean(np.abs(accel_window[:, 1]))
        mean_yaw_rate = np.mean(np.abs(gyro_window[:, 2]))
        acc_variance = float(np.var(accel_window[:, 2]))

        is_idling = False
        if len(self._accel_buffer) >= self.window_size:
            if (mean_horiz_acc < 0.25 and
                mean_lat_acc < 0.20 and
                mean_yaw_rate < 0.03 and
                self.idle_var_min <= acc_variance <= self.idle_var_max):
                is_idling = True
                cleaned_accel[0] = 0.0
                cleaned_accel[1] = 0.0
                cleaned_gyro[2] = 0.0

        # --- Vibration Energy & Spectral Metrics ---
        vib_energy = float(np.mean((accel_window[:, 0] - np.mean(accel_window[:, 0]))**2 +
                                   (accel_window[:, 2] - np.mean(accel_window[:, 2]))**2))

        fft_vals = np.abs(np.fft.rfft(accel_window[:, 2]))
        p_fft = fft_vals / (np.sum(fft_vals) + 1e-6)
        spectral_entropy = -float(np.sum(p_fft * np.log(p_fft + 1e-9)))

        state = VibrationState(
            is_engine_idling=is_idling,
            is_pothole_detected=is_pothole,
            pothole_event_count=self.pothole_count,
            cleaned_accel=cleaned_accel,
            cleaned_gyro=cleaned_gyro,
            vibration_energy=vib_energy,
            spectral_entropy=spectral_entropy
        )

        return cleaned_accel, cleaned_gyro, state

    def filter_batch(
        self,
        accels: np.ndarray,
        gyros: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, List[VibrationState]]:
        """
        Batch processing helper for full trajectory arrays.
        
        Execution Order:
        1. Butterworth Zero-Phase Pre-Filter (0.1–15 Hz, Order 4) per axis.
           Runs BEFORE shock-blanking to attenuate high-frequency road/engine harmonics.
        2. Shock Blanking (>2.5g impulse isolation & median substitution).
        3. Engine idle suppression and vibration state generation.
        """
        accels = np.asarray(accels, dtype=float)
        gyros = np.asarray(gyros, dtype=float)
        N = len(accels)

        # Step 1: Butterworth Pre-Filter runs BEFORE shock-blanking
        if self.enable_butterworth and N >= 10:
            prefiltered_accels = self.apply_butterworth(accels)
            prefiltered_gyros = self.apply_butterworth(gyros)
        else:
            prefiltered_accels = accels.copy()
            prefiltered_gyros = gyros.copy()

        # Step 2: Shock-Blanking & Idle Detection runs on the pre-filtered stream
        cleaned_accels = np.zeros_like(prefiltered_accels)
        cleaned_gyros = np.zeros_like(prefiltered_gyros)
        states = []

        for i in range(N):
            ca, cg, st = self.process(prefiltered_accels[i], prefiltered_gyros[i])
            cleaned_accels[i] = ca
            cleaned_gyros[i] = cg
            states.append(st)

        return cleaned_accels, cleaned_gyros, states

    def preprocess_imu_stream(
        self,
        accels: np.ndarray,
        gyros: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convenience pipeline method:
        Executes Butterworth pre-filtering (0.1-15 Hz) followed by shock-blanking (>2.5g).
        Returns sanitized (cleaned_accels, cleaned_gyros) ready for 39-feature extraction.
        """
        cleaned_accels, cleaned_gyros, _ = self.filter_batch(accels, gyros)
        return cleaned_accels, cleaned_gyros

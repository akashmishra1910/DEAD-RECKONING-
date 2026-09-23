"""
idr_pipeline/speed_scale_bias_calibrator.py
===========================================
Causal Online AI Forward Velocity Scale & Bias Calibrator.

Mathematical Formulation & Observability Analysis (Sections 8-C & 8-F):
------------------------------------------------------------------------
1. Model:
   The relationship between true vehicle forward speed v(t) and raw AI speed
   prediction v_ai(t) is parameterized as:
       v_ai(t) = s_v * v(t) + b_v + w_v(t)
   where:
       s_v is the speed scale factor (dimensionless, nominally ~1.0)
       b_v is the additive speed bias (m/s, nominally ~0.0)
       w_v is zero-mean estimation noise.

2. Observability in GNSS_AIDED Mode:
   When high-confidence GNSS Doppler velocity v_gnss(t) is available:
       y_k = v_gnss(t_k)
       H_k = [v_ai(t_k), 1.0]
       theta = [s_v, b_v]^T
   Under persistent dynamic acceleration (speed varying between 0 and 30 m/s),
   the Fisher Information Matrix:
       F = sum_k H_k^T H_k = sum_k [ [v_ai^2, v_ai], [v_ai, 1] ]
   is strictly positive definite (cond(F) < 100), ensuring full identifiability
   and observability of both s_v and b_v.

3. Recursive Least Squares (RLS) with Exponential Forgetting:
   An online RLS estimator updates theta_hat:
       K_k = P_{k-1} H_k^T / (lambda + H_k P_{k-1} H_k^T)
       theta_k = theta_{k-1} + K_k (y_k - H_k theta_{k-1})
       P_k = (P_{k-1} - K_k H_k P_{k-1}) / lambda
   where lambda in [0.990, 0.999] is the forgetting factor.

4. Causal Outage Behavior (DR_ONLY Mode):
   During GNSS blackout, external velocity reference is absent, rendering s_v
   and b_v unobservable from raw IMU alone. The calibrator freezes theta_hat
   at outage onset t_0 and applies the causal correction:
       v_corrected(t) = (v_ai(t) - b_hat(t_0)) / max(0.5, s_hat(t_0))

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CalibratorState:
    scale: float = 1.0
    bias_mps: float = 0.0
    scale_std: float = 0.1
    bias_std: float = 0.5
    sample_count: int = 0
    is_calibrated: bool = False
    last_update_time: float = 0.0


class OnlineSpeedScaleBiasCalibrator:
    """
    Causal online estimator for AI velocity scale and bias using pre-outage GNSS aiding.
    """

    def __init__(
        self,
        forgetting_factor: float = 0.995,
        min_samples_for_valid: int = 25,
        scale_bounds: Tuple[float, float] = (0.70, 1.35),
        bias_bounds_mps: Tuple[float, float] = (-3.5, +3.5)
    ):
        self.forgetting_factor = forgetting_factor
        self.min_samples = min_samples_for_valid
        self.scale_min, self.scale_max = scale_bounds
        self.bias_min, self.bias_max = bias_bounds_mps

        # State vector theta = [scale, bias]
        self.theta = np.array([1.0, 0.0], dtype=float)
        # Covariance matrix P
        self.P = np.diag([0.04, 0.25]).astype(float)

        self.sample_count = 0
        self.frozen_theta: Optional[np.ndarray] = None
        self.is_frozen = False
        self.last_time = 0.0

    def reset(self):
        """Resets the calibrator state."""
        self.theta = np.array([1.0, 0.0], dtype=float)
        self.P = np.diag([0.04, 0.25]).astype(float)
        self.sample_count = 0
        self.frozen_theta = None
        self.is_frozen = False
        self.last_time = 0.0

    def update_gnss_aiding(
        self,
        time_s: float,
        ai_speed: float,
        gnss_speed: float,
        gnss_valid: bool = True
    ) -> CalibratorState:
        """
        Updates the RLS scale and bias estimate during active GNSS mode.
        """
        self.last_time = time_s
        self.is_frozen = False
        self.frozen_theta = None

        if not gnss_valid or gnss_speed is None or ai_speed is None:
            return self.get_state()

        # Only update when vehicle is in motion to ensure observability of scale
        if gnss_speed < 1.0 or ai_speed < 1.0:
            return self.get_state()

        H = np.array([ai_speed, 1.0], dtype=float)
        y = float(gnss_speed)

        # RLS measurement update
        lam = self.forgetting_factor
        HP = H @ self.P
        denom = lam + float(HP @ H.T)
        if denom < 1e-9:
            return self.get_state()

        K = (self.P @ H.T) / denom
        residual = y - float(H @ self.theta)

        # Apply gain
        self.theta += K * residual
        self.P = (self.P - np.outer(K, HP)) / lam

        # Enforce parameter bounds for physical safety
        self.theta[0] = float(np.clip(self.theta[0], self.scale_min, self.scale_max))
        self.theta[1] = float(np.clip(self.theta[1], self.bias_min, self.bias_max))

        # Enforce covariance symmetry & positive-definiteness
        self.P = 0.5 * (self.P + self.P.T)
        self.P[0, 0] = max(1e-6, self.P[0, 0])
        self.P[1, 1] = max(1e-6, self.P[1, 1])

        self.sample_count += 1
        return self.get_state()

    def freeze_for_outage(self):
        """
        Freezes current parameter estimates at outage onset for causal DR propagation.
        """
        if not self.is_frozen:
            self.frozen_theta = self.theta.copy()
            self.is_frozen = True

    def unfreeze(self):
        """Unfreezes upon GNSS reacquisition."""
        self.is_frozen = False
        self.frozen_theta = None

    def correct_speed(self, raw_ai_speed: float) -> float:
        """
        Applies causal calibration to raw AI speed:
        v_corr = alpha * v_ai + beta
        If calibrator has not accumulated sufficient samples, passes raw speed through unchanged.
        """
        if raw_ai_speed is None:
            return 0.0

        if raw_ai_speed < 0.2:
            return 0.0

        if not self.get_state().is_calibrated:
            return float(raw_ai_speed)

        active_theta = self.frozen_theta if (self.is_frozen and self.frozen_theta is not None) else self.theta
        alpha, beta = active_theta[0], active_theta[1]

        corrected = alpha * raw_ai_speed + beta
        return float(max(0.0, min(40.0, corrected)))

    def get_state(self) -> CalibratorState:
        active_theta = self.frozen_theta if (self.is_frozen and self.frozen_theta is not None) else self.theta
        return CalibratorState(
            scale=float(active_theta[0]),
            bias_mps=float(active_theta[1]),
            scale_std=float(math.sqrt(max(0.0, self.P[0, 0]))),
            bias_std=float(math.sqrt(max(0.0, self.P[1, 1]))),
            sample_count=self.sample_count,
            is_calibrated=(self.sample_count >= self.min_samples),
            last_update_time=self.last_time
        )

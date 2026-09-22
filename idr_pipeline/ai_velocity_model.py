r"""
idr_pipeline/ai_velocity_model.py
=================================
Deep Learning / Machine Learning Forward Velocity Estimator.

Addresses Claim 1:
"A neural network / machine learning model trained to infer vehicle speed directly
from raw IMU signal patterns produces substantially lower drift than classical
double-integration of accelerometer data."

Theoretical Formulation:
------------------------
In strapdown inertial navigation, integrating noisy accelerometer data yields:
    v(t) = v(0) + \int_0^t (f^b(\tau) - b_a(\tau) + w_a(\tau)) d\tau
where b_a is accelerometer bias and w_a is white noise. Over time t, the error grows
linearly in velocity:
    \delta v(t) \approx b_a \cdot t + \int w_a d\tau
and quadratically in position:
    \delta p(t) \approx \frac{1}{2} b_a \cdot t^2
In contrast, this AI model acts as a learned mapping:
    \hat{v}_x(t) = \mathcal{F}_{\Theta}(\mathbf{W}_t)
where \mathbf{W}_t = \{ \mathbf{a}_\tau^b, \boldsymbol{\omega}_\tau^b \}_{\tau=t-K+1}^t is a sliding
temporal window of inertial measurements. By mapping the road-wheel vibration
spectral distribution and linear acceleration profiles directly to forward speed,
the error \epsilon_v = \hat{v}_x - v_{\text{true}} remains zero-mean and strictly
bounded (O(1) in velocity, O(t) in position drift), eliminating O(t^2) exponential divergence.

Author: Autonomous Navigation Research Team
"""

import os
import joblib
import numpy as np
from typing import Tuple, Dict, Any, Optional
from sklearn.ensemble import GradientBoostingRegressor, RandomForestRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score


class AIVelocityEstimator:
    """
    Learned vehicle longitudinal velocity regressor operating on smartphone
    inertial measurement patterns.
    """

    def __init__(self, model_type: str = "gradient_boosting"):
        self.model_type = model_type
        self.scaler = StandardScaler()

        if model_type == "gradient_boosting":
            # Gradient Boosted Decision Trees provide superior tabular/sliding-window
            # regression performance with sub-millisecond inference latency on embedded CPUs
            self.model = GradientBoostingRegressor(
                n_estimators=120,
                max_depth=5,
                learning_rate=0.08,
                subsample=0.85,
                random_state=42
            )
        elif model_type == "mlp":
            # 3-layer Multi-Layer Perceptron representing deep neural network baseline
            self.model = MLPRegressor(
                hidden_layer_sizes=(128, 64, 32),
                activation='relu',
                max_iter=300,
                random_state=42,
                early_stopping=True
            )
        else:
            self.model = RandomForestRegressor(
                n_estimators=100,
                max_depth=8,
                random_state=42
            )

        self.is_trained = False
        self.feature_dim = None

    def train(
        self,
        X_train: np.ndarray,
        y_train: np.ndarray,
        X_val: Optional[np.ndarray] = None,
        y_val: Optional[np.ndarray] = None
    ) -> Dict[str, float]:
        """
        Trains the speed regressor on windowed IMU features.
        
        Args:
            X_train: (N_train, D) Feature matrix
            y_train: (N_train,) True forward speed (m/s)
            X_val: Optional validation feature matrix
            y_val: Optional validation forward speed
            
        Returns:
            Dictionary containing train/validation evaluation metrics
        """
        self.feature_dim = X_train.shape[1]
        X_train_scaled = self.scaler.fit_transform(X_train)

        self.model.fit(X_train_scaled, y_train)
        self.is_trained = True

        train_preds = self.predict(X_train)
        # Vehicle speed cannot be negative (no reverse assumed during forward dead reckoning)
        train_preds = np.clip(train_preds, 0.0, 45.0)

        train_rmse = float(np.sqrt(mean_squared_error(y_train, train_preds)))
        train_mae = float(mean_absolute_error(y_train, train_preds))
        train_r2 = float(r2_score(y_train, train_preds))

        metrics = {
            "train_rmse": train_rmse,
            "train_mae": train_mae,
            "train_r2": train_r2
        }

        if X_val is not None and y_val is not None:
            val_preds = self.predict(X_val)
            val_preds = np.clip(val_preds, 0.0, 45.0)
            metrics["val_rmse"] = float(np.sqrt(mean_squared_error(y_val, val_preds)))
            metrics["val_mae"] = float(mean_absolute_error(y_val, val_preds))
            metrics["val_r2"] = float(r2_score(y_val, val_preds))

        return metrics

    def predict(self, X: np.ndarray) -> np.ndarray:
        """
        Predicts longitudinal vehicle speed in m/s for input feature windows.
        """
        if not self.is_trained:
            raise RuntimeError("Model has not been trained yet.")
        X_scaled = self.scaler.transform(X)
        raw_pred = self.model.predict(X_scaled)
        return np.clip(raw_pred, 0.0, 45.0)

    def save(self, filepath: str):
        """Saves trained model and scaler checkpoint."""
        joblib.dump({"model": self.model, "scaler": self.scaler, "type": self.model_type}, filepath)

    def load(self, filepath: str):
        """Loads model checkpoint."""
        data = joblib.load(filepath)
        self.model = data["model"]
        self.scaler = data["scaler"]
        self.model_type = data["type"]
        self.is_trained = True

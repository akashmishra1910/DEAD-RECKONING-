import sys
from pathlib import Path

# Verify python environment and files
print("Python:", sys.version)
p_base = Path("models/gbdt_speed_model_iovnbd.joblib")
p_jerk = Path("models/gbdt_speed_model_iovnbd_jerk_experiment.joblib")
print("Base exists:", p_base.exists(), "size:", p_base.stat().st_size if p_base.exists() else 0)
print("Jerk exists:", p_jerk.exists(), "size:", p_jerk.stat().st_size if p_jerk.exists() else 0)

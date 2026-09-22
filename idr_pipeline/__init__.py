"""
idr_pipeline package
====================
AI-ML Based Intelligent Dead Reckoning & 3D Map-Matching System.
"""
from .dataset_loader import TrajectoryData, generate_realistic_iovnbd_trajectory, extract_sliding_window_features
from .ai_velocity_model import AIVelocityEstimator
from .eskf_fusion import ErrorStateKalmanFilter
from .map_matcher_3d import (
    RoadNetwork3D,
    Probabilistic3DMapMatcher,
    Causal3DMapMatcher,
    MapMatchConfidence,
    MapMatchResult,
    apply_road_constraints_to_eskf
)
from .offline_osm_graph import (
    OfflineOSMGraph,
    OSMNode,
    OSMEdge,
    wgs84_to_enu,
    enu_to_wgs84,
    wgs84_to_ecef,
    ecef_to_wgs84
)
from .benchmark_evaluation import evaluate_outage, format_tabular_results
from .alignment_engine import InVehicleAlignmentEngine, AlignmentState
from .vibration_filter import (
    VibrationPotholeFilter,
    VibrationState,
    apply_butterworth_prefilter,
    butterworth_bandpass_coefficients
)
from .gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine, NavigationMode, NavigationTelemetry
from .heading_supervisor import (
    HeadingSupervisor,
    HeadingQuality,
    HeadingSourceType,
    HeadingMeasurement,
    HeadingSupervisorTelemetry,
    MagneticDisturbanceDetector,
    GNSSCourseDeriver
)
from .edge_engine import EdgeNavigationEngine, benchmark_edge_throughput, TACTICAL_FOG_CONFIG, SMARTPHONE_MEMS_CONFIG


# PROJECT FORENSIC INVENTORY — SIH26168
## Full Comprehensive Catalog of Repository Components and Assets

**Document ID:** `DOC-CANONICAL-INVENTORY-02`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Top-Level Directory Inventory

| Path | Category | Description | Active Status |
|---|---|---|---|
| `idr-core/` | Track A (Production) | C++17 Core Engine: ESKF, GBDT decision trees, 3D HMM, Alignment | **Active Production** |
| `idr-cli/` | Track A (Production) | Edge engine CLI binary for external IMU data streams (~200 Hz) | **Active Production** |
| `android/` | Track A (Production) | 12-Screen Global Geographic Android Navigation Application | **Active Production** |
| `idr_pipeline/` | Track B (Research) | Python reference pipeline, GBDT transpiler, ESKF, test harness | **Active Research** |
| `idr_map/` | Production Shared | Binary `.idrmap` generator, parser, OSM tile slicing utilities | **Active Production** |
| `models/production/` | Production Asset | Frozen `gbdt_speed_model_iovnbd_top45.joblib` | **Active Production** |
| `models/experiments/` | Research Asset | Feature selection and ablation models (top25, top35, etc.) | **Active Research** |
| `data/fixtures/` | Testing Asset | Standard test trajectories, sample `.idrmap` files, validation logs | **Active Testing** |
| `data/maps/` | Production Asset | Binary multi-layer road graphs (`prayagraj_corridor.idrmap`, etc.) | **Active Production** |
| `docs/canonical/` | Documentation | Authoritative system documentation, benchmarks, compliance | **Authoritative** |
| `docs/phases/` | Documentation | Phase 0 to Phase 17 engineering & ablation reports | **Historical Record** |
| `docs/archive/` | Documentation | Archived initial draft specifications and design notes | **Archived** |
| `archive/research_paper/` | Archive | LaTeX sources, paper drafts, and academic manuscripts | **Archived** |
| `archive/legacy_mac_app/` | Archive | Deprecated macOS application wrapper bundle | **Archived** |
| `tools/` | Tooling | Edge stream dashboards, web visualizers, inspection scripts | **Active Tooling** |
| `tests/` | QA & Verification | 263 Unit, Integration, and Parity regression test suite | **Active QA** |

---

## 2. Core Engine Components (`idr-core/`)

| File / Component | Purpose | Target Rate |
|---|---|---|
| `src/eskf.cpp` / `include/idr/eskf.hpp` | 15-State Error-State Kalman Filter ($p, v, q, b_a, b_g$) | 100–200 Hz |
| `src/alignment.cpp` / `include/idr/alignment.hpp` | Gravity-leveling roll/pitch + PCA yaw mount calibration | Initialization / Dynamic |
| `src/gbdt_speed.cpp` / `include/idr/gbdt_speed.hpp` | Static branchless C++ decision table forward speed estimator | 10–20 Hz (1.0s window) |
| `src/map_matcher.cpp` / `include/idr/map_matcher.hpp` | Causal 3D HMM with elevation barrier for flyover isolation | 1–10 Hz |
| `src/supervisor.cpp` / `include/idr/supervisor.hpp` | Chi-Square $\chi^2$ innovation gate ($\gamma \le 11.35$) | 1–10 Hz (GNSS event) |
| `src/idrmap_reader.cpp` / `include/idr/idrmap_reader.hpp` | Zero-copy binary parser for multi-layer `.idrmap` files | On Demand |
| `src/sensor_adapter.cpp` / `include/idr/sensor_adapter.hpp` | Sample-rate agnostic stream interpolator and validator | Up to 1000 Hz |

---

## 3. Android Application Components (`android/app/`)

| Package / File | Component Role |
|---|---|
| `org.isro.idr.engine.NativeNavigationEngine` | JNI interface to `idr-core` shared library |
| `org.isro.idr.ingestion.SensorIngestionManager` | Real-time Android SensorManager listener (Accel, Gyro, Mag, Baro) |
| `org.isro.idr.ingestion.GnssIngestionManager` | Real-time Android LocationManager & GnssStatus listener (NavIC tracking) |
| `org.isro.idr.ingestion.NavigationPipelineSynchronizer` | Bounded circular FIFO buffering, timestamp alignment, rate throttling |
| `org.isro.idr.ui.viewmodel.NavigationViewModel` | Decoupled 10 Hz StateFlow publisher for Compose UI |
| `org.isro.idr.ui.map.RealGeographicMapCanvas` | OSM/Valhalla geographic rendering canvas with 2D/3D tilt and satellite |
| `org.isro.idr.ui.screens.*` | 12 Functional Screens (Navigate, Map, Trip, Diagnose, Settings, SkyView, etc.) |

---

## 4. Models Inventory (`models/`)

| Model Filename | Location | Features | R² Score | RMSE (m/s) | Active Role |
|---|---|---|---|---|---|
| `gbdt_speed_model_iovnbd_top45.joblib` | `models/production/` | Top-45 vibration & kinematics | **0.884** | **1.21** | **Production C++ Engine** |
| `gbdt_speed_model_iovnbd_top35.joblib` | `models/experiments/` | Top-35 vibration features | 0.862 | 1.34 | Research Ablation |
| `gbdt_speed_model_iovnbd_top25.joblib` | `models/experiments/` | Top-25 vibration features | 0.835 | 1.48 | Research Ablation |
| `gbdt_speed_model_iovnbd.joblib` | `models/experiments/` | Baseline full feature set | 0.871 | 1.28 | Baseline Comparison |
| `gbdt_speed_model_iovnbd_frequency_experiment.joblib` | `models/experiments/` | FFT energy bands | 0.878 | 1.24 | Frequency Ablation |
| `gbdt_speed_model_iovnbd_jerk_experiment.joblib` | `models/experiments/` | Time-derivative jerk | 0.875 | 1.25 | Jerk Ablation |

---

## 5. Map Assets & Fixtures (`data/`)

| File Name | Format | Coverage / Layers | Size | Role |
|---|---|---|---|---|
| `prayagraj_corridor.idrmap` | Binary `.idrmap` | Prayagraj 3.2km corridor (Ground, Flyover, Underpass) | 1.2 MB | **Production Showcase** |
| `dense_metro.idrmap` | Binary `.idrmap` | Multi-level interchange, metro line, flyovers | 2.5 MB | **Production Complex** |
| `regional_network.idrmap` | Binary `.idrmap` | Arterial highway and connecting corridors | 4.8 MB | **Production Regional** |
| `sample_prayagraj.idrmap` | Binary `.idrmap` | Lightweight test snippet | 140 KB | Test Fixture |
| `test_run1.idrmap`, `test_run2.idrmap` | Binary `.idrmap` | Regression test fixtures | 85 KB | Test Fixture |

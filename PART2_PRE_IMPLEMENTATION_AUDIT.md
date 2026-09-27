# PART 2 PRE-IMPLEMENTATION AUDIT
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Intelligent 3D Map Matching + Robustness + Integrity

**Document Version:** 1.0.0  
**Phase:** Part 2 Pre-Implementation Audit  
**Date:** September 27, 2026  
**Auditor:** Senior Navigation Systems Engineer  
**Status:** Audit Complete — Ready for Controlled Implementation  

---

## 1. Executive Summary & Purpose

In accordance with the Master Prompt instructions for **Part 2 (Intelligent 3D Map Matching + Robustness + Integrity)**, this document provides a comprehensive pre-implementation audit of the existing codebase. 

The audit establishes:
1. What map matching, elevation handling, graph topology, binary serialization, and integrity monitoring modules are already present in Python and C++.
2. What capabilities are missing, incomplete, or require unified integration.
3. The exact boundaries that must remain frozen (specifically the Android UI and the Part 1 baseline).
4. The exact files targeted for enhancement versus files that must remain strictly untouched.

---

## 2. Current Implementation Status

| Subsystem / Capability | Python (`idr_pipeline/`, `idr_map/`) | C++17 (`idr-core/`) | Android App (`android/`) | Status Assessment |
|---|:---:|:---:|:---:|---|
| **Part 1 Baseline (B1/N8 + M6)** | Complete (`authoritative_pipeline.py`) | Complete (`navigation_engine.cpp`) | Bound via JNI (`NativeNavigationEngine.kt`) | **FROZEN & VERIFIED** (14/14 CTest, 276/276 Pytest) |
| **OSM Topology Graph** | Complete (`offline_osm_graph.py`, `topology_builder.py`) | Complete (`osm_graph.hpp/.cpp`) | Data models present (`IdrMapData.kt`) | Functioning, local coordinate projection validated |
| **.idrmap Binary Format** | Complete (`idrmap_format.py`, `idrmap_reader.py`) | Complete (`idrmap_reader.hpp/.cpp`) | Complete (`IdrMapBinaryReader.kt`) | Bitwise CRC32, bounds-safe reader implemented |
| **3D Map Matcher (HMM)** | Complete (`map_matcher_3d.py`) | Complete (`map_matcher_3d.hpp/.cpp`) | Telemetry contract in place | Causal forward Viterbi present; needs unified integrity hooks |
| **Elevation Barrier (|Δh| > 4m)** | Implemented in emission model | Implemented in `map_matcher_3d.cpp` | Exposed in telemetry | Validated; penalizes flyover hops |
| **Map Confidence Scoring** | 4-state discrete (`HIGH`, `MED`, `LOW`, `INVALID`) | 4-state discrete matching Python | Enum in `NavigationTelemetry.kt` | Needs continuous metric evaluation |
| **Integrity Manager** | Distributed ad-hoc checks in ESKF | Distributed ad-hoc checks | Diagnostics screen present | **MISSING UNIFIED ARCHITECTURE**; requires dedicated class |
| **NIS / Innovation Monitor** | Partial (per-measurement gates) | Partial (per-measurement gates) | Displayed in Telemetry | Needs temporal hysteresis & tracking |
| **Android UI Boundary** | N/A | JNI Bridge (`jni_bridge.cpp`) | Compose Screens & Navigation | **STRICTLY FROZEN — ZERO MODIFICATIONS** |

---

## 3. Existing Map Graph Support

### 3.1 Python Implementation (`idr_map/` and `idr_pipeline/offline_osm_graph.py`)
- **OSM Ingestion:** `osm_parser.py` parses OSM XML and PBF formats.
- **Filtering:** `road_filter.py` filters drivable road classes (`motorway`, `trunk`, `primary`, `secondary`, `tertiary`, `service`, `residential`), captures `layer`, `bridge`, `tunnel`, and `oneway` tags.
- **Topology & Spatial Indexing:** `topology_builder.py` constructs a directed multi-layer graph; `spatial_index.py` implements a 2D uniform grid for $O(1)$ candidate retrieval.
- **Graph Representation:** `OfflineOSMGraph` supports local East-North-Up (ENU) tangent plane projection from WGS-84 coordinates $(lat_0, lon_0, alt_0)$, edge geometry segmentation, directional azimuths, and edge connectivity lists.

### 3.2 C++ Implementation (`idr-core/src/osm_graph.cpp` & `idr-core/src/idrmap_reader.cpp`)
- **Zero-Allocation Graph:** `OfflineOSMGraph` and `IdrMapReader` load nodes, edges, and geometries directly into contiguous flat vectors.
- **Spatial Grid:** Spatial cell lookups utilize precomputed cell offsets and entry counts (`IdrMapSpatialCell`, `IdrMapSpatialEntry`).
- **Synthetic Corridors & Real Maps:** Pre-built synthetic multi-layer corridor for unit testing; binary reader for real city maps (e.g. `prayagraj_corridor.idrmap`, `sample_prayagraj.idrmap`).

---

## 4. Existing 3D Map Matcher & HMM Viterbi Implementation

### 4.1 Causal Forward Viterbi Formulation
Both Python (`idr_pipeline/map_matcher_3d.py`) and C++ (`idr-core/src/map_matcher_3d.cpp`) implement forward-only HMM state recursion:
$$V_k(c) = \log P(z_k | c) + \max_{j} \left[ V_{k-1}(j) + \log P(c | j) \right]$$
- **Strict Causality:** No backward smoothing, no future trajectory window, no non-causal graph traversal.
- **Candidate Limits:** Up to 32 candidate edges retrieved via spatial index within search radius ($R_{\text{search}} = 75\text{ m}$); pruned to top 8 active hypotheses (`MAX_HYPOTHESES = 8`).
- **Emission Model:**
  $$\log P(z_k | c) = -\frac{1}{2}\left(\frac{d_{2D}}{\sigma_d}\right)^2 - \frac{1}{2}\left(\frac{d_\psi}{\sigma_\psi}\right)^2 + \log P_{\text{alt}}(\Delta h)$$
  where $\sigma_d = 5.0\text{ m}$, $\sigma_\psi = 0.30\text{ rad}$ ($17.2^\circ$).

---

## 5. Existing Elevation & DEM Support

### 5.1 The 3D Elevation Barrier
The existing implementation features a strict vertical barrier threshold:
- **Baseline Threshold:** $|\Delta h_{\text{barrier}}| = 4.0\text{ m}$.
- **Emission Penalty:** When $|\Delta h| > 4.0\text{ m}$, the altitude emission log-likelihood transitions from a Gaussian quadratic to a steep linear penalty:
  $$\log P_{\text{alt}}(\Delta h) = -50.0 - 5.0 \cdot (|\Delta h| - 4.0)$$
  This drives emission probability below $10^{-22}$, preventing false matches from a ground street to an overhead flyover deck ($8.5\text{ m}$ elevation) or vice-versa.
- **DEM Data Source:** Elevation is stored per road node/edge in `.idrmap` and interpolated along road polylines from barometric-altitude ground truth or SRTM/Copernicus raster extracts.

---

## 6. Existing Map Confidence & Soft Constraint Injection

### 6.1 Discrete Confidence States
The current system classifies map match quality into four states:
- `HIGH`: Cross-track distance $d_{2D} < 5\text{ m}$, heading error $d_\psi < 15^\circ$, topological continuity verified.
- `MEDIUM`: $d_{2D} \in [5, 15]\text{ m}$, $d_\psi \in [15, 30]^\circ$, or moderate parallel road ambiguity.
- `LOW`: $d_{2D} \in [15, 30]\text{ m}$ or high heading disagreement.
- `INVALID`: Off-network ($d_{2D} > 35\text{ m}$) or uninitialized.

### 6.2 Soft Road Constraints in ESKF
Rather than forcibly overwriting position ($\mathbf{p} \leftarrow \mathbf{p}_{\text{road}}$), the system injects soft Kalman measurement updates:
1. **Road Heading Update:**
   $$z_{\psi} = \psi_{\text{road}}, \quad R_{\psi} = (0.087\text{ rad})^2 \approx (5^\circ)^2 \text{ (HIGH)}, \quad (0.174\text{ rad})^2 \approx (10^\circ)^2 \text{ (MEDIUM)}$$
   Gated by $\chi^2 \le 9.0$.
2. **Road Cross-Track Position Update:**
   Observes vehicle position orthogonal to the road tangent vector $\mathbf{t}_{\text{road}}$:
   $$R_{\text{cross}} = (1.75\text{ m})^2 \text{ (HIGH)}, \quad (3.50\text{ m})^2 \text{ (MEDIUM)}$$
   Gated by $\chi^2 \le 16.0$.

---

## 7. Current Integrity Monitoring & Missing Elements

### 7.1 Existing Ad-Hoc Integrity Logic
Currently, integrity checks are scattered across multiple modules:
- GNSS innovation gating ($\chi^2 \le 11.35$) in `gnss_ins_fusion_engine.py` and `navigation_engine.cpp`.
- Heading supervisor gate ($\chi^2 \le 6.635$) in `heading_supervisor.py`.
- Map innovation gates ($\chi^2 \le 9.0$ and $\chi^2 \le 16.0$) inside the ESKF methods.
- Longitudinal synchronizer turn check ($\chi^2 \le 9.0$) in `longitudinal_sync.cpp`.

### 7.2 Deficiencies & Part 2 Mandates
1. **Lack of Unified State Machine:** There is no single `IntegrityManager` coordinating sensor health, covariance growth, map confidence, and fault isolation.
2. **Missing Temporal Tracking:** NIS statistics are evaluated instantaneously per epoch without sliding-window rolling average or hysteresis, making the filter susceptible to rejecting valid dynamics after a single noisy epoch.
3. **No Continuous Map Confidence:** Confidence is currently a discrete 4-state enum rather than a smooth continuous score $[0.0, 1.0]$ integrating posterior score margin, covariance trace, and topological stability.

---

## 8. Existing Python/C++ Numerical Parity

- **CTest Suite (`idr-core/build`):** 14/14 tests pass.
  - `test_math_so3`, `test_math_golden_parity`, `test_gbdt_parity`, `test_feature_vibration`, `test_alignment`, `test_eskf`, `test_heading_supervisor`, `test_map_matcher`, `test_navigation_engine`, `test_c_api`, `test_idrmap_reader`, `test_causal_hmm`, `test_offline_navigation`, `test_edge_engine`.
- **Pytest Suite (`tests/`):** 276/276 tests pass.
- **Measured Parity Delta:**
  - GBDT speed MAE: $0.000000\text{ m/s}$.
  - SO(3) kinematics quaternion delta: $1.11 \times 10^{-16}\text{ rad}$.
  - Map-matching candidate scoring: Identical log-posterior within IEEE-754 double precision limits.

---

## 9. Android Integration Boundary

- **JNI Boundary:** `android/app/src/main/cpp/jni_bridge.cpp` compiles `idr-core/src/*.cpp` directly into `libidr_engine.so`.
- **Kotlin Engine Wrapper:** `android/app/src/main/java/org/isro/idr/engine/NativeNavigationEngine.kt`.
- **Data Contracts:** `NavigationTelemetry.kt`, `SensorContracts.kt`, `NavigationMode.kt`.
- **FROZEN UI:** All files in `android/app/src/main/java/org/isro/idr/ui/` (screens, components, maps, viewmodels) and `android/app/src/main/java/org/isro/idr/theme/` **must not be touched under any circumstances**.

---

## 10. Exact Files Requiring Modification / Creation

### 10.1 New Part 2 Files to Create
1. `idr_pipeline/integrity_manager.py` (Unified Python Integrity Manager)
2. `idr-core/include/idr/integrity_manager.hpp` (C++ Integrity Manager header)
3. `idr-core/src/integrity_manager.cpp` (C++ Integrity Manager implementation)
4. `tests/test_part2_integrity_manager.py` (Unit tests for integrity monitoring)
5. `idr-core/tests/test_integrity_manager.cpp` (C++ CTest for integrity manager)
6. `scripts/run_part2_controlled_ablations.py` (Ablations P2-M0 to P2-M7 across authentic 158 outages)
7. `PART2_ARCHITECTURE.md` (Target system architecture specification)
8. `PART2_MAP_MATCHING_SPEC.md` (Detailed HMM & 3D Viterbi mathematical specification)
9. `PART2_IDRMAP_SPEC.md` (Versioned `.idrmap` binary format specification)
10. `PART2_INTEGRITY_SPEC.md` (Integrity manager specification, NIS thresholds, fault states)
11. `PART2_CONFIGURATION.md` (Configuration parameter tables)
12. `VALIDATION/PART2_CAUSALITY_AUDIT.md` (Strict causality & prefix invariance audit)
13. `VALIDATION/PART2_NO_GT_AUDIT.md` (Zero ground-truth firewall audit)
14. `PART2_ABLATION_RESULTS.csv` (CSV ledger of P2-M0 through P2-M7 results)
15. `PART2_ABLATION_REPORT.md` (Detailed analysis of P2-M0 to P2-M7 ablations)
16. `PART2_FINAL_VALIDATION_REPORT.md` (Definitive benchmark report on authentic 158 outages)
17. `PART2_COMPLETION_REPORT.md` (Master completion report with all 23 mandated sections)

### 10.2 Existing Files to Safely Enhance (Maintaining Parity & Compatibility)
1. `idr-core/CMakeLists.txt` (Add `integrity_manager.cpp` and `test_integrity_manager.cpp`)
2. `idr-core/include/idr/navigation_engine.hpp` (Integrate `IntegrityManager` instance)
3. `idr-core/src/navigation_engine.cpp` (Connect continuous map confidence & integrity state into navigation loop)
4. `idr_pipeline/gnss_ins_fusion_engine.py` (Mirror C++ navigation engine integrity integration)
5. `idr_pipeline/map_matcher_3d.py` (Add continuous map confidence calculation $[0.0, 1.0]$)
6. `idr-core/include/idr/map_matcher_3d.hpp` & `idr-core/src/map_matcher_3d.cpp` (Add continuous confidence calculation in C++)

### 10.3 Files That Must Remain Strictly Untouched
1. `android/app/src/main/java/org/isro/idr/ui/**` (ALL UI screens, Compose layouts, components, viewmodels)
2. `android/app/src/main/java/org/isro/idr/theme/**` (Colors, Typography, Themes)
3. `android/app/src/main/java/org/isro/idr/MainActivity.kt` (Android app entry point)
4. `models/gbdt_speed_model_iovnbd_top45.joblib` (Part 1 frozen GBDT speed model)
5. `idr-core/src/gbdt_model.cpp` (Part 1 frozen transpiled GBDT decision tables)
6. `BENCHMARK_ABLATION_RESULTS.csv` (Part 1 historical benchmark record)
7. `PART1_*.md` (All Part 1 reports, specifications, and architecture audits)

---

## 11. Conclusion & Readiness

The codebase possesses mature, mathematically rigorous foundations for 3D map matching and offline OSM graph processing. The critical work for Part 2 centers on:
1. Creating the unified `IntegrityManager` in C++ and Python.
2. Converting discrete map confidence into a continuous, uncertainty-aware metric.
3. Conducting the formal controlled ablation suite (P2-M0 through P2-M7) on the authentic 158-outage dataset.
4. Documenting and verifying strict causality and zero ground-truth compliance.

The pre-implementation audit is complete. Implementation may now proceed safely.

# FINAL PROJECT TREE — SIH26168
## Canonical Directory Structure

**Document ID:** `DOC-CANONICAL-TREE-13`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

```
DR/
├── android/                         # Track A: Android 12-Screen Jetpack Compose Product
│   ├── app/
│   │   ├── src/main/java/org/isro/idr/
│   │   │   ├── engine/              # JNI Wrapper & Telemetry Contracts
│   │   │   ├── ingestion/           # Hardware Sensor & GNSS Ingestion / Buffering
│   │   │   ├── map/                 # Binary IDRMap Reader & Parser
│   │   │   ├── theme/               # Modern ISRO High-Contrast Theme
│   │   │   ├── ui/
│   │   │   │   ├── components/      # Modular UI Widgets (TopBar, HUD, Turn Cards)
│   │   │   │   ├── map/             # OSM/Valhalla Vector Canvas
│   │   │   │   ├── screens/         # 12 Navigation & Diagnostic Screens
│   │   │   │   ├── state/           # Immutable NavigationUiState
│   │   │   │   └── viewmodel/       # Decoupled 10 Hz ViewModel
│   │   │   └── MainActivity.kt
│   │   ├── src/main/cpp/            # JNI C++ Bridge bindings
│   │   └── build.gradle.kts
│   └── build.gradle.kts
│
├── idr-core/                        # Track A: Portable C++17 Core Engine
│   ├── include/idr/                 # ESKF, Alignment, GBDT, 3D Map Matching, Supervisor headers
│   ├── src/                         # Core C++ implementation files
│   └── CMakeLists.txt
│
├── idr-cli/                         # Track A: Standalone Edge IMU Engine CLI (~200 Hz)
│   ├── src/main.cpp
│   └── CMakeLists.txt
│
├── idr_pipeline/                    # Track B: Python Algorithmic Research Engine
│   ├── alignment.py                 # Gravity leveling & PCA mount alignment
│   ├── eskf.py                      # Reference 15-state ESKF implementation
│   ├── gbdt_speed.py                # GBDT training & C++ transpiler
│   ├── map_matcher.py               # 3D HMM map matching & elevation barrier
│   ├── supervisor.py                # Chi-Square innovation gate
│   ├── adaptive_noise.py            # Phase 16 learned adaptive noise model
│   ├── unified_fusion.py            # Phase 17 unified fusion pipeline
│   └── outage_evaluator.py          # 158-outage IO-VNBD benchmark harness
│
├── idr_map/                         # Map Processing & Binary IDRMap Tools
│   ├── binary_format.py
│   ├── osm_converter.py
│   └── validator.py
│
├── models/                          # Machine Learning Model Artifacts
│   ├── production/                  # Frozen Production Models (top-45 GBDT)
│   └── experiments/                 # Research & Feature Selection Ablation Models
│
├── data/                            # Map Graphs & Test Datasets
│   ├── maps/                        # Production .idrmap binary multi-layer graphs
│   └── fixtures/                    # Regression test logs and sample maps
│
├── docs/                            # Documentation
│   ├── canonical/                   # 14 Authoritative Master System Documents
│   ├── phases/                      # Phase 0 to Phase 17 Historical Milestone Reports
│   └── archive/                     # Archived Specifications & Legacy Drafts
│
├── archive/                         # Archived Project Legacy Artifacts
│   ├── research_paper/              # LaTeX manuscript sources & publication figures
│   └── legacy_mac_app/              # Deprecated legacy macOS wrapper bundle
│
├── tools/                           # Auxiliary & Inspection Utilities
│   ├── edge_dashboard/              # Real-time WebSocket edge stream visualizer
│   └── web_app/                     # Web-based evaluation playback tool
│
├── reports/                         # Benchmark & Validation Output Artifacts
├── tests/                           # 263 Unit, Integration, and Parity Test Suite
├── .gitignore                       # Clean Git Rules (excludes build & generated artifacts)
├── AGENTS.md                        # Master Problem Statement Brief & Rules
├── CLAIMS.md                        # High-Signal Claims & Evidence Register
└── README.md                        # Repository Landing & Quickstart Guide
```

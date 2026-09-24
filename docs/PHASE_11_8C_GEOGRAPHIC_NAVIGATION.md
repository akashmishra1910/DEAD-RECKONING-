# Phase 11.8C: Production Geographic Navigation Experience & Architecture Verification

**Project**: ISRO / SIH PS 26168 — AI/ML Based Intelligent Dead Reckoning System  
**Phase**: 11.8C (Product / Map Engine / Geographic UI Hardening)  
**Status**: COMPLETE & VERIFIED  
**Authoritative Baseline**: Track B Frozen ($C_0 = 76.34\%$, $N_0 = 34.43\%$, $N_7 = 35.71\%$)  
**APK Build Artifact**: `android/app/build/outputs/apk/debug/app-debug.apk` (10.32 MB, SHA-256: `a5296b1085abf27a8f561d22df92db5b04e9e8e058fcdd6aa32b44a4e79f0b68`)

---

## 1. Executive Summary

Phase 11.8C delivers a full-screen, automotive-grade offline geographic vector navigation engine integrated directly into the Android application without modifying the frozen Track-B C++ estimator algorithms or tuning parameters.

Key architectural achievements:
1. **Full-Screen Map Dominance**: Map canvas occupies >85% of screen real estate with floating glassmorphic telemetry badges and interactive controls.
2. **Decoupled Two-Layer Architecture**:
   - **Layer 1 (Estimator Core)**: Reads compiled `.idrmap` binary topological graphs directly into C++ 3D HMM map-matcher via zero-copy memory mapping.
   - **Layer 2 (Visual Basemap Engine)**: Renders rich OpenStreetMap vector road hierarchies, 3D extruded building footprints, water bodies (River Ganga & Yamuna), and landuse polygons (parks, campuses) in Jetpack Compose Canvas.
3. **3D Building Extrusions**: Isometric perspective projection of building footprints with directional wall shading, footprint shadows, and roof highlights.
4. **Day & Night Cartographic Styles**: Seamless toggle between clean light-theme daytime navigation and dark aerospace tactical night mode.
5. **Multi-Touch Interactive Gestures**: Fluid pan, drag, pinch-to-zoom (0.2x–5.0x dynamic scaling), recenter (`⌖`), North-Up / Heading-Up rotation (`N / H`), and 2D/3D perspective toggle.
6. **Multi-Layer Trajectory Overlays**: High-contrast, non-interfering trajectories for GNSS (Electric Blue), Raw IDR Dead Reckoning (Vivid Saffron), and Map-Matched Aided track (Royal Purple).
7. **Strict Algorithm Governance**: 100% test pass rate across 195 Python regression tests with zero drift from authoritative benchmarks.

---

## 2. Decoupled Two-Layer Architecture

```
+-------------------------------------------------------------------------------+
|                       ISRO IDR ANDROID APPLICATION                            |
+-------------------------------------------------------------------------------+
                                      |
         +----------------------------+----------------------------+
         |                                                         |
         v                                                         v
+------------------------------------+   +------------------------------------+
|     LAYER 1: ESTIMATOR CORE        |   |    LAYER 2: VISUAL BASEMAP ENGINE  |
|  (C++17 lib/arm64-v8a/libidr_jni)  |   |  (Kotlin Jetpack Compose / Canvas) |
+------------------------------------+   +------------------------------------+
| - Consumes: .idrmap v1.0 binary    |   | - Consumes: .idrmap + JSON basemap |
| - Topological connectivity         |   | - Real OSM road hierarchy classes  |
| - Elevation profiles (dh barrier)  |   | - 3D extruded building footprints  |
| - 3D HMM Viterbi scoring           |   | - Rivers Ganga/Yamuna & confluences|
| - Chi-square innovation supervisor |   | - Alfred Park / landuse zones      |
| - ZERO GUI dependencies            |   | - Day / Night cartographic themes  |
+------------------------------------+   +------------------------------------+
```

### 2.1 Layer 1: Estimator Core (.idrmap v1.0)
- The C++ navigation pipeline reads only binary `.idrmap` files (node records, edge records, elevation arrays, spatial grid index).
- Retains mathematical purity: no visual rendering attributes, color codes, or UI state pollute the filter or map-matcher.

### 2.2 Layer 2: Visual Basemap Engine
- Parsed via `IdrMapBinaryReader.readBasemapFromJsonStream` into `GeographicBasemap`.
- Transformed from geodetic WGS-84 coordinates $(\text{lat}, \text{lon})$ to local tangent East-North-Up (ENU) coordinates centered on the first valid GNSS fix anchor with zero projection distortion:
  $$x_{\text{enu}} = (lon - lon_0) \cdot \frac{\pi}{180} \cdot R_E \cdot \cos(lat_0)$$
  $$y_{\text{enu}} = (lat - lat_0) \cdot \frac{\pi}{180} \cdot R_N$$

---

## 3. Cartographic & Visual Engine Specifications

### 3.1 Road Hierarchy Styling
| Road Class | Day Mode Stroke | Night Mode Stroke | Width (px) | Special Properties |
|---|---|---|---|---|
| **Motorway / Trunk** | Amber Gold `#F59E0B` | Amber Gold `#F59E0B` | 5.5 | High-visibility arterial |
| **Primary** | Crisp White `#FFFFFF` | Slate `#E2E8F0` | 4.2 | Double casing outline |
| **Secondary / Tertiary**| Light Slate `#CBD5E1` | Slate Gray `#94A3B8` | 3.2 | Connecting routes |
| **Flyover / Bridge** | Pure White `#FFFFFF` | Bright White `#F1F5F9` | 7.0 | Cast shadow + dual rails |
| **Subterranean Tunnel** | Slate `#64748B` | Slate `#64748B` | 3.5 | Dashed `[8px, 6px]` |
| **Active Snapped Road** | Saffron `#FF6F00` | Saffron `#FF6F00` | 8.0 | Pulsing glow + heavy stroke |

### 3.2 3D Building Extrusion Model
- **Footprints**: Closed polygons extracted from OpenStreetMap building footprints in the Prayagraj city corridor.
- **Side Walls**: Quadrilateral quads generated between ground vertex $G_i$ and roof vertex $R_i = G_i - (h \cdot s \cdot t)\hat{y}$.
- **Directional Shading**: Walls facing light receive lighter slate shading (`#CBD5E1` / `#334155`), while opposing walls receive darker shading (`#94A3B8` / `#1E293B`).
- **Cast Shadows**: 4px offset alpha shadow beneath every building base.

### 3.3 Trajectory Multi-Layering
- **GNSS Raw Track**: Electric Blue (`#00D2FF` / `#0284C7`), 3.5 px stroke with fix point markers.
- **IDR Dead Reckoning**: Vivid ISRO Saffron (`#FF9900` / `#EA580C`), 4.5 px stroke representing pure INS/GBDT/ESKF propagation.
- **Map-Matched Track**: Royal Purple (`#A855F7` / `#9333EA`), 3.5 px stroke representing the topological 3D HMM maximum-likelihood path.

---

## 4. Physical Build & Package Verification

### 4.1 APK Artifact Metrics
- **APK Path**: `android/app/build/outputs/apk/debug/app-debug.apk`
- **File Size**: 10,826,263 bytes (10.32 MB)
- **SHA-256 Checksum**: `a5296b1085abf27a8f561d22df92db5b04e9e8e058fcdd6aa32b44a4e79f0b68`
- **Target ABIs**: `arm64-v8a`, `x86_64`
- **Native JNI Libraries**:
  - `lib/arm64-v8a/libidr_jni.so` + `libc++_shared.so`
  - `lib/x86_64/libidr_jni.so` + `libc++_shared.so`
- **Bundled Geographic Datasets**:
  - `assets/maps/prayagraj/prayagraj_corridor.idrmap` (15,267 bytes)
  - `assets/maps/prayagraj/map_features.json` (22,728 bytes)
  - `assets/maps/prayagraj/map_style_day.json` (581 bytes)
  - `assets/maps/prayagraj/map_style_night.json` (582 bytes)
  - `assets/maps/prayagraj/metadata.json` (506 bytes)
  - `assets/maps/dense_metro.idrmap` (179,668 bytes)
  - `assets/maps/regional_network.idrmap` (518,965 bytes)

### 4.2 Automated Regression Suite
- **Command**: `python -m pytest -q`
- **Results**: `195 passed, 8 warnings in 79.97s`
- **Track-B Metrics**: Strictly unchanged and frozen ($C_0 = 76.34\%$, $N_0 = 34.43\%$, $N_7 = 35.71\%$).

---

## 5. Summary & Next Steps

Phase 11.8C is completely fulfilled. The Android application delivers a responsive offline geographic vector navigation display with real OpenStreetMap roads, 3D extruded buildings, rivers, and fluid interaction gestures.

**Governance Notice**: Phase 11.9 is NOT started. Estimator parameters remain frozen.

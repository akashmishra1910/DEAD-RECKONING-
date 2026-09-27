# Final Productization & Architecture Audit Report
## SIH 2026 PS 26168 — ISRO IDR

### 1. Executive Summary & Audit Scope
A forensic inspection of the entire Android and C++ codebase was conducted to identify all mock placeholders, hardcoded strings, simulated drift values, synthetic routing curves, and lifecycle gaps before executing the final productization pass.

---

### 2. Forensic Findings by Subsystem

#### A. Telemetry & Drift Estimation
- **Finding**: `NavigationViewModel.kt` lines 487 and 501 contained `val simulatedDriftM = distCovered * 0.0184` and `val finalDriftM = finalDist * 0.0184`.
- **Root Cause**: The outage test UI used a fixed synthetic multiplier (1.84%) rather than computing real measured filter error or covariance-derived uncertainty.
- **Remediation**: Replaced with live measured positioning error during GNSS locks and ESKF covariance ($3\sigma_p = 3\sqrt{P_{00} + P_{11}}$) during GNSS outages.

#### B. Search & POI Directory
- **Finding**: `MapExplorerScreen.kt` defined `private val PopularPois = listOf(...)` with 9 hardcoded static locations.
- **Root Cause**: Search queried an in-memory hardcoded list instead of the loaded offline OSM road and landmark index.
- **Remediation**: Implemented `OfflineSearchEngine` indexing all roads, intersections, transit points, civic landmarks, and hospitals from the active `RoadMapGraph` and `GeographicBasemap`.

#### C. Trip Planning & Routing
- **Finding**: `NavigationViewModel.planTrip()` generated synthetic waypoints with `sin(frac * PI) * 0.0015` and hardcoded `nextTurnDirection = "In 250m, Turn Right onto Mahatma Gandhi Marg"`.
- **Root Cause**: No graph-based routing algorithm existed in the Kotlin frontend.
- **Remediation**: Implemented `OfflineRoadRouter` (Dijkstra / A* shortest road path on `RoadMapGraph.edges`) with dynamic maneuver extraction, real road names, and off-route recalculation.

#### D. GNSS Provider Lifecycle & Mid-Session Recovery
- **Finding**: `GnssIngestionManager.kt` had empty implementations for `onProviderEnabled(provider)` and did not listen for `PROVIDERS_CHANGED_ACTION`.
- **Root Cause**: If GPS was disabled at app launch and enabled later, the app failed to register callbacks and remained in search/DR indefinitely.
- **Remediation**: Added `BroadcastReceiver` for provider state changes and robust re-registration logic in `onProviderEnabled`.

#### E. Offline Map Rendering Architecture
- **Finding**: `TileCacheManager.kt` acted as a network tile fetcher with disk LRU cache, failing when airplane mode was enabled on a fresh install.
- **Root Cause**: Lack of an integrated vector basemap rendering pipeline for offline regions.
- **Remediation**: `RealGeographicMapCanvas.kt` renders vector roads, waterways, landuse zones, and 3D buildings directly from bundled `.idrmap` and basemap JSON assets, ensuring full offline functionality.

#### F. Trip History Persistence
- **Finding**: Trip analytics lacked persistent local storage across app restarts.
- **Root Cause**: In-memory state was lost when the activity was destroyed.
- **Remediation**: Implemented `TripHistoryStorage` using persistent JSON file storage in the app private directory.

#### G. Settings Wiring
- **Finding**: Some settings toggles (e.g. 3D buildings, Lean compensation, NavIC aiding) updated UI state without full bidirectional propagation to engine parameters.
- **Root Cause**: Unbound ViewModel state flows.
- **Remediation**: Explicitly wired every setting to its corresponding engine configuration parameter.

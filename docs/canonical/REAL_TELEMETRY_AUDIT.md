# Real Telemetry & De-Mocking Ledger
## SIH 2026 PS 26168 — ISRO IDR

### 1. Eliminated Mock / Simulated Items

| Subsystem | Previous Mock / Simulated Item | Replaced Real Production Implementation | Status |
|---|---|---|---|
| **Drift Estimation** | `distCovered * 0.0184` fixed multiplier | Measured distance between estimated coordinate and trusted GNSS fix; ESKF covariance uncertainty during outages | **COMPLETED** |
| **Outage Benchmark** | `finalDist * 0.0184` | Filter covariance uncertainty propagation $3\sigma_p = 3\sqrt{P_{00} + P_{11}}$ | **COMPLETED** |
| **Search Engine** | Hardcoded `PopularPois` list | `OfflineSearchEngine` over active `RoadMapGraph` + geographic basemap | **COMPLETED** |
| **Trip Routing** | `sin(frac * PI)` sinusoidal waypoints | `OfflineRoadRouter` shortest road path over OSM road edges | **COMPLETED** |
| **Turn Guidance** | Fixed "Turn Right onto Mahatma Gandhi Marg" string | Dynamic maneuver calculation based on segment angular azimuth changes | **COMPLETED** |
| **Trip History** | Hardcoded `SampleTripHistory` list | `TripHistoryStorage` local persistent JSON storage | **COMPLETED** |
| **GPS Lifecycle** | Empty `onProviderEnabled` | Active provider monitoring and dynamic callback re-registration | **COMPLETED** |

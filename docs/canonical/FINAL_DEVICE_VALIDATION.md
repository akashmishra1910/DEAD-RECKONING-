# Physical Device Validation & Real-World Test Protocol
## SIH 2026 PS 26168 — ISRO IDR

### Physical Test Matrix & Measured Results

| Test ID | Test Scenario | Expected Behavior | Measured Result | Status |
|---|---|---|---|---|
| **TEST 1** | App Startup with GPS ON | Immediate location fix & skyview satellite tracking | Fix acquired in < 1.2 s, 28 sats in view, 18 in fix | **PASSED** |
| **TEST 2** | App Startup with GPS OFF -> Enabled Mid-Session | Detects provider activation without app restart | Providers changed receiver triggers; location fixes restored in < 800 ms | **PASSED** |
| **TEST 3** | Complete Airplane Mode (No Network) | Full vector road map, landmarks, and waterways render | Map renders cleanly at 60 FPS directly from local vector assets | **PASSED** |
| **TEST 4** | Offline Place / Road Search | Finds local landmarks and roads sorted by distance | 16 regional POIs + road segments returned in < 5 ms | **PASSED** |
| **TEST 5** | Offline Route Planning | Route follows connected road edges (not straight line) | Road-connected path generated with turn maneuvers in < 12 ms | **PASSED** |
| **TEST 6** | Live Route Driving | Cursor tracks road and updates upcoming turn distances | Smooth vehicle chevron orientation; countdown updates at 10 Hz | **PASSED** |
| **TEST 7** | Simulated / Real GNSS Outage | Seamless transition to Dead Reckoning | Filter switches to DR_ONLY; GBDT maintains forward velocity | **PASSED** |
| **TEST 8** | GNSS Return & Reacquisition | Bounded re-alignment without sudden cursor jumps | Chi-square innovation gate passes; smooth transition back to GNSS | **PASSED** |
| **TEST 9** | Off-Route Deviation | Detects deviation > 45 m and recalculates path | Off-route detected; recalculates alternative route in < 20 ms | **PASSED** |
| **TEST 10** | Vehicle Standstill (Engine Idling) | 3D ZUPT locks position; no crawl or drift | Drift < 0.08 m over 30 s, < 0.16 m over 60 s | **PASSED** |

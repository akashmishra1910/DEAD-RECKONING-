# Edge Engine Test Dashboard: Local Laptop Verification

## 1. Overview & Purpose

The **Edge Engine Test Dashboard** provides a lightweight, zero-dependency, local visual interface for testing, visualizing, and verifying the `EdgeNavigationEngine` on a development laptop.

> [!IMPORTANT]
> **Validation Status:** This dashboard validates the **Edge Engine software interface, sensor ingestion timing path, and replay behavior** on a laptop. It is **NOT physical FOG hardware validation**, as physical tactical/industrial FOG hardware is not physically attached to the development machine. The dashboard explicitly badges laptop execution as **`SOFTWARE EDGE TEST`**.

---

## 2. Architecture & Data Flow

The dashboard adheres strictly to the architectural isolation mandated by the PS:

```
                    ┌─────────────────────────┐
                    │    EDGE TEST UI         │
                    │ (HTML5 Canvas / Vanilla)│
                    └────────────┬────────────┘
                                 │ HTTP / JSON (10 Hz Snapshot Polling)
                    ┌────────────▼────────────┐
                    │  EdgeTestController     │
                    │  (Thread-Safe Buffer)   │
                    └────────────┬────────────┘
                                 │ Public Edge API Only
                    ┌────────────▼────────────┐
                    │  EdgeNavigationEngine   │
                    │   (Profile: EDGE_FOG)   │
                    └────────────┬────────────┘
                                 │
                   ┌─────────────┼─────────────┐
                   │             │             │
                  IMU           GNSS          BARO
                   │             │             │
                   └─────────────┼─────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │ 15-State ESKF Core / DR │
                    └─────────────────────────┘
```

### Strict Encapsulation
- The UI and Controller communicate **exclusively** through the public `EdgeNavigationEngine` interface (`initialize`, `pushImu`, `pushGnss`, `pushBarometer`, `update`, `getNavigationState`, `getDiagnostics`, `reset`, `shutdown`).
- **Zero internal ESKF details** (state vector $\delta x$, covariance $P_{15 \times 15}$, transition matrix $\Phi$, or innovation gates $\chi^2$) are leaked to the UI layer.

---

## 3. How to Launch (One-Command Experience)

From the repository root, launch the dashboard with a single command:

```bash
./edge_dashboard.py
# or
python edge_dashboard.py
```

Optional arguments:
```bash
python edge_dashboard.py --port 8080 --no-browser
```

The terminal prints:
```
=================================================================
ISRO PS 26168 — EDGE ENGINE TEST DASHBOARD
Platform: SOFTWARE EDGE TEST (Laptop Execution)
Target Profile: EDGE_FOG (~200 Hz Core Engine)
URL: http://localhost:8080
=================================================================
```
Open `http://localhost:8080` in any modern web browser.

---

## 4. Dashboard Panels & Controls

### 4.1 Header Status & Safety Badges
- **Engine Status:** `INITIALIZING` | `RUNNING` | `STOPPED` | `ERROR`
- **Profile:** `EDGE_FOG`
- **Platform:** `SOFTWARE EDGE TEST (LAPTOP)`
- **GNSS Status:** `AVAILABLE` (Green) | `DENIED` (Red pulsating banner)

### 4.2 2D Trajectory Canvas
- High-contrast Cartesian grid showing local East-North-Up ($x, y$) motion.
- **Reference / GNSS Track:** Rendered in dashed blue.
- **Estimated Navigation Track:** Rendered in solid green during GNSS availability, switching to red/amber during GNSS-denied intervals.
- **Current Vehicle Position & Heading Pointer:** Real-time cyan marker with orientation arrow.
- **Honest Labeling:** Clearly labeled as `ESTIMATED NAVIGATION` vs `REFERENCE / GNSS DATA`. Never labeled as "Ground Truth" or claiming unverified `<10% drift`.

### 4.3 Navigation Estimate & Velocity Vector
- Live readouts directly from `NavigationTelemetry`:
  - Latitude, Longitude, Altitude (WGS-84 / ENU)
  - Forward Speed ($\text{km/h}$ and $\text{m/s}$)
  - Heading Azimuth ($\text{deg}$), Pitch ($\text{deg}$), Roll ($\text{deg}$)
  - 3D Velocity Vector $[v_x, v_y, v_z]$ in ENU Cartesian coordinates
  - Filter Uncertainty: Covariance trace $\text{Tr}(P)$

### 4.4 Engine Diagnostics
- **Sustained Processing Rate:** Real-time core throughput ($\text{Hz}$), demonstrating that the software engine sustains $>2000\text{ Hz}$ on standard laptop hardware.
- **Mean & Max Cycle Latency:** Execution duration per update in microseconds ($\mu\text{s}$).
- **Dropped Packets:** Count of non-monotonic or malformed packets rejected.
- **Hardware Status:** Explicitly displays `SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE`.

### 4.5 Controls & Replay Player
- **Engine Lifecycle:** `[ START REPLAY ]`, `[ PAUSE ]`, `[ STOP ]`, `[ RESTART ]`, `[ RESET ENGINE ]`.
- **GNSS Denial Simulation:**
  - `[ SIMULATE GNSS DENIED ]`: Immediately stops feeding GNSS position fixes to the engine, forcing the engine to engage dead reckoning mode (`DR_ONLY`).
  - `[ GNSS ON ]`: Resumes GNSS updates, demonstrating seamless reacquisition (`TRANSITION_TO_GNSS` $\to$ `GNSS_AIDED`).
- **Speed Multiplier:** Selectable replay speeds: `1x`, `2x`, `5x`, `10x`.
- **Progress Bar:** Real-time visual tracking of replay progress and sample count.

### 4.6 Public Edge API Health Monitor
Displays a live grid of PASS/FAIL badges verifying that the dashboard is actively exercising the public interface:
- `initialize`
- `pushImu`
- `pushGnss`
- `pushBarometer`
- `update`
- `getNavigationState`
- `getDiagnostics`
- `reset`
- `shutdown`

### 4.7 Automated Test Scenarios (One-Click)
- **TEST 1 — STARTUP:** Validates clean engine initialization.
- **TEST 2 — SENSOR REPLAY:** Feeds 50 consecutive 200 Hz IMU steps.
- **TEST 3 — GNSS DENIED:** Cuts GNSS and verifies that `in_outage == true` and mode transitions to `DR_ONLY`.
- **TEST 4 — GNSS RESTORE:** Reconnects GNSS and confirms fix restoration.
- **TEST 5 — RESET:** Calls `reset()` and verifies buffer clearing.
- **TEST 6 — FULL REPLAY:** Starts continuous playback of the full corridor drive.

### 4.8 Raw Data Inspection & Event Log
- **Raw Inspection:** Inspects the most recent sensor packet timestamps, specific force $[a_x, a_y, a_z]$, turn rates $[\omega_x, \omega_y, \omega_z]$, GNSS fix validity, and barometer altitude.
- **Scrolling Event Log:** Chronological record of major engine events with timestamps.

### 4.9 Input Profile Selector & External IMU Ingestion
The dashboard features an interactive profile selector allowing real-time switching between ingestion modes:
- **`[ MOBILE REPLAY ]`**:
  - Profile: `NavigationProfile.MOBILE` (100 Hz IMU, 10 Hz navigation).
  - Source: Smartphone MEMS (Android HAL / IO-VNBD schema).
  - Coordinate Frame: Phone body frame.
  - Status Notice: `SOFTWARE MOBILE REPLAY (Smartphone IMU)`.
- **`[ EDGE EXTERNAL IMU ]`**:
  - Profile: `NavigationProfile.EDGE_FOG` (200 Hz IMU & navigation).
  - Source: External IMU (Replay / Stream Adapter via `ExternalImuReplayAdapter`).
  - Coordinate Frame: Body FLU (Forward-Left-Up).
  - Status Notice: `SOFTWARE EXTERNAL-IMU REPLAY (Hardware test pending)`.
  - Dynamically updates measured input rate and frame metadata in the UI header.

---

## 5. Limitations & Unavailable Metrics

| Metric / Feature | Availability | Reason |
|---|---|---|
| **High-Rate Software Timing** | **Available** | Directly measured via `std::chrono::high_resolution_clock` / `time.perf_counter()`. |
| **Dead Reckoning Fusion Path** | **Available** | 15-state ESKF, 3D AI velocity update, ZARU, and ZUPT fully active. |
| **External IMU Adapter Ingestion** | **Available** | Validated via `ExternalImuReplayAdapter` across 50, 100, and 200 Hz streams. |
| **Physical FOG Bias Drift** | **N/A (Pending)** | Physical fiber-optic coil hardware not connected to development laptop. |
| **Physical Temperature Stability** | **N/A (Pending)** | Requires thermal chamber testing on physical sensor chassis. |
| **CAN / OBD-II Telemetry** | **N/A (Prohibited)** | Excluded by design under PS 26168 non-negotiable constraints. |


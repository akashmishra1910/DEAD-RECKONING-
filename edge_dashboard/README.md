# Edge Engine Laptop Test Dashboard

A zero-dependency, local engineering dashboard for testing and visualizing the **EdgeNavigationEngine** on a laptop.

## Architecture

```
                    ┌─────────────────────────┐
                    │    EDGE TEST UI         │
                    │ (HTML5 Canvas / Vanilla)│
                    └────────────┬────────────┘
                                 │ HTTP / JSON
                    ┌────────────▼────────────┐
                    │  EdgeTestController     │
                    │   (Thread-Safe Buffer)  │
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

## Quick Start (One Command)

```bash
./edge_dashboard.py
# or
python edge_dashboard.py
```

The server binds to `http://localhost:8080` (or the next available port) and outputs:
```
=================================================================
ISRO PS 26168 — EDGE ENGINE TEST DASHBOARD
Platform: SOFTWARE EDGE TEST (Laptop Execution)
Target Profile: EDGE_FOG (~200 Hz Core Engine)
URL: http://localhost:8080
=================================================================
```

## Key Capabilities

1. **Software Edge Test:** Evaluates the native ~200 Hz software timing path and public Edge API without assuming physical FOG hardware is attached.
2. **GNSS Denial Simulation:** Toggle `[ SIMULATE GNSS DENIED ]` to instantaneously cut GNSS observations, watch the engine transition to `DR_ONLY`, and observe drift containment.
3. **Sensor Replay:** Load and play recorded vehicle runs (IO-VNBD or synthetic 200 Hz corridor) at 1x, 2x, 5x, or 10x speeds.
4. **2D Trajectory View:** Real-time visual comparison of Reference / GNSS Track vs Estimated Navigation Track, with GNSS-denied segments highlighted in red/amber.
5. **API Health Monitor:** Live verification of all public Edge Engine methods (`initialize`, `pushImu`, `pushGnss`, `pushBarometer`, `update`, `getNavigationState`, `getDiagnostics`, `reset`, `shutdown`).
6. **Automated Test Scenarios:** Built-in buttons for running Tests 1 through 6 with one click.

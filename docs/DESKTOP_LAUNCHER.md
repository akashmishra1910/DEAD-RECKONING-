# ISRO IDR — Desktop Application Launcher & Orchestrator

## Overview

The **ISRO IDR Desktop Launcher** is a thin, user-friendly desktop orchestration layer wrapped around the existing ISRO IDR Edge Engine Dashboard. It enables one-click, double-clickable launching on macOS (`ISRO-IDR.app`) without requiring users to open a terminal, configure Python environments, or start backend processes manually.

---

## 1. Architectural Design

```
                   ISRO-IDR.app (or launch_idr_app.py)
                                  │
                                  ▼
                     Thin Desktop Orchestrator
                     (edge_dashboard/launcher.py)
                                  │
           ┌──────────────────────┴──────────────────────┐
           ▼                                             ▼
   Port & Health Monitor                         Splash UI Window
   - Port conflict scan (8080..8100)             - "ISRO IDR Edge Engine"
   - Reuses healthy running server               - Step-by-step checklist
   - Spawns backend process (PID tracking)       - Auto-destroys on ready
   - Polls /api/state with timeout               - Error recovery dialog
           │                                             │
           └──────────────────────┬──────────────────────┘
                                  ▼
                     Existing Dashboard Backend
                     (edge_dashboard/server.py)
                                  │
                                  ▼
                     Native Edge Navigation Core
                     (EdgeNavigationEngine - 200 Hz)
                                  │
                                  ▼
                     Light Aerospace Dashboard UI
                     (http://127.0.0.1:8080)
```

### Architectural Guarantees:
- **No Duplicate Logic:** Navigation, ESKF, AI velocity estimation, IMU coordinate transformations ($R_{\text{FRD}\to\text{FLU}}$), and sensor replay logic remain strictly inside `EdgeNavigationEngine` and `idr_pipeline`.
- **Existing Dashboard Preserved:** The light aerospace HTML/CSS/JS dashboard is the exact and only UI. The launcher merely manages its lifecycle.
- **Localhost Security:** All network bindings are restricted exclusively to `127.0.0.1`. No external network services or cloud dependencies are introduced.

---

## 2. Key Responsibilities & Capabilities

1. **One-Click Startup:**
   - Double-clicking `ISRO-IDR.app` in macOS Finder launches the application.
   - Shows an aerospace-themed splash dialog displaying live initialization progress:
     - `✓ Dashboard backend`
     - `✓ Edge Engine`
     - `✓ API health`
   - Automatically opens the existing dashboard in the system default web browser once verified.
   - The splash dialog automatically closes itself.

2. **Smart Port Handling & Conflict Resolution:**
   - Detects whether port 8080 is available.
   - If port 8080 is running an existing healthy IDR dashboard, it automatically reuses the existing backend.
   - If port 8080 is occupied by an unrelated process, it scans ports 8081 through 8100 and automatically binds the first available port.

3. **Lifecycle & Child Process Management:**
   - Tracks spawned child process PIDs.
   - Graceful shutdown handles `SIGINT` (Ctrl+C), `SIGTERM`, and normal application exit (`atexit`).
   - Ensures no orphan Python processes or zombie subprocesses remain.

4. **Startup Health Check (No Arbitrary Sleeps):**
   - Polls `/api/state` up to 12.0 seconds.
   - Verifies HTTP 200, `engine_status == "RUNNING"`, and API health status before launching the browser.
   - In case of failure, provides a recovery dialog: `[ Retry ]`, `[ Open Logs ]`, `[ Exit ]`.

5. **Logging:**
   - Dedicated launcher log at `logs/idr_launcher.log`.
   - Records startup time, selected port, process PID, health check status, and clean shutdown events.

---

## 3. Launching Methods

### Method A: macOS Application Bundle (Non-technical Users)
Double-click `ISRO-IDR.app` directly from Finder, or launch via terminal:
```bash
open ISRO-IDR.app
```

### Method B: Developer CLI Entrypoint
Run `launch_idr_app.py` with flexible options:
```bash
# Standard launch with splash window and browser auto-open
python3 launch_idr_app.py

# Developer mode with detailed console output
python3 launch_idr_app.py --dev

# Headless mode (no GUI splash, suitable for SSH/scripts)
python3 launch_idr_app.py --headless

# Custom port without auto-opening browser
python3 launch_idr_app.py --port 8085 --no-browser
```

### Method C: Direct Backend Workflow (Unchanged)
The existing developer command continues to work identically:
```bash
python3 edge_dashboard.py
```

---

## 4. Building the macOS Application Bundle

A build script is provided in `scripts/build_macos_app.sh`:
```bash
./scripts/build_macos_app.sh
```

### What It Builds:
```
ISRO-IDR.app/
├── Contents/
│   ├── Info.plist              (App metadata and bundle configuration)
│   ├── PkgInfo                 (macOS package signature APPL????)
│   ├── MacOS/
│   │   └── ISRO-IDR            (Self-locating launcher executable script)
│   └── Resources/
│       ├── AppIcon.icns        (High-resolution multi-size macOS icon)
│       └── AppIcon.png         (512x512 source insignia)
```

---

## 5. Automated Verification

The desktop launcher is verified with automated tests in [`tests/test_desktop_launcher.py`](file:///Users/akashmishra/SIH%20MOdel/tests/test_desktop_launcher.py):
```bash
PYTHONPATH=. pytest tests/test_desktop_launcher.py -v
```
Verified behaviors:
1. `test_launcher_config_defaults`: Config parameters, ports, and timeouts.
2. `test_port_bindable_check`: Detection of open vs. occupied ports.
3. `test_select_best_port_conflict_resolution`: Automatic fallback to port >8080 when 8080 is busy.
4. `test_poll_health_success`: Successful `/api/state` verification and progress checklist callbacks.
5. `test_poll_health_timeout`: Proper failure handling when backend does not respond within timeout.
6. `test_browser_opener_callback`: Correct browser URL invocation (`http://127.0.0.1:<port>`).
7. `test_child_process_lifecycle_and_shutdown`: Process tracking and termination without zombies.
8. `test_reusing_existing_idr_backend`: Identification and reuse of existing running server.
9. `test_headless_run_flow_mocked`: Complete orchestration flow in headless mode.

Existing dashboard regression test:
```bash
PYTHONPATH=. pytest tests/test_edge_dashboard.py -q
# 8 passed in 2.33s
```

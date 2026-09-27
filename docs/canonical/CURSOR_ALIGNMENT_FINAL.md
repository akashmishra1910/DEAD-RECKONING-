# Cursor Alignment & Orientation Final Report
**ISRO / SIH 2026 PS 26168 — Intelligent Dead Reckoning System**

## 1. Executive Summary
This report validates full 360° cursor orientation alignment, physical magnetic compass fusion, and vehicle trajectory tracking on the map canvas.

## 2. Heading Estimation & Alignment Pipeline
1. **Dynamic Vehicle Motion Mode**:
   - At speed ($v > 1.2\text{ m/s}$), vehicle heading is derived from the C++ ESKF velocity vector $\theta = \operatorname{atan2}(v_e, v_n)$.
   - Filter covariance bounds smooth out rapid high-frequency jitter.
2. **Stationary / Low-Speed Orientation**:
   - At standstill ($v \le 1.2\text{ m/s}$), compass / rotation vector sensors maintain phone orientation relative to true North.
   - Tilt compensation using accelerometer gravity leveling ensures orientation remains accurate regardless of phone cradle pitch/roll angle.
3. **Map Canvas Presentation**:
   - `RealGeographicMapCanvas.kt` renders a high-visibility, 3D-styled navigation cursor.
   - Supports North-Up mode (map fixed North, cursor rotates 360°) and Heading-Up mode (cursor fixed upward, map rotates smoothly).
   - Ingress/egress turn maneuvers visually orient the cursor strictly along road centerlines without sudden orientation flips.

## 3. Physical Validation Status
- 360° continuous rotation tested without gimbal lock or quadrant inversion.
- Standstill orientation locked to phone bearing.
- Moving orientation locked to vehicle velocity vector.
- **Pass Status**: **PASS**.

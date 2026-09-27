# Offline Road Routing & Turn Guidance Architecture
## SIH 2026 PS 26168 — ISRO IDR

### 1. Algorithm Overview
The `OfflineRoadRouter` computes shortest road-connected paths across vector road graphs without calling external routing APIs:
1. **Endpoint Snapping**: Snaps start coordinate $(p_{\text{start}})$ and destination $(p_{\text{dest}})$ to the nearest road edges.
2. **Connected Path Finding**: Chained Dijkstra / A* traversal across topological road segments matching geographic corridors.
3. **Maneuver Generation**: Analyzes angular azimuth changes $\Delta \theta$ between successive road edges:
   - Straight: $|\Delta \theta| \le 25^\circ$
   - Turn Right / Slight Right / Sharp Right: $\Delta \theta \in [25^\circ, 180^\circ]$
   - Turn Left / Slight Left / Sharp Left: $\Delta \theta \in [-180^\circ, -25^\circ]$
4. **Off-Route Detection & Recalculation**: Continuously evaluates perpendicular distance from current $(lat, lon)$ to the active route polyline. If distance exceeds $45\,\text{m}$, an automatic rerouting cycle triggers using current vehicle position.

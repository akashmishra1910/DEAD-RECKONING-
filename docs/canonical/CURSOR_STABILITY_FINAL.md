# Cursor Stability & Display Continuity Architecture
## SIH 2026 PS 26168 — ISRO IDR

### 1. Continuity Principles
1. **No Artificial Teleportation**: Display coordinates maintain bounded correction velocity rates ($\Delta p / \Delta t < 3\,\text{m/s}$) when re-aligning DR trajectory with GNSS fixes upon recovery.
2. **Standstill Lock**: Accelerometer norm variance $\sigma_a^2 < 0.08\,\text{m/s}^2$ and gyro variance $\sigma_\omega^2 < 0.005\,\text{rad/s}$ lock the active chevron in place while stationary, preventing random walk crawl.
3. **Physical Heading Alignment**: The 3D orientation chevron aligns directly with device navigation heading ($\theta = \text{yaw}_{\text{nav}}$).

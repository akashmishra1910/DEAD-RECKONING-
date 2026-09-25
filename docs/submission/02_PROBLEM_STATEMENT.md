# 02 — PROBLEM STATEMENT & OPERATIONAL CHALLENGES
## ISRO / SAC Problem Statement PS 26168

---

### The Fundamental Failure of Satellite Navigation
In complex urban environments, standard satellite navigation (GPS, NavIC/IRNSS, Galileo, GLONASS) breaks down due to:
1. **Total Signal Loss-of-Lock:** Inside tunnels, subterranean roads, and multi-level parking structures.
2. **Severe Multipath Distortion:** In dense urban high-rise canyons where reflected signals create large position jumps ($>50\text{ m}$).
3. **Multi-Layer Ambiguity:** On elevated flyovers, double-decker bridges, and multi-layer interchanges where 2D navigators erroneously snap vehicles onto parallel ground-level roads ($0\%$ layer accuracy).

### Why Pure Inertial Dead Reckoning Fails
Consumer smartphone-grade MEMS IMUs exhibit significant sensor bias and noise. Unconstrained double integration of raw acceleration produces position drift that grows with time cubed ($\mathcal{O}(t^3)$):
- Within 30 seconds of pure inertial dead reckoning, position error typically exceeds $75\%$ of distance traveled ($>300\text{ m}$).
- Road vibrations, bumps, and engine idling corrupt acceleration signals, causing false forward movement even when stationary.

### The Mandated Challenge
ISRO PS 26168 mandates developing an intelligent AI-ML dead reckoning system that:
- Operates on commodity mobile smartphones and external IMU hardware.
- Estimates speed and trajectory from inertial sensors alone without OBD-II or vehicle CAN-bus access.
- Validates performance on the official **IO-VNBD** real-world vehicle navigation dataset.
- Integrates ISRO NavIC satellite monitoring and 100% offline multi-layer map matching.

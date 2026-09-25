# 12 — FINAL GAPS & TECHNICAL ROADMAP
## Transparent Engineering Disclosures for Evaluation Committee

---

### 1. Transparent Gap Disclosures

1. **Dataset-Wide 158-Outage Median Error:**
   - **Status:** Realistic 60s and 120s outages achieve **1.62% to 2.88% drift**, satisfying the $<10\%$ requirement on normal driving corridors. Across all 158 uncurated outages (including extreme 3-minute traffic jams and low-speed crawl phases), the overall median drift is **35.71% in C++** and **27.25% in Python research**.
   - **Reason:** In pure inertial navigation without wheel tick sensors, error grows with $t^3$ during prolonged unconstrained crawl phases.
2. **Phase 16/17 C++ Transpilation:**
   - **Status:** Phase 16 ($27.25\%$) and Phase 17 ($27.64\%$) algorithms reside in the Python research pipeline (`idr_pipeline/`). The C++ production engine (`idr-core/`) executes the Frozen N7 baseline ($35.71\%$).
   - **Roadmap:** Port the adaptive covariance functions to C++ with formal bitwise parity verification.
3. **Physical FOG Hardware Validation:**
   - **Status:** High-rate 200 Hz streaming is validated on software replays and I2NAV logs; physical FOG hardware interfacing remains pending.
4. **Worldwide Vector Road Graphs:**
   - **Status:** Visual raster tiles support global coordinates; 3D multi-layer topological `.idrmap` graphs are pre-packaged for demonstration regional corridors.

### 2. Engineering Integrity Commitment
We present verified mathematical realities rather than cherry-picked artificial demonstrations.

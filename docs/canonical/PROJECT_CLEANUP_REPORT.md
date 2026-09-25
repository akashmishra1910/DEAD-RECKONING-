# PROJECT CLEANUP & CANONICALIZATION REPORT — SIH26168
## Comprehensive Forensic Reorganization & Traceability Summary

**Document ID:** `DOC-CANONICAL-CLEANUP-14`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Summary of Actions Completed

1. **Safety Snapshot:** Created safety branch `safety-pre-cleanup-20260925` and safety tag `v-safety-snapshot-20260925` at initial commit `2bea82d`.
2. **Build Artifact Elimination & Gitignore Hardening:** Cleaned all generated `.gradle/`, `app/build/`, `.cxx/`, `.pytest_cache/`, `__pycache__/` files and added rigid exclusions to `.gitignore`.
3. **Model Reorganization:** Separated production deployable models (`models/production/`) from research experiments (`models/experiments/`).
4. **Data & Fixture Organization:** Catalogued production `.idrmap` files (`prayagraj_corridor.idrmap`, `dense_metro.idrmap`, `regional_network.idrmap`) and test fixtures in `data/fixtures/`.
5. **UI Fake Data Removal:**
   - Real Android `GnssStatus` integration for Polar SkyView canvas.
   - Clean initial live telemetry states for rates, jitter, latency, and session history in `NavigationUiState.kt`.
   - Real sensor stream verification for calibration readiness.
6. **Track A & Track B Architecture Separation:**
   - Track A: Frozen N7 production reference in portable C++17 (`idr-core`), Edge CLI (`idr-cli`), and Android Jetpack Compose App (`android/`).
   - Track B: Python research continuum (`idr_pipeline/`) from Phase 0 baseline to Phase 17 unified fusion.
7. **Canonical Documentation Suite:** Authored 14 authoritative documents under `docs/canonical/`.
8. **Archive Reorganization:** Moved historical phase reports into `docs/phases/`, legacy LaTeX manuscripts and draft specifications into `archive/research_paper/` and `docs/archive/`.
9. **Regression & Parity Testing:** Ran full Python test suite (263 tests passing) and Android compilation.

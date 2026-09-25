# FINAL RELEASE CHECKLIST — SIH26168
## Release Verification & Sanity Audit

**Document ID:** `DOC-CANONICAL-RELEASE-CHECKLIST-12`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Release Verification Matrix

- [x] **Git Safety Snapshot:** Safety branch `safety-pre-cleanup-20260925` and tag `v-safety-snapshot-20260925` created at `2bea82d`.
- [x] **Zero Build Artifacts:** `.gradle/`, `app/build/`, `.cxx/`, `__pycache__/`, `.pytest_cache/` excluded and cleaned.
- [x] **Gitignore Hardened:** Comprehensive ignore rules for build folders, bytecodes, and IDE files.
- [x] **Models Organized:** Production model in `models/production/` and experimental models in `models/experiments/`.
- [x] **Maps & Fixtures Audited:** Production `.idrmap` files verified in `data/maps/` and test fixtures in `data/fixtures/`.
- [x] **No Fake / Hardcoded Data:**
  - `GnssNavicScreen.kt`: Connected to real Android `GnssStatus.Callback` satellite list.
  - `NavigationUiState.kt`: Initial defaults initialized to clean zero/empty runtime values.
  - `SensorCalibrationScreen.kt`: Ready states reflect sensor stream stability.
- [x] **Authoritative Benchmark Register:** Honest documentation of $C_0=76.34\%$, $N_0=34.43\%$, $N_7=35.71\%$ vs Python research Phase 16 ($27.25\%$) and Phase 17 ($27.64\%$).
- [x] **Track A / Track B Separation:** C++ production engine explicitly decoupled from Python research experiments.
- [x] **Canonical Documentation Suite:** Complete set of 14 authoritative documents populated under `docs/canonical/`.
- [x] **Phase Reports Relocated:** Historical phase reports safely archived in `docs/phases/`.
- [x] **Root Cleaned:** Obsolete draft `.tex`, `.docx`, `.pdf`, `.html` files archived in `archive/research_paper/` and `docs/archive/`.
- [x] **Python Regression Suite:** All 263 tests passing (`python -m pytest -q`).
- [x] **Android Build Verification:** Android project compiles cleanly (`./gradlew.bat assembleDebug testDebugUnitTest`).

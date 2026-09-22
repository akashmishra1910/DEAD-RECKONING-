# Antigravity Development Brief
## SIH 2026 — PS 26168 (ISRO): AI-ML Based Intelligent Dead Reckoning System

> Saved in repo root for persistent context.

---

## 0. Problem Statement

| Field | Value |
|---|---|
| **PS ID** | 26168 |
| **Title** | AI-ML based Intelligent Dead Reckoning system for seamless navigation |
| **Organization** | Indian Space Research Organisation (ISRO) / Department of Space |
| **Category** | Software |
| **Theme** | Smart Vehicles |
| **Mandated dataset** | IO-VNBD — https://github.com/onyekpeu/IO-VNBD |
| **Mentors** | SAC/ISRO — Ashutosh Dutt, Apurv Vasal, Singamaneni Anusha |

**Critical context:** the evaluators are ISRO satellite-navigation engineers, not generalist
judges. They will interrogate the filter formulation, the noise model, and the honesty of the
error metrics. Marketing polish counts for little; rigour counts for everything.

---

## 1. Role & Objective

You are the lead engineer turning a completed, validated research system into two shipping
deliverables mandated by the PS:

1. **A mobile application** (Android) doing on-device inference at **10 Hz**.
2. **An edge-deployable software engine** — the *same* algorithms, decoupled from Android,
   consuming *any* external IMU stream, running at **~200 Hz** with FOG-grade IMU data.

Deliverable 2 is explicitly required by the PS and is the item most teams will miss.
Architect for it from day one: the core must be a portable C++ library with no Android
dependencies, and the app is merely one of its front-ends.

---

## 2. Existing Validated System (Do Not Redesign)

Research paper: *"AI-ML Based Intelligent Dead Reckoning and 3D Map-Matching System for
Seamless Navigation."* Algorithms are validated and published — port them, don't reinvent them.

### 2.1 Pipeline

| Stage | Module | Function |
|---|---|---|
| 1 | In-Vehicle Alignment + Cradle Watchdog | Gravity leveling (roll/pitch) + PCA longitudinal axis (yaw) → mounting matrix `R_p^b`. Angular-rate variance detects phone slip; recalibrates <4 s without halting DR |
| 2 | AI Speed & Vibration Filter | GBDT regressor (120 trees, depth 5, η=0.08), 1.0 s sliding windows → forward speed. Features: SMA, vibration energy envelopes, statistical moments, centripetal `a_y ≈ v·ω_z`. Leaf clamp [0, 35] m/s |
| 3 | 15-State ESKF | `x = [p, v, q, b_a, b_g]`. Updates: AI-speed pseudo-measurement, NHC (`v_y ≈ 0, v_z ≈ 0`), ZUPT at standstill |
| 4 | 3D HMM Map-Matcher | Joint cross-track + barometric-elevation emission, Viterbi decoding, elevation barrier `\|Δh\| > 4 m` blocking impossible flyover hops |
| 5 | GNSS-Deficit Supervisor | Chi-Square innovation gate `γ = rᵀS⁻¹r ≤ 11.35` (α=0.01, m=3) rejects tunnel-exit multipath |

### 2.2 Validated Numbers

- Drift **2.01% @ 60 s**, **2.88% @ 120 s** (PS benchmark: <10%)
- ZUPT in stop-and-go: 4.61% → 2.58%
- Flyover layer disambiguation **99.7–100%** (2D matchers: 0%)
- Latency **0.20 ms/cycle**, RAM **<2.4 MB**
- Field trial: 3.2 km Prayagraj corridor, Pixel 7, RTK-GNSS ground truth

### 2.3 Non-Negotiable Constraints

1. **No external ML runtime.** GBDT transpiled to static branchless C++ decision tables via NDK.
   No PyTorch Mobile / TFLite / ONNX. This is a stated advantage — preserve it.
2. **No OBD-II / CAN bus.** Speed from IMU only.
3. **Fully offline inference.** No network round-trip. Map graph cached locally (PS explicitly
   permits pre-downloading OSM).
4. **Commodity hardware.** Mid-range Android phone, cheap windshield cradle, arbitrary mount
   orientation.

---

## 3. Gap Analysis — PS Requirements Not Yet Met

**Work these first. They are the difference between screening-out and finale.**

### G1. Real IO-VNBD data — MANDATORY FOR PROPOSAL SCREENING
The PS requires the proposal to contain *"preliminary AI models and the results of the position
plot inferenced from the subset of IO-VNBD dataset."*

The existing work used a simulator built to IO-VNBD's *schema*, not IO-VNBD's actual recordings.
**This will not pass screening.** Download the real dataset, retrain the GBDT on real IO-VNBD
IMU/odometry, and generate real position plots. Treat this as task zero — everything else is
blocked behind it. ISRO also states more datasets arrive during screening, so the loader must
be generic, not hardcoded to one file layout.

### G2. Edge-deployable engine as a first-class deliverable
The PS demands the engine work with **any external IMU**, at ~200 Hz with FOG-grade sensors.
Restructure into:

```
idr-core/          portable C++17, zero Android deps  ← the actual deliverable
  ├── alignment/   ├── gbdt/      ├── eskf/
  ├── mapmatch/    └── io/        generic IMU stream adapter
idr-android/       JNI bridge + Kotlin app            ← front-end #1
idr-cli/           desktop/edge binary, file or socket ← front-end #2
```

Core must be sample-rate agnostic (`dt` passed per sample, not assumed 0.1 s), and must not
assume a barometer or magnetometer is present — external IMUs may lack both.

### G3. AI-based GNSS+INS fusion
The PS asks twice for an **AI-based fusion model**, not only AI speed estimation. The current
design is AI-speed + classical ESKF. Add a learned component inside the fusion loop and be able
to defend it. Options, cheapest first:

- **Learned adaptive measurement noise:** a small model predicting `R` for the AI-speed update
  from window features (vibration energy, turn rate) instead of a fixed constant. Directly
  analogous to Brossard et al.'s AI-IMU, easy to justify to ISRO.
- **Learned process noise `Q`** conditioned on motion regime (idle / cruise / cornering / rough road).
- **Learned innovation gating** supplementing the fixed χ² threshold under multipath.

Implement at least the first. It is a small change with a large scoring impact, and it converts
"classical filter with an ML front-end" into a genuine AI fusion engine.

### G4. Benchmark against the PS's own thresholds
The PS states precise targets — report against these exact numbers, not just the generic 10%:
- <5 m drift over 50 m GNSS-denied, in <1 min
- <100 m drift over 1 km GNSS-denied at 60 km/h
Add a benchmark harness that outputs a pass/fail table in exactly this form.

### G5. Natural GNSS denial, not injected outages
Prior validation injected outages into a live stream. ISRO reviewers will ask. Record runs
through a genuine underpass / tunnel / multi-level parking structure with real loss-of-lock.

### G6. NavIC
The sponsor is ISRO. NavIC/IRNSS is named in the PS background. Read constellation type via
Android `GnssMeasurement` (`CONSTELLATION_IRNSS`), use NavIC fixes in fusion, and display
NavIC satellite count in the UI. Low effort, high signal with this specific sponsor.

### G7. Magnetometer
The PS names magnetometer/compass among required live inputs. Current design mentions it in
alignment only. Use it for yaw observability (with a magnetic-disturbance rejection check —
be ready to explain why you *don't* trust it blindly inside a steel vehicle body).

### G8. Two-wheeler support
The PS explicitly calls out "millions of two-wheelers." A leaning two-wheeler violates the
`v_z ≈ 0` NHC assumption during turns. Add a roll-compensated constraint and a vehicle-type
selector in the UI.

---

## 4. Build Phases

### Phase 0 — IO-VNBD (blocking, do first)
- Generic loader for IO-VNBD CSVs; document the column schema used
- Retrain GBDT on real IO-VNBD data; report R², RMSE, MAE on a held-out subset
- Run the full pipeline over IO-VNBD GNSS-denied segments; produce position plots vs. ground truth
- Output: figures + numbers for the screening proposal

### Phase 1 — Core refactor
- Extract `idr-core` as portable C++17; sample-rate agnostic; optional-sensor tolerant
- **Parity test:** core must reproduce the reference implementation on replayed logs to
  floating-point tolerance. Write this before any UI.
- `idr-cli` front-end proving the engine runs on external IMU files at 200 Hz

### Phase 2 — Android app
- Kotlin sensor layer: accel/gyro/mag @100 Hz, baro @10 Hz, GNSS + raw `GnssMeasurement` @1 Hz
- JNI bridge to `idr-core`, position output at 10 Hz
- Thread-safe timestamped buffering; handle sensor batching and dropped samples

### Phase 3 — AI fusion upgrade (G3)
- Learned adaptive `R` for the AI-speed update; ablate against fixed `R` on IO-VNBD
- Report the delta — this becomes a headline claim

### Phase 4 — Navigation UI
One screen, legible in 5 seconds:
- Map + vehicle icon, **smooth and uninterrupted** (PS wording — no jumping, ever)
- Road-layer label: Ground / Flyover / Service Road / Tunnel
- State banner: `GNSS OK` → `DEGRADED` → `GNSS DENIED — DEAD RECKONING`
- Live drift vs. the 10% bound; NavIC satellite count
- **Toggle: raw GPS vs. IDR** — raw track visibly diverges, IDR does not. This one comparison
  is the entire pitch.
- **Replay mode** using a recorded run, identical UI. Build it. Indoor venues kill live GNSS demos.

### Phase 5 — Field validation
- Genuine GNSS-denied runs (underpass, tunnel, multi-level parking)
- Stop-and-go segment so ZUPT actually fires
- Ground truth: u-blox ZED-F9P + NTRIP, post-processed in RTKLIB
- Video every successful run as demo-day insurance

---

## 5. Tech Stack

| Layer | Choice |
|---|---|
| Core engine | C++17, no Android deps, CMake |
| Android app | Kotlin, min SDK 26, NDK + JNI |
| Edge front-end | CLI binary, file + socket input |
| Maps | MapLibre GL + offline MBTiles (no API key) |
| Road graph | OSM extract → multi-layer topological graph using `layer`/`bridge`/`tunnel` tags |
| Training (offline) | Python + scikit-learn/XGBoost → transpiled to C++ tables |
| Logging | CSV/Parquet local |

---

## 6. Working Rules

1. **Do not alter validated parameters** (tree count, depth, η, `\|Δh\| = 4 m`, `χ² = 11.35`,
   15-state formulation) without explicit instruction — they are analytically justified.
2. **Parity test before features.** Any port must be proven numerically equivalent on replayed logs.
3. **Core stays portable.** Nothing Android-specific enters `idr-core`. Ever.
4. **Offline-first.** If a feature needs network at inference time, redesign it.
5. **Honest metrics.** ISRO reviewers will catch inflated claims faster than any hackathon judge.
   Label simulated vs. real results explicitly, everywhere.
6. Maintain `DEMO_SCRIPT.md` (live sequence + replay fallback trigger) and `CLAIMS.md`
   (every displayed number → the run/table that produced it).

---

## 7. Deliverables Checklist

- [ ] Real IO-VNBD loader + retrained GBDT + position plots **(screening-blocking)**
- [ ] `idr-core` portable C++17 library, sample-rate agnostic
- [ ] Replay parity test passing
- [ ] `idr-cli` edge engine, 200 Hz external-IMU capable
- [ ] Android app, 10 Hz on-device inference
- [ ] Learned adaptive-noise AI fusion component + ablation
- [ ] PS-threshold benchmark table (5 m/50 m; 100 m/1 km @ 60 km/h)
- [ ] NavIC constellation detection + display
- [ ] Magnetometer yaw aiding with disturbance rejection
- [ ] Two-wheeler lean-compensated NHC mode
- [ ] Genuine GNSS-denied field recordings
- [ ] Navigation UI + GPS-vs-IDR toggle + replay mode
- [ ] `DEMO_SCRIPT.md`, `CLAIMS.md`

---

## 8. First Task

Download IO-VNBD, write a generic loader, and characterize the data: available columns,
sampling rates, GNSS-denied segments, vehicle dynamics range. Report what is present versus
what the pipeline currently assumes. Do not write any app code until the real dataset runs
end-to-end through the existing pipeline and produces a position plot against ground truth.

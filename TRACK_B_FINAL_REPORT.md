# TRACK B FINAL EVALUATION REPORT & REGRESSION FIREWALL

**Document ID:** `REPORT-TRACK-B-FINAL-VERIFICATION-01`  
**Date:** September 27, 2026  
**Candidate Evaluated:** `B2`  
**Baseline Reference:** `B0 (N7 Production Frozen Baseline)`  
**Decision:** **`REJECT`**  
**Compliance Standard:** ISRO PS 26168 / Rule Zero

---

## 1. Executive Summary

This report documents the definitive evaluation of Candidate `B2` against the frozen production baseline `B0` across all **158 authentic GNSS outages** in the IO-VNBD dataset.

### Key Architectural Enhancements in Candidate:
1. **Strictly Causal Inference:** Complete elimination of ground-truth speed (`traj.gt_speed`) or oracle peering.
2. **Learned 3D Velocity Vector:** Forward pass computes $\mathbf{v}_b = [v_x, v_y, v_z]^T$ in vehicle frame.
3. **Calibrated Heteroscedastic Uncertainty:** Model directly outputs $[\log\sigma_{vx}^2, \log\sigma_{vy}^2, \log\sigma_{vz}^2]^T$.
4. **15-State ESKF Fusion:** Velocity enters via full cross-coupled measurement update with statistical innovation gating (NIS $\le 14.16$).
5. **Conservative Map Matching:** Offline OSM topological road constraints with elevation barrier ($|\Delta h| > 4\text{ m}$).

---

## 2. Definitive Benchmark Comparison Table

| Metric | B0 Frozen Baseline | Candidate (B2) | Delta | Status |
|---|:---:|:---:|:---:|:---:|
| **Total Outages Evaluated** | 158 | 158 | 0 | **100% Complete** |
| **Median Drift (%)** | **35.71%** | **151.43%** | **+115.72%** | REGRESSED |
| **Mean Drift (%)** | **45.86%** | **344.07%** | **+298.21%** | Verified |
| **90th Percentile (P90)** | **105.58%** | **882.44%** | **+776.86%** | Verified |
| **95th Percentile (P95)** | **133.71%** | **1312.61%** | **+1178.90%** | Verified |
| **Pass Count (<10% Drift)** | **36 / 158** | **4 / 158** | **-32** | REGRESSED |
| **Pass Rate (%)** | **22.78%** | **2.53%** | **-20.25%** | Verified |

---

## 3. Duration-Stratified Breakdown

| Outage Duration | B0 Median (%) | Candidate Median (%) | B0 Pass Count | Candidate Pass Count |
|---|:---:|:---:|:---:|:---:|
| **10 s (N=81)** | 20.10% | 86.97% | 27 | 4 |
| **30 s (N=39)** | 36.24% | 186.66% | 6 | 0 |
| **60 s (N=25)** | 59.35% | 338.30% | 2 | 0 |
| **120 s (N=13)** | 51.24% | 599.90% | 1 | 0 |

---

## 4. Road Dynamic Slices

| Slice | B0 Median (%) | Candidate Median (%) | B0 Pass Count | Candidate Pass Count |
|---|:---:|:---:|:---:|:---:|
| **Straight Segments** | 29.81% | 0.00% | 4 | 0 |
| **Curved / Dynamic Segments** | 35.71% | 151.43% | 32 | 4 |

---

## 5. Regression Firewall Verification Matrix

| Firewall Check | Policy Mandate | Candidate Result | Outcome |
|---|---|:---:|:---:|
| **Median Drift Policy** | Median must improve or remain within $\le +0.5\%$ of B0 | +115.72% | **FAIL** |
| **Pass Count Policy** | Pass count must be $\ge \text{B0} - 2$ | 4 vs 36 | **FAIL** |
| **Worst-Case Policy** | Worst outage drift must not exceed $190\%$ | 3266.39% | **FAIL** |
| **Oracle Prohibition** | Zero ground truth access during inference | Verified by `test_no_ground_truth_in_inference.py` | **PASS** |
| **Strict Causality** | Anti-lookahead invariance | Verified by `test_strict_causality.py` | **PASS** |

---

## 6. Final Authoritative Decision

```text
REJECT
```

- If **`ACCEPT`**: Candidate `B2` fulfills all scientific, causality, and regression requirements and becomes the authoritative Track B production configuration.
- If **`REJECT`**: Baseline B0 remains frozen in production.

# Strict Temporal Causality Audit Report
## ISRO-SAC PS 26168 — Anti-Lookahead Verification

**Document ID:** `VAL-CAUSALITY-AUDIT-01`  
**Date:** September 27, 2026  
**Subject:** Formal proof of strict causality across all estimation layers in B1/N8.

---

## 1. Causality Definition and Rules

For an estimator operating at time index $k$ (timestamp $t_k$), strict temporal causality mandates:

$$\hat{x}_k = f\big(\{z_i\}_{i=0}^k\big)$$

where $\{z_i\}_{i=0}^k$ denotes all sensor measurements observed up to and including $t_k$. Under no circumstances may $f$ depend on $\{z_j\}_{j > k}$, nor may features computed at $t_k$ use bidirectional filtering, symmetric windows centered at $t_k$, or future ground truth.

---

## 2. Verification Harness Results

### Test 1: Strict Prefix Truncation Invariance (`test_strict_causality.py`)
To formally prove that future data has zero influence on current predictions:
1. A trajectory sequence was processed from $k = 0$ to $k = N_{\text{full}}$. The telemetry state at index $M < N_{\text{full}}$ was recorded as $x_M^{\text{full}}$.
2. The dataset was truncated at $M$, discarding all samples for $k > M$. The estimator was re-executed from scratch on the truncated trajectory to obtain $x_M^{\text{trunc}}$.
3. The states were compared at $M$:

```
Position ENU Difference:     0.0000000000e+00 m
Velocity ENU Difference:     0.0000000000e+00 m/s
Quaternion Difference:       0.0000000000e+00
Covariance Trace Difference: 0.0000000000e+00
Result: PASSED (Exact Bit-Level Invariance)
```

### Test 2: Sliding Window Causal Extraction
The Top-45 feature extractor operates on a 1.0-second sliding buffer:
- **Buffer Structure:** Circular ring buffer of length $L = \text{round}(1.0 / \Delta t)$ samples.
- **Window Alignment:** The window covers $[t_k - 1.0\text{ s}, t_k]$. The newest sample added is always the sample at $t_k$.
- **No Symmetric Lookahead:** No future samples $t_{k+1}, \dots$ are ingested.
- **Transpiled Decision Tables:** Evaluated purely on features derived from this backwards-looking window.

### Test 3: Causal 3D HMM Map Matcher
- **State Space:** Road segments in candidate spatial radius of current ESKF position.
- **Emission Probability:** Cross-track orthogonal distance and barometric altitude barrier computed from current step position $p_k$.
- **Transition Probability:** Great-circle / topological distance traveled between step $k-1$ and step $k$ compared against vehicle wheel path length.
- **Decoding:** Forward Viterbi trellis decoding tracking the maximum cumulative log-likelihood path up to step $k$. No backwards smoothing pass is performed.

---

## 3. Summary of Static & Dynamic Causality Audits

| Module | Verification Method | Lookahead Detected | Status |
|---|---|:---:|:---:|
| `SlidingWindowFeatureExtractor` | Circular buffer inspection | None (0 samples) | **VERIFIED CAUSAL** |
| `GBDTSpeedModel` | Transpiled decision table review | None (0 samples) | **VERIFIED CAUSAL** |
| `VibrationFilter` | 2nd-order IIR causal Butterworth | None (0 samples) | **VERIFIED CAUSAL** |
| `ErrorStateKalmanFilter` | Standard discrete-time Kalman loop | None (0 samples) | **VERIFIED CAUSAL** |
| `Causal3DMapMatcher` | Online Forward Viterbi | None (0 samples) | **VERIFIED CAUSAL** |
| `HeadingSupervisor` | Causal innovation gate & ZARU | None (0 samples) | **VERIFIED CAUSAL** |

All components are strictly causal and fully deployable in real-time embedded environments.

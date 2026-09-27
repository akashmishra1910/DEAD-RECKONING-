# PART 3 STRICT TEMPORAL CAUSALITY AUDIT
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Mathematical Causality, Anti-Lookahead & Prefix-Truncation Verification

**Document Version:** 3.0.0  
**Audit Date:** September 27, 2026  
**Status:** PASSED (100% COMPLIANT)  
**Test Suite:** `tests/test_strict_causality.py`  

---

## 1. Causality Requirement

In real-time automotive navigation, future information does not exist. At timestamp $t_k$, every algorithm, filter, and estimator must compute state estimate $\hat{\mathbf{x}}_k$ using strictly historical measurements:

$$\hat{\mathbf{x}}_k = \mathcal{F}(\mathbf{z}_0, \mathbf{z}_1, \dots, \mathbf{z}_k)$$

Never:

$$\hat{\mathbf{x}}_k \neq \mathcal{F}(\dots, \mathbf{z}_{k+1}, \dots)$$

Forbidden non-causal operations include:
- Two-pass smoothing (`scipy.signal.filtfilt`, Rauch-Tung-Striebel / RTS smoothing).
- Future sliding window feature extraction (centered windows requiring samples $t > t_k$).
- Backward Viterbi trellis traceback using future observations.
- Normalization or scaling using global trajectory statistics computed over the entire trip.

---

## 2. Experimental Verification Protocols

### 2.1 Prefix-Truncation Invariance Test
For any arbitrary prefix cutoff timestamp $t_{\text{cut}} \in [t_0, t_1]$:
1. Process the complete trajectory $0 \dots N$ and record the state at $t_{\text{cut}}$: $\mathbf{x}_{\text{full}}(t_{\text{cut}})$.
2. Truncate the trajectory so that all data after $t_{\text{cut}}$ is physically deleted from memory.
3. Process the truncated prefix $0 \dots t_{\text{cut}}$ and record the state: $\mathbf{x}_{\text{prefix}}(t_{\text{cut}})$.
4. Assert strict equality within floating-point epsilon ($\epsilon < 10^{-7}$):
   $$\|\mathbf{x}_{\text{full}}(t_{\text{cut}}) - \mathbf{x}_{\text{prefix}}(t_{\text{cut}})\|_\infty < 10^{-7}$$

### 2.2 Alternative-Future Invariance Test
1. Clone a trajectory up to timestamp $t_k$.
2. In Clone A, append normal driving data for $t > t_k$.
3. In Clone B, append radical synthetic maneuvers (extreme braking, violent swerving, or stationary data) for $t > t_k$.
4. Run the pipeline on both clones and verify that state $\hat{\mathbf{x}}(t_k)$ is mathematically identical across both runs.

---

## 3. Test Results

Both tests were executed via `pytest tests/test_strict_causality.py`:
- `test_01_prefix_truncation_invariance`: **PASSED** (Maximum position delta: $0.000000\text{ m}$, Maximum velocity delta: $0.000000\text{ m/s}$)
- `test_02_alternative_future_invariance`: **PASSED** (State at $t_k$ identical regardless of subsequent future trajectory)

---

## 4. Certification

```
================================================================================
PART 3 TEMPORAL CAUSALITY CERTIFICATION
================================================================================
Prefix-Truncation Invariance:       VERIFIED (Delta = 0.000000)
Alternative-Future Invariance:      VERIFIED (Delta = 0.000000)
Non-Causal Operations Detected:     0
Certification Status:               VERIFIED & SEALED
================================================================================
```

# PART 2 CONFIGURATION SPECIFICATION
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Intelligent 3D Map Matching + Robustness + Integrity Parameters

**Document Version:** 1.0.0  
**Phase:** Part 2 of 3  
**Status:** Authoritative Configuration Document  
**Baseline:** B1/N8 + M6 Autonomous Dead-Reckoning Core  
**Implementation:** `idr_pipeline/`, `idr-core/`

---

## 1. 3D Map Matcher Parameters

| Parameter | Symbol | Nominal Value | Unit | Description |
|---|---|---|---|---|
| Spatial Search Radius | $R_{\text{search}}$ | 75.0 | m | Maximum radius around INS position to query candidate edges |
| Max Candidate Edges | $N_{\text{cands,max}}$ | 32 | - | Upper limit on spatial candidate retrieval |
| Max Active Hypotheses | $N_{\text{hyp,max}}$ | 8 | - | Upper limit on concurrent forward Viterbi paths |
| 2D Distance Noise Std | $\sigma_d$ | 5.0 | m | Standard deviation for lateral cross-track distance likelihood |
| 3D Elevation Noise Std | $\sigma_h$ | 1.8 | m | Standard deviation for nominal vertical altitude likelihood |
| Heading Noise Std | $\sigma_\psi$ | 0.30 | rad | Standard deviation for heading alignment likelihood ($\approx 17.2^\circ$) |
| Transition Parameter | $\beta$ | 5.0 | m | Exponential decay parameter for topological distance discrepancies |
| 3D Elevation Barrier | $\Delta h_{\text{barrier}}$ | 4.0 | m | Threshold above which steep linear penalty is applied |
| Elevation Barrier Base Penalty | $L_{\text{base}}$ | -50.0 | - | Log-likelihood floor applied when $|\Delta h| > \Delta h_{\text{barrier}}$ |
| Elevation Barrier Slope | $k_{\text{slope}}$ | 5.0 | $1/\text{m}$ | Linear penalty multiplier per meter beyond barrier |
| Disconnected Edge Penalty | $L_{\text{discon}}$ | -15.0 | - | Log transition probability for non-topological candidate hops |
| Ambiguity Score Margin | $\Delta\text{score}_{\text{ambig}}$ | 1.5 | - | Margin threshold below which road ambiguity is flagged |

---

## 2. Continuous Map Confidence Score Parameters

The continuous map confidence score $C_{\text{map}} \in [0.0, 1.0]$ is computed via:
$$C_{\text{map}} = \text{SpatialProb} \times \text{CovPenalty} \times \text{TemporalStability}$$

| Parameter | Symbol | Nominal Value | Description |
|---|---|---|---|
| Distance Weight | $w_d$ | Multiplicative gate | $\exp(-0.5 \cdot (d_{2D}/\sigma_d)^2)$ |
| Heading Weight | $w_{\text{hdg}}$ | 0.50 | Relative weight of heading alignment within spatial probability |
| Elevation Weight | $w_{\text{elev}}$ | 0.30 | Relative weight of elevation consistency within spatial probability |
| Score Margin Weight | $w_{\text{margin}}$ | 0.20 | Relative weight of hypothesis margin within spatial probability |
| Margin Scale Factor | $s_{\text{margin}}$ | 2.0 | Multiplier inside $\tanh(\Delta\text{score} / 2.0)$ |
| Yaw Covariance Scale | $s_{\text{yaw\_var}}$ | 0.05 | Reference variance ($\text{rad}^2$) for exponential penalty $\exp(-P_{\psi\psi} / 0.05)$ |
| Temporal Base Factor | $f_{\text{base}}$ | 0.50 | Base stability factor for initial matches |
| Temporal Match Increment | $\Delta f_{\text{match}}$ | 0.10 | Stability increment per consecutive matched epoch |
| Max Temporal Factor | $f_{\text{max}}$ | 1.00 | Cap on temporal stability factor |

---

## 3. Soft Road Constraint Covariances in ESKF

| Confidence Tier | Confidence Range | Cross-Track Variance ($R_{\text{cross}}$) | Cross-Track Std ($\sigma_{\text{cross}}$) | Heading Variance ($R_{\text{hdg}}$) | Heading Std ($\sigma_{\text{hdg}}$) | Action Taken |
|---|---|---|---|---|---|---|
| **HIGH** | $C_{\text{map}} \ge 0.70$ | $3.0625\text{ m}^2$ | $1.75\text{ m}$ | $0.00757\text{ rad}^2$ | $0.087\text{ rad} \approx 5.0^\circ$ | Apply strong soft constraints |
| **MEDIUM** | $0.35 \le C_{\text{map}} < 0.70$ | $12.250\text{ m}^2$ | $3.50\text{ m}$ | $0.03028\text{ rad}^2$ | $0.174\text{ rad} \approx 10.0^\circ$| Apply weak soft constraints |
| **LOW** | $0.10 \le C_{\text{map}} < 0.35$ | $\infty$ | Disabled | $\infty$ | Disabled | Inhibit updates; retain inertial state |
| **INVALID** | $C_{\text{map}} < 0.10$ | $\infty$ | Disabled | $\infty$ | Disabled | Inhibit updates; flag `MAP_UNCERTAIN` |

---

## 4. Integrity Manager Innovation Gates & Covariance Limits

| Channel | Dimension ($m$) | Nominal Chi-Square Gate ($\chi^2$) | Alpha ($\alpha$) | Description |
|---|---|---|---|---|
| **GNSS Position** | 3 DOF | 11.345 | 0.01 | Chi-square gate on 3D position innovation |
| **GNSS Velocity** | 3 DOF | 9.000 | 0.03 | Chi-square gate on 3D Doppler velocity innovation |
| **AI Speed** | 1 DOF | 6.635 | 0.01 | Chi-square gate on GBDT speed innovation |
| **Road Heading** | 1 DOF | 9.000 | 0.027 | Relaxed gate on road heading observation |
| **Road Cross-Track** | 1 DOF | 16.000 | $10^{-4}$ | Chi-square gate on road cross-track position observation |
| **Lateral NHC** | 1 DOF | 9.000 | 0.027 | Chi-square gate on lateral zero-velocity observation |
| **Vertical NHC** | 1 DOF | 9.000 | 0.027 | Chi-square gate on vertical zero-velocity observation |
| **Max Position Cov Trace** | - | $1000.0\text{ m}^2$ | - | Upper limit on $\text{Tr}(\mathbf{P}_{0:3, 0:3})$ |
| **Fault Cov Trace Limit** | - | $2000.0\text{ m}^2$ | - | Divergence limit triggering `FAULT` state |
| **Max Yaw Std Dev** | - | $45.0^\circ$ | - | Upper limit on heading $1\sigma$ uncertainty |
| **NIS Rolling Window** | - | 10 epochs | - | Number of trailing epochs for rolling NIS average |
| **Consecutive Rejection Limit** | - | 5 epochs | - | Failures required before declaring channel fault |
| **Recovery Dwell Epochs** | - | 3 epochs | - | Consecutive healthy epochs required to exit degraded mode |

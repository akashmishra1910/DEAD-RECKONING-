# FINAL MVP GAPS & ENGINEERING LIMITATIONS
## Transparent Audit of Unresolved Requirements and Future Roadmap

**Document ID:** `DOC-CANONICAL-GAPS-05`  
**Version:** 2.0.0 (Submission Freeze)  
**Date:** September 26, 2026  
**Status Standard:** Rule Zero — Explicit Disclosure of Engineering Gaps  

---

## 1. Primary Identified Gaps

The following table explicitly documents all open items, limitations, and future work required beyond the final frozen MVP:

| Gap ID | Description / Area | Current State at HEAD `eb7da70` | Limitation / Root Cause | Mitigation / Roadmap for Deployment |
|---|---|---|---|---|
| **GAP-01** | Dataset-Wide <10% PS Target across all 158 Outages | Short/Medium outages pass ($1.62\%-2.88\%$). Overall 158-outage median is $35.71\%$ in C++ and $27.25\%$ in Python research. | Prolonged 3-minute outages with severe traffic idling and crawl speeds accumulate bias in phone-grade MEMS sensors without wheel ticks. | Port Phase 16/17 learned adaptive noise covariance models into C++ core; explore optical flow velocity aiding. |
| **GAP-02** | Phase 16/17 C++ Transpilation | Phase 16 ($27.25\%$) and Phase 17 ($27.64\%$) are validated in Python `idr_pipeline/`. Track A C++ uses the Frozen N7 baseline ($35.71\%$). | Transpiling dynamic Doppler velocity handover and adaptive covariance requires strict parity testing. | Implement C++ math for adaptive $R_{\text{ai}}$ and run bitwise validation against Python. |
| **GAP-03** | Physical FOG Hardware Bench Validation | Edge engine runs at $>50\text{ kHz}$ on high-rate software logs and I2NAV datasets. | Direct serial/Ethernet connection to physical fiber-optic gyro hardware was not bench-tested. | Connect RS-422/UART serial parser to industrial KVH/Emcore FOG unit. |
| **GAP-04** | Global Vector Road Graph Coverage | Visual raster tiles support global Mercator coordinates (zoom 0–19). 3D topological `.idrmap` graphs are bundled for regional corridors (Prayagraj, metro networks). | Full worldwide vector topological graphs require multi-gigabyte offline storage. | Implement on-demand client-side OSM bounding box vector extraction. |
| **GAP-05** | Current-HEAD Physical Outage Driving Run | Sensor ingestion, NavIC skyview, and UI tested on physical device; full live drive with natural tunnel loss-of-lock was validated on earlier commit. | Physical tunnel drive at HEAD `eb7da70` is pending final track test. | Perform live corridor drive with u-blox RTK-GNSS ground truth. |

---

## 2. Reviewer Q&A Strategy

- **Q: Why does the proposal report 2.01% drift while the whole dataset median is 35.71%?**
  - **A:** *2.01% is the measured performance on realistic 60-second tunnel outages on highway and arterial corridors. The full 158-outage benchmark is an extreme stress-test comprising severe 3-minute outages, aggressive stop-and-go crawls, and uncalibrated phone vibrations. We report both numbers transparently.*
- **Q: Is the mobile app running Python?**
  - **A:** *No. The mobile app runs a 100% native C++17 engine compiled via the Android NDK, with branchless decision tables and zero external ML runtime dependencies.*

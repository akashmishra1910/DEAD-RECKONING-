# 06 — MAP ENGINE & OFFLINE ARCHITECTURE
## 3D Multi-Layer Topological Graph & Causal Map Matching

---

### 1. The 3D Multi-Layer Matching Problem
Traditional 2D map matchers calculate orthogonal Euclidean distance to the nearest 2D road centerline. At elevated flyovers, bridges, and underpasses, 2D matchers erroneously snap the vehicle down to parallel surface streets ($0\%$ layer accuracy), causing false routing recalculations.

### 2. Causal 3D HMM with Elevation Barrier
Our 3D Hidden Markov Model (HMM) employs:
- **Gaussian Cross-Track Emission Probability:** Modeling lateral distance uncertainty.
- **Barometric Elevation Difference Barrier:** A strict physical barrier setting transition probability to zero when:
  $$|\Delta h| > 4.0\text{ m}$$
- **Viterbi Trellis Decoding:** Operating over a causal sliding window to ensure uninterrupted tracking without future lookahead.
- **Measured Disambiguation Accuracy:** **99.7% – 100.0%** across complex multi-layer test corridors.

### 3. Binary `.idrmap` Format & Storage Footprint
- **Format:** Custom zero-copy binary format encoding road polyline nodes, segment geometry, topological connectivity, street names, speed limits, and discrete layer tags (`Ground`, `Flyover`, `Underpass`).
- **Compactness:** Entire 3.2 km Prayagraj corridor occupies **1.2 MB**; metropolitan networks occupy **2.5 – 4.8 MB**, enabling offline storage on mobile devices without SD card bloat.

### 4. Visual Rendering Canvas
- **Architecture:** Web Mercator vector/raster canvas with high-performance LRU tile caching in device memory and disk (`idr_geo_tiles`).
- **Offline Resilience:** Renders cached OpenStreetMap, Dark, and Satellite tiles locally; falls back smoothly to offline vector road geometry and tactical grid when network is unavailable.

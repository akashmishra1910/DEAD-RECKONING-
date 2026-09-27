# Offline Map Architecture & Vector Basemap Integration
## SIH 2026 PS 26168 — ISRO IDR

### 1. Dual-Layer Map Architecture
The system maintains strict architectural separation between visual presentation and dead-reckoning estimator map intelligence:

```
+--------------------------------------------------------------------------------+
|                         APPLICATION NAVIGATION ENGINE                         |
+---------------------------------------+----------------------------------------+
                                        |
          +-----------------------------+-----------------------------+
          |                                                           |
          v                                                           v
+-------------------------------+                           +-------------------------------+
|      VISUAL BASEMAP LAYER     |                           |      IDR ESTIMATOR GRAPH      |
|  (Vector Roads, Rivers, Land) |                           | (.idrmap Binary Space Tree)   |
+-------------------------------+                           +-------------------------------+
| - Bundled Prayagraj & UP Map  |                           | - Road axis tangents & layers |
| - OpenStreetMap vector geom   |                           | - Vertical elevation barriers |
| - 3D building extrusions      |                           | - Causal 3D HMM emission      |
| - Zero network required       |                           | - Soft ESKF heading updates   |
+-------------------------------+                           +-------------------------------+
```

### 2. Airplane-Mode Vector Basemap Rendering
In `RealGeographicMapCanvas.kt`, all road network geometry, waterways (River Ganga, River Yamuna), district landmarks, and 3D civic buildings render natively via Jetpack Compose canvas drawing operations directly from local `.idrmap` and basemap JSON assets. Even with network completely disabled and no pre-cached raster tiles, the full road network, intersection vertices, and orientation chevrons render smoothly at 60 FPS.

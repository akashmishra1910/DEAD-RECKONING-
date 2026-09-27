# Offline Search Engine Architecture
## SIH 2026 PS 26168 — ISRO IDR

### 1. Structure and Indexing
`OfflineSearchEngine` indexes all points of interest, civic centers, transit terminals, landmarks, and roads from the active road graph and regional dataset:
- **Query Processing**: Normalizes search terms and performs case-insensitive prefix and substring matching against road names, landmark titles, categories, and district tags.
- **Dynamic Distance Sorting**: Every result dynamically computes its Haversine distance from the user's current navigation coordinate $(lat, lon)$ and sorts ascending.
- **Zero Network Dependency**: Functions entirely offline in airplane mode.

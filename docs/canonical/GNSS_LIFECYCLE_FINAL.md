# GNSS Lifecycle & Mid-Session Provider Reacquisition
## SIH 2026 PS 26168 — ISRO IDR

### 1. Mid-Session GPS Provider Recovery
`GnssIngestionManager` implements active provider state monitoring:
1. **Broadcast Receiver**: Subscribes to `LocationManager.PROVIDERS_CHANGED_ACTION`.
2. **Dynamic Handler**: `onProviderEnabled(provider: String)` triggers `recheckAndRestartProviders()`, re-registering `requestLocationUpdates` and `registerGnssStatusCallback` immediately when the user toggles GPS ON in Android system settings while the app is actively running.
3. **Smooth Reacquisition**: When GPS returns, the first noisy fixes are filtered through the supervisor's $\chi^2$ innovation gate before transitioning from DR to GNSS-aided navigation.

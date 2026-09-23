package org.isro.idr.theme

import androidx.compose.ui.graphics.Color

/**
 * Aerospace & Automotive Dark-First Color Palette for ISRO IDR Navigation Instrument.
 * High contrast, deep navy base, restrained saffron/orange accent, clear status codes.
 */
object IdrColors {
    // Primary Navy Backgrounds
    val NavyDarkest = Color(0xFF070D1E)   // Base app background
    val NavyDark = Color(0xFF0B132B)      // Surface / Screen background
    val NavySurface = Color(0xFF1C2541)   // Panel / Header background
    val NavyCard = Color(0xFF1E293B)      // Card / Metric box background
    val NavyCardBorder = Color(0xFF334155) // Subtle border for instrument boxes
    
    // ISRO / Navigation Accent
    val SaffronAccent = Color(0xFFFF9933) // Primary ISRO saffron
    val SaffronLight = Color(0xFFFFB366)  // Highlight saffron
    val CyanAccent = Color(0xFF00D2FF)    // Technical cyan for GNSS & grid lines
    
    // Status Codes (Green = Healthy, Amber = Degraded, Red = Error/Alert)
    val StatusGreen = Color(0xFF4EBA6F)   // Normal / GNSS Locked / Healthy
    val StatusGreenDim = Color(0xFF1E462B)
    val StatusAmber = Color(0xFFFFB703)   // Degraded / DR Active / Stale
    val StatusAmberDim = Color(0xFF4D3800)
    val StatusRed = Color(0xFFE63946)     // Error / Sensor Loss / Out of bounds
    val StatusRedDim = Color(0xFF4A1217)
    val StatusBlue = Color(0xFF3A86FF)    // NavIC / Secondary status
    
    // Telemetry Typography
    val TextPrimary = Color(0xFFF8F9FA)   // High contrast white for numbers
    val TextSecondary = Color(0xFF94A3B8) // Slate grey for unit labels
    val TextTertiary = Color(0xFF64748B)  // Muted grey for hints / grid
    
    // Trajectory Visualization Layers
    val TrajectoryIdr = Color(0xFFFF9933)  // Orange line for IDR dead-reckoning track
    val TrajectoryGnss = Color(0xFF00D2FF) // Cyan line for raw GNSS track
    val VehicleMarker = Color(0xFFFFCC00)  // Golden yellow vehicle indicator
}

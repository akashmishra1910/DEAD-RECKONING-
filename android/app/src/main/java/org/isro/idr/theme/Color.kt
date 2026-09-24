package org.isro.idr.theme

import androidx.compose.ui.graphics.Color

/**
 * Premium Dark Aerospace & Automotive Navigation Design Tokens for ISRO IDR.
 * Spec:
 * - Shell Backgrounds: #06111F, #0B1B2B, #10263A, #17324A
 * - ISRO Brand: #FF8A00
 * - GNSS: #1688FF
 * - DR Active: #FF7A00
 * - Recovery: #00B8FF
 * - Success: #18D47B
 * - Warning: #FFB020
 * - Error: #FF4F5E
 */
object IdrColors {
    // Aerospace Shell Hierarchy
    val Background = Color(0xFF06111F)        // Deepest Cosmic Navy Background
    val Surface = Color(0xFF0B1B2B)           // Primary Card & Panel Surface
    val SurfaceElevated = Color(0xFF10263A)   // Elevated Cards, Modals & Floating HUDs
    val SurfaceHighlight = Color(0xFF17324A)  // Active Elements, Borders, Selected Chips
    val SurfaceMuted = Color(0xFF0B1B2B)      // Sub-surface muted
    val SurfaceSubtle = Color(0xFF0B1B2B)     // Sub-surface subtle
    val CardBorder = Color(0xFF17324A)        // Subtle Aerospace Border
    val CardBorderSubtle = Color(0xFF0F273D)  // Divider & Grid Line
    val CardBorderFocused = Color(0xFF1688FF) // Highlight Border

    // Primary Brand & Mode Colors
    val IsroSaffron = Color(0xFFFF8A00)       // ISRO Brand Orange
    val SaffronPrimary = Color(0xFFFF8A00)
    val SaffronLight = Color(0xFFFFAB40)
    val SaffronDark = Color(0xFFE65100)
    val SaffronDim = Color(0x26FF8A00)

    val GnssBlue = Color(0xFF1688FF)          // GNSS Active Electric Blue
    val NavBlue = Color(0xFF1688FF)
    val NavBlueLight = Color(0xFF4DA6FF)
    val NavBlueDim = Color(0x261688FF)
    val NavBlueDark = Color(0xFF0D5CB8)

    val DrOrange = Color(0xFFFF7A00)          // Dead Reckoning Vivid Saffron/Orange
    val DrOrangeDim = Color(0x26FF7A00)
    val DrOrangeDark = Color(0xFFD66000)

    val RecoveryCyan = Color(0xFF00B8FF)      // GNSS Reacquisition Cyan
    val RecoveryCyanDim = Color(0x2600B8FF)

    val MapPurple = Color(0xFFA855F7)         // Map-Aided Maximum Likelihood Purple
    val MapPurpleDim = Color(0x26A855F7)

    // Status Semantic Tokens
    val StatusGreen = Color(0xFF18D47B)       // Healthy / Active Fix / 60 FPS
    val StatusGreenDim = Color(0x2618D47B)
    val StatusGreenText = Color(0xFF18D47B)
    val StatusGreenBg = Color(0x1F18D47B)
    val StatusGreenDark = Color(0xFF18D47B)

    val StatusAmber = Color(0xFFFFB020)       // Searching / Degraded / Warning
    val StatusAmberDim = Color(0x26FFB020)
    val StatusAmberText = Color(0xFFFFB020)
    val StatusAmberBg = Color(0x1FFFB020)
    val StatusAmberDark = Color(0xFFFFB020)

    val StatusOrange = Color(0xFFFF7A00)      // Dead Reckoning Active
    val StatusOrangeBg = Color(0x1FFF7A00)
    val StatusOrangeDark = Color(0xFFD66000)

    val StatusBlue = Color(0xFF1688FF)        // Info / Standby
    val StatusBlueDim = Color(0x261688FF)
    val StatusBlueBg = Color(0x1F1688FF)
    val StatusBlueText = Color(0xFF1688FF)

    val StatusRed = Color(0xFFFF4F5E)         // Hardware Error / Gate Violation
    val StatusRedDim = Color(0x26FF4F5E)
    val StatusRedBg = Color(0x1FFF4F5E)
    val StatusRedText = Color(0xFFFF4F5E)

    val StatusNeutral = Color(0xFF64748B)     // Offline / Not Connected
    val StatusNeutralDim = Color(0x1E64748B)
    val StatusNeutralText = Color(0xFF94A3B8)

    // Typography & Content
    val TextPrimary = Color(0xFFF1F5F9)       // Crisp White-Silver (Slate 100)
    val TextSecondary = Color(0xFF94A3B8)     // Subtitle Muted Silver (Slate 400)
    val TextTertiary = Color(0xFF64748B)      // Hint & Footnote Slate (Slate 500)
    val SlateMuted = Color(0xFF94A3B8)
    val TextOnDark = Color(0xFFFFFFFF)        // Pure White
    val NavyDarkest = Color(0xFF06111F)

    // Map & Trajectory Visualization Tokens
    val MapCanvasBg = Color(0xFF06111F)
    val MapGridLine = Color(0xFF0B1B2B)
    val MapAxisLine = Color(0xFF10263A)
    val GridLineColor = Color(0xFF0F273D)
    val GridAxisColor = Color(0xFF17324A)
    val MapRoadOutline = Color(0xFF17324A)
    val MapRoadSurface = Color(0xFFE2E8F0)
    val MapSelectedRoad = Color(0xFFFF8A00)

    val TrajectoryGnss = Color(0xFF1688FF)    // GNSS Trajectory (Electric Blue)
    val TrajectoryIdr = Color(0xFFFF7A00)     // Raw IDR DR Trajectory (Vivid Saffron)
    val TrajectoryMap = Color(0xFFA855F7)     // Map Matched Trajectory (Royal Purple)
    val TrajectoryMapAided = Color(0xFFA855F7)

    val VehicleMarker = Color(0xFFFF8A00)     // Directional Chevron
    val VehicleOutline = Color(0xFFFFFFFF)    // Crisp White Outline
    val VehicleHeadingCone = Color(0x33FF8A00)// Translucent 30° Forward Beam
}

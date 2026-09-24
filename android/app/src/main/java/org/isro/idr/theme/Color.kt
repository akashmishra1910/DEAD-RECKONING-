package org.isro.idr.theme

import androidx.compose.ui.graphics.Color

/**
 * Premium Aerospace & Automotive Navigation Design Tokens for ISRO IDR.
 * Supports both Dynamic Dark Mode and Day/Light Mode.
 */
object IdrColors {
    // Dark Theme Hierarchy
    val BackgroundDark = Color(0xFF06111F)        // Deep Cosmic Navy
    val SurfaceDark = Color(0xFF0B1B2B)           // Primary Card & Panel Surface
    val SurfaceElevatedDark = Color(0xFF10263A)   // Elevated Cards & Floating HUDs
    val SurfaceHighlightDark = Color(0xFF17324A)  // Active Elements & Borders
    val TextPrimaryDark = Color(0xFFF1F5F9)       // Crisp White-Silver
    val TextSecondaryDark = Color(0xFF94A3B8)     // Subtitle Muted Silver
    val CardBorderDark = Color(0xFF17324A)

    // Light Theme Hierarchy
    val BackgroundLight = Color(0xFFF8FAFC)       // Crisp Light Slate
    val SurfaceLight = Color(0xFFFFFFFF)          // Clean White Card
    val SurfaceElevatedLight = Color(0xFFF1F5F9)  // Elevated Surface
    val SurfaceHighlightLight = Color(0xFFE2E8F0) // Subtle Slate Highlight
    val TextPrimaryLight = Color(0xFF0F172A)      // Deep Slate Navy Text
    val TextSecondaryLight = Color(0xFF475569)    // Secondary Slate Text
    val CardBorderLight = Color(0xFFCBD5E1)

    // Default Dynamic Proxies (Default to Dark Theme Tokens for backward compatibility)
    var Background = BackgroundDark
    var Surface = SurfaceDark
    var SurfaceElevated = SurfaceElevatedDark
    var SurfaceHighlight = SurfaceHighlightDark
    var SurfaceMuted = SurfaceDark
    var SurfaceSubtle = SurfaceDark
    var CardBorder = CardBorderDark
    var CardBorderSubtle = Color(0xFF0F273D)
    var CardBorderFocused = Color(0xFF1688FF)
    var TextPrimary = TextPrimaryDark
    var TextSecondary = TextSecondaryDark
    var TextTertiary = Color(0xFF64748B)
    var SlateMuted = Color(0xFF94A3B8)
    var TextOnDark = Color(0xFFFFFFFF)
    var NavyDarkest = Color(0xFF06111F)

    // Primary Brand & Mode Tokens (Consistent across themes)
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
    val StatusGreen = Color(0xFF18D47B)       // Healthy / Active Fix
    val StatusGreenDim = Color(0x2618D47B)
    val StatusGreenText = Color(0xFF18D47B)
    val StatusGreenBg = Color(0x1F18D47B)
    val StatusGreenDark = Color(0xFF18D47B)

    val StatusAmber = Color(0xFFFFB020)       // Searching / Degraded
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

    // Map & Trajectory Visualization Tokens
    val MapCanvasBg = Color(0xFF06111F)
    val MapGridLine = Color(0xFF0B1B2B)
    val MapAxisLine = Color(0xFF10263A)
    val GridLineColor = Color(0xFF0F273D)
    val GridAxisColor = Color(0xFF17324A)
    val MapRoadOutline = Color(0xFF17324A)
    val MapRoadSurface = Color(0xFFE2E8F0)
    val MapSelectedRoad = Color(0xFFFF8A00)
    val MapMatchedCyan = Color(0xFF00E5FF)
    val GroundRoadColor = Color(0xFF64748B)
    val FlyoverRoadColor = Color(0xFFFF8A00)
    val TunnelRoadColor = Color(0xFF9333EA)
    val WaterBodyColor = Color(0xFF0C2B4E)
    val LanduseParkColor = Color(0xFF0D2818)
    val Building3dColor = Color(0xFF1E3A5F)
}

package org.isro.idr.theme

import androidx.compose.ui.graphics.Color

/**
 * Modern Aerospace & Automotive Light-Theme Color Palette for ISRO IDR Navigation Instrument.
 * Crisp clean white/neutral base, deep navy typography, vivid ISRO saffron brand accent,
 * electric navigation blue, and clear multi-state status indicators.
 */
object IdrColors {
    // Primary Backgrounds & Surfaces (Light Theme)
    val Background = Color(0xFFF8FAFC)        // App canvas background (Slate 50)
    val Surface = Color(0xFFFFFFFF)           // Primary card/panel surface
    val SurfaceElevated = Color(0xFFFFFFFF)   // Elevated cards/modals
    val SurfaceMuted = Color(0xFFF1F5F9)      // Secondary neutral background (Slate 100)
    val SurfaceSubtle = Color(0xFFF1F5F9)     // Subtle container surface (Slate 100)
    val CardBorder = Color(0xFFE2E8F0)        // Clean subtle border (Slate 200)
    val CardBorderSubtle = Color(0xFFCBD5E1)  // Card separator (Slate 300)

    // Primary Brand & Navigation Accents
    val SaffronPrimary = Color(0xFFFF6F00)    // Vivid ISRO Saffron Primary
    val SaffronLight = Color(0xFFFF8F00)      // Lighter Saffron
    val SaffronDark = Color(0xFFE65100)       // Deep Saffron for contrast
    val SaffronDim = Color(0xFFFFF3E0)        // Soft Saffron chip background (Orange 50)

    val NavBlue = Color(0xFF0284C7)           // Electric Navigation Blue (Sky 600)
    val NavBlueLight = Color(0xFF38BDF8)      // Bright Blue (Sky 400)
    val NavBlueDim = Color(0xFFE0F2FE)        // Soft Blue chip background (Sky 100)
    val NavBlueDark = Color(0xFF0369A1)       // Deep Blue for text (Sky 700)

    val MapPurple = Color(0xFF7C3AED)         // Map-Aided DR Purple (Violet 600)
    val MapPurpleDim = Color(0xFFEDE9FE)      // Soft Purple chip background (Violet 100)

    // Status Colors (Active, Searching, Degraded, Error, Offline)
    val StatusGreen = Color(0xFF16A34A)       // Active / Fix Locked / Healthy (Green 600)
    val StatusGreenDim = Color(0xFFDCFCE7)    // Soft Green chip background (Green 100)
    val StatusGreenBg = Color(0xFFDCFCE7)     // Background for Green pill
    val StatusGreenText = Color(0xFF15803D)   // High contrast Green text (Green 700)
    val StatusGreenDark = Color(0xFF15803D)   // Deep Green for text

    val StatusAmber = Color(0xFFD97706)       // Searching / Degraded / DR Active (Amber 600)
    val StatusAmberDim = Color(0xFFFEF3C7)    // Soft Amber chip background (Amber 100)
    val StatusAmberBg = Color(0xFFFEF3C7)     // Background for Amber pill
    val StatusAmberText = Color(0xFFB45309)   // High contrast Amber text (Amber 700)
    val StatusAmberDark = Color(0xFFB45309)   // Deep Amber for text

    val StatusOrange = Color(0xFFEA580C)      // Warm Orange for Dead Reckoning (Orange 600)
    val StatusOrangeBg = Color(0xFFFFEDD5)    // Soft Orange chip background (Orange 100)
    val StatusOrangeDark = Color(0xFFC2410C)  // Deep Orange text (Orange 700)

    val StatusBlue = Color(0xFF2563EB)        // Initializing / Info (Blue 600)
    val StatusBlueDim = Color(0xFFDBEAFE)     // Soft Blue chip background (Blue 100)
    val StatusBlueBg = Color(0xFFE0F2FE)      // Soft Sky chip background
    val StatusBlueText = Color(0xFF1D4ED8)    // High contrast Blue text (Blue 700)

    val StatusRed = Color(0xFFDC2626)         // Actual Hardware Error / Out of Bounds (Red 600)
    val StatusRedDim = Color(0xFFFEE2E2)      // Soft Red chip background (Red 100)
    val StatusRedBg = Color(0xFFFEE2E2)       // Soft Red chip background
    val StatusRedText = Color(0xFFB91C1C)     // High contrast Red text (Red 700)

    val StatusNeutral = Color(0xFF64748B)     // Offline / Not Loaded (Slate 500)
    val StatusNeutralDim = Color(0xFFF1F5F9)  // Soft Gray chip background (Slate 100)
    val StatusNeutralText = Color(0xFF475569) // Gray text (Slate 600)

    // Typography & Content
    val TextPrimary = Color(0xFF0F172A)       // Deep Space Navy text (Slate 900)
    val TextSecondary = Color(0xFF475569)     // Muted Slate text (Slate 600)
    val TextTertiary = Color(0xFF94A3B8)      // Hint / Footnote text (Slate 400)
    val SlateMuted = Color(0xFF94A3B8)        // Slate Muted
    val TextOnDark = Color(0xFFFFFFFF)        // High contrast white text for dark badges
    val NavyDarkest = Color(0xFF0F172A)       // Deep Navy Darkest

    // Map & Trajectory Visualization Layers
    val MapCanvasBg = Color(0xFFF1F5F9)       // Map background (Slate 100)
    val MapGridLine = Color(0xFFE2E8F0)       // Map grid lines (Slate 200)
    val MapAxisLine = Color(0xFFCBD5E1)       // Map center axes (Slate 300)
    val GridLineColor = Color(0xFFCBD5E1)     // Tactical grid lines
    val GridAxisColor = Color(0xFF94A3B8)     // Major tactical grid axes
    val MapRoadOutline = Color(0xFF94A3B8)    // Road network outline (Slate 400)
    val MapRoadSurface = Color(0xFFFFFFFF)    // Road network surface
    val MapSelectedRoad = Color(0xFFFFD54F)   // Snapped active road highlight (Amber 300)

    val TrajectoryGnss = Color(0xFF0284C7)    // Electric Blue for raw GNSS track
    val TrajectoryIdr = Color(0xFFFF6F00)     // ISRO Saffron for IDR dead-reckoning track
    val TrajectoryMap = Color(0xFF7C3AED)     // Royal Purple for Map-Aided track
    val TrajectoryMapAided = Color(0xFF7C3AED)// Royal Purple for Map-Aided track
    val VehicleMarker = Color(0xFFFF6F00)     // ISRO Saffron directional vehicle indicator
    val VehicleOutline = Color(0xFF0F172A)    // Deep navy chevron outline
    val VehicleHeadingCone = Color(0x33FF6F00)// Translucent heading cone

    // Legacy Aliases for backwards-compatibility
    val NavyDark = Surface
    val NavySurface = Surface
    val NavyCard = Surface
    val NavyCardBorder = CardBorder
    val CyanAccent = NavBlue
    val SaffronAccent = SaffronPrimary
}

package org.isro.idr.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable

private val LightColorScheme = lightColorScheme(
    primary = IdrColors.SaffronPrimary,
    onPrimary = IdrColors.TextOnDark,
    primaryContainer = IdrColors.SaffronDim,
    onPrimaryContainer = IdrColors.SaffronDark,
    secondary = IdrColors.NavBlue,
    onSecondary = IdrColors.TextOnDark,
    secondaryContainer = IdrColors.NavBlueDim,
    onSecondaryContainer = IdrColors.NavBlueDark,
    tertiary = IdrColors.MapPurple,
    onTertiary = IdrColors.TextOnDark,
    tertiaryContainer = IdrColors.MapPurpleDim,
    onTertiaryContainer = IdrColors.MapPurple,
    background = IdrColors.Background,
    onBackground = IdrColors.TextPrimary,
    surface = IdrColors.Surface,
    onSurface = IdrColors.TextPrimary,
    surfaceVariant = IdrColors.SurfaceMuted,
    onSurfaceVariant = IdrColors.TextSecondary,
    outline = IdrColors.CardBorder,
    error = IdrColors.StatusRed,
    onError = IdrColors.TextOnDark,
    errorContainer = IdrColors.StatusRedDim,
    onErrorContainer = IdrColors.StatusRedText
)

@Composable
fun IdrNavigationTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = LightColorScheme,
        typography = IdrTypography,
        content = content
    )
}

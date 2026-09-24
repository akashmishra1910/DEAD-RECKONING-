package org.isro.idr.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.SideEffect

private val DarkColorScheme = darkColorScheme(
    primary = IdrColors.SaffronPrimary,
    onPrimary = IdrColors.TextOnDark,
    primaryContainer = IdrColors.SaffronDim,
    onPrimaryContainer = IdrColors.SaffronLight,
    secondary = IdrColors.NavBlue,
    onSecondary = IdrColors.TextOnDark,
    secondaryContainer = IdrColors.NavBlueDim,
    onSecondaryContainer = IdrColors.NavBlueLight,
    tertiary = IdrColors.MapPurple,
    onTertiary = IdrColors.TextOnDark,
    background = IdrColors.BackgroundDark,
    onBackground = IdrColors.TextPrimaryDark,
    surface = IdrColors.SurfaceDark,
    onSurface = IdrColors.TextPrimaryDark,
    surfaceVariant = IdrColors.SurfaceHighlightDark,
    onSurfaceVariant = IdrColors.TextSecondaryDark,
    outline = IdrColors.CardBorderDark,
    error = IdrColors.StatusRed,
    onError = IdrColors.TextOnDark
)

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
    background = IdrColors.BackgroundLight,
    onBackground = IdrColors.TextPrimaryLight,
    surface = IdrColors.SurfaceLight,
    onSurface = IdrColors.TextPrimaryLight,
    surfaceVariant = IdrColors.SurfaceElevatedLight,
    onSurfaceVariant = IdrColors.TextSecondaryLight,
    outline = IdrColors.CardBorderLight,
    error = IdrColors.StatusRed,
    onError = IdrColors.TextOnDark
)

@Composable
fun IdrNavigationTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    content: @Composable () -> Unit
) {
    val colorScheme = if (darkTheme) DarkColorScheme else LightColorScheme

    SideEffect {
        if (darkTheme) {
            IdrColors.Background = IdrColors.BackgroundDark
            IdrColors.Surface = IdrColors.SurfaceDark
            IdrColors.SurfaceElevated = IdrColors.SurfaceElevatedDark
            IdrColors.SurfaceHighlight = IdrColors.SurfaceHighlightDark
            IdrColors.SurfaceMuted = IdrColors.SurfaceDark
            IdrColors.SurfaceSubtle = IdrColors.SurfaceDark
            IdrColors.CardBorder = IdrColors.CardBorderDark
            IdrColors.TextPrimary = IdrColors.TextPrimaryDark
            IdrColors.TextSecondary = IdrColors.TextSecondaryDark
        } else {
            IdrColors.Background = IdrColors.BackgroundLight
            IdrColors.Surface = IdrColors.SurfaceLight
            IdrColors.SurfaceElevated = IdrColors.SurfaceElevatedLight
            IdrColors.SurfaceHighlight = IdrColors.SurfaceHighlightLight
            IdrColors.SurfaceMuted = IdrColors.SurfaceLight
            IdrColors.SurfaceSubtle = IdrColors.SurfaceElevatedLight
            IdrColors.CardBorder = IdrColors.CardBorderLight
            IdrColors.TextPrimary = IdrColors.TextPrimaryLight
            IdrColors.TextSecondary = IdrColors.TextSecondaryLight
        }
    }

    MaterialTheme(
        colorScheme = colorScheme,
        typography = IdrTypography,
        content = content
    )
}

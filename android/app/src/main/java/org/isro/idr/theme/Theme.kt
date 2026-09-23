package org.isro.idr.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable

private val DarkColorScheme = darkColorScheme(
    primary = IdrColors.SaffronAccent,
    onPrimary = IdrColors.NavyDarkest,
    primaryContainer = IdrColors.NavySurface,
    onPrimaryContainer = IdrColors.SaffronLight,
    secondary = IdrColors.CyanAccent,
    onSecondary = IdrColors.NavyDarkest,
    background = IdrColors.NavyDarkest,
    onBackground = IdrColors.TextPrimary,
    surface = IdrColors.NavyDark,
    onSurface = IdrColors.TextPrimary,
    surfaceVariant = IdrColors.NavyCard,
    onSurfaceVariant = IdrColors.TextSecondary,
    error = IdrColors.StatusRed,
    onError = IdrColors.TextPrimary
)

@Composable
fun IdrNavigationTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = DarkColorScheme,
        typography = IdrTypography,
        content = content
    )
}

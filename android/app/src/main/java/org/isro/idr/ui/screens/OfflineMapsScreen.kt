package org.isro.idr.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.isro.idr.theme.IdrColors
import org.isro.idr.theme.IdrTypography
import org.isro.idr.ui.state.NavigationUiState
import org.isro.idr.ui.state.OfflineRegionPack

/**
 * Screen 10: Offline World Maps.
 * Multi-scale regional and global vector pack manager, storage capacity indicator,
 * and offline dataset downloader.
 */
@Composable
fun OfflineMapsScreen(
    uiState: NavigationUiState,
    onDownloadRegion: (String) -> Unit,
    onActivateRegion: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    var selectedCategory by remember { mutableStateOf("India") }

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(IdrColors.Background)
            .padding(14.dp),
        verticalArrangement = Arrangement.spacedBy(14.dp)
    ) {
        // Header
        Text(
            text = "Offline Maps",
            style = IdrTypography.titleLarge.copy(fontSize = 18.sp, fontWeight = FontWeight.Bold),
            color = Color.White
        )

        // Storage Usage Panel
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(8.dp, RoundedCornerShape(14.dp))
                .clip(RoundedCornerShape(14.dp))
                .background(IdrColors.Surface)
                .border(1.2.dp, IdrColors.CardBorder, RoundedCornerShape(14.dp))
                .padding(14.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "Storage Used",
                    style = IdrTypography.labelMedium.copy(fontSize = 12.sp, fontWeight = FontWeight.Bold),
                    color = Color.White
                )
                Text(
                    text = "1.74 GB / 64 GB",
                    style = IdrTypography.labelSmall.copy(fontSize = 11.sp),
                    color = Color(0xFF00B8FF)
                )
            }

            LinearProgressIndicator(
                progress = { 0.08f },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(5.dp)
                    .clip(RoundedCornerShape(2.5.dp)),
                color = Color(0xFF1688FF),
                trackColor = Color(0xFF10263A)
            )
        }

        // Category Tab Switcher (India, My Regions, Global)
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(10.dp))
                .background(Color(0xFF0B1B2B))
                .padding(3.dp),
            horizontalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            CategoryTabChip("India", selectedCategory == "India", modifier = Modifier.weight(1f)) { selectedCategory = "India" }
            CategoryTabChip("My Regions", selectedCategory == "My Regions", modifier = Modifier.weight(1f)) { selectedCategory = "My Regions" }
            CategoryTabChip("Global", selectedCategory == "Global", modifier = Modifier.weight(1f)) { selectedCategory = "Global" }
        }

        // Region Pack List
        val filteredPacks = uiState.offlineRegionPacks.filter {
            if (selectedCategory == "My Regions") it.isInstalled else it.category == selectedCategory
        }

        LazyColumn(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
            verticalArrangement = Arrangement.spacedBy(10.dp)
        ) {
            items(filteredPacks) { pack ->
                RegionPackCard(
                    pack = pack,
                    onDownload = { onDownloadRegion(pack.id) },
                    onActivate = { onActivateRegion(pack.id) }
                )
            }

            // Custom Region Selector CTA Card
            item {
                CustomRegionCard()
            }
        }
    }
}

@Composable
private fun CategoryTabChip(
    title: String,
    isSelected: Boolean,
    modifier: Modifier = Modifier,
    onClick: () -> Unit
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(if (isSelected) Color(0xFF1688FF) else Color.Transparent)
            .clickable { onClick() }
            .padding(vertical = 7.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = title,
            style = IdrTypography.labelMedium.copy(
                fontSize = 11.sp,
                fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Medium
            ),
            color = if (isSelected) Color.White else Color(0xFF94A3B8)
        )
    }
}

@Composable
private fun RegionPackCard(
    pack: OfflineRegionPack,
    onDownload: () -> Unit,
    onActivate: () -> Unit
) {
    val sizeMb = pack.sizeBytes / (1024 * 1024)
    val sizeStr = if (sizeMb >= 1024) "${String.format("%.1f", sizeMb / 1024.0)} GB" else "$sizeMb MB"

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(6.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.Surface)
            .border(1.dp, if (pack.isInstalled) Color(0xFF18D47B).copy(alpha = 0.5f) else IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        // City / Region Thumbnail Icon
        Box(
            modifier = Modifier
                .size(46.dp)
                .clip(RoundedCornerShape(8.dp))
                .background(Color(0xFF10263A))
                .border(1.dp, Color(0xFF17324A), RoundedCornerShape(8.dp)),
            contentAlignment = Alignment.Center
        ) {
            Text(text = "🗺", fontSize = 18.sp)
        }

        Spacer(modifier = Modifier.width(12.dp))

        Column(modifier = Modifier.weight(1f)) {
            Text(
                text = pack.name,
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )
            Text(
                text = "$sizeStr • ${pack.version}",
                style = IdrTypography.bodySmall.copy(fontSize = 10.sp),
                color = Color(0xFF94A3B8)
            )
            Text(
                text = pack.subtitle,
                style = IdrTypography.labelSmall.copy(fontSize = 9.sp),
                color = Color(0xFF64748B)
            )
        }

        Spacer(modifier = Modifier.width(8.dp))

        if (pack.isInstalled) {
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(16.dp))
                    .background(Color(0xFF18D47B).copy(alpha = 0.18f))
                    .border(1.dp, Color(0xFF18D47B), RoundedCornerShape(16.dp))
                    .clickable { onActivate() }
                    .padding(horizontal = 10.dp, vertical = 5.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "ACTIVE",
                    style = IdrTypography.labelSmall.copy(fontSize = 10.sp, fontWeight = FontWeight.Bold),
                    color = Color(0xFF18D47B)
                )
            }
        } else {
            Box(
                modifier = Modifier
                    .size(34.dp)
                    .clip(CircleShape)
                    .background(Color(0xFF1688FF))
                    .clickable { onDownload() },
                contentAlignment = Alignment.Center
            ) {
                Text(text = "↓", color = Color.White, fontSize = 16.sp, fontWeight = FontWeight.Bold)
            }
        }
    }
}

@Composable
private fun CustomRegionCard() {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(4.dp, RoundedCornerShape(12.dp))
            .clip(RoundedCornerShape(12.dp))
            .background(IdrColors.Surface)
            .border(1.dp, IdrColors.CardBorder, RoundedCornerShape(12.dp))
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(
            modifier = Modifier
                .size(46.dp)
                .clip(RoundedCornerShape(8.dp))
                .background(Color(0xFF10263A)),
            contentAlignment = Alignment.Center
        ) {
            Text(text = "⌖", fontSize = 20.sp, color = Color(0xFF1688FF))
        }

        Spacer(modifier = Modifier.width(12.dp))

        Column(modifier = Modifier.weight(1f)) {
            Text(
                text = "Custom Region",
                style = IdrTypography.titleMedium.copy(fontSize = 13.sp, fontWeight = FontWeight.Bold),
                color = Color.White
            )
            Text(
                text = "Select area on map for offline vector download",
                style = IdrTypography.bodySmall.copy(fontSize = 10.sp),
                color = Color(0xFF94A3B8)
            )
        }

        Box(
            modifier = Modifier
                .size(34.dp)
                .clip(CircleShape)
                .background(Color(0xFF10263A))
                .border(1.dp, IdrColors.CardBorder, CircleShape),
            contentAlignment = Alignment.Center
        ) {
            Text(text = "+", color = Color.White, fontSize = 16.sp, fontWeight = FontWeight.Bold)
        }
    }
}

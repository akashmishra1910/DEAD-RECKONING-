#!/usr/bin/env bash
# ==============================================================================
# scripts/build_macos_app.sh
# ==============================================================================
# Builds the reproducible macOS desktop application bundle: ISRO-IDR.app
# Wraps the Edge Navigation Engine and Dashboard launcher in a standard double-clickable
# macOS application bundle without requiring users to use terminal or command line.
#
# Usage:
#     ./scripts/build_macos_app.sh
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
APP_NAME="ISRO-IDR.app"
APP_BUNDLE="$REPO_ROOT/$APP_NAME"

echo "================================================================="
echo "Building macOS Desktop Application Bundle: $APP_NAME"
echo "Repository Root: $REPO_ROOT"
echo "================================================================="

# 1. Clean previous bundle
rm -rf "$APP_BUNDLE"

# 2. Create bundle directory hierarchy
mkdir -p "$APP_BUNDLE/Contents/MacOS"
mkdir -p "$APP_BUNDLE/Contents/Resources"

# 3. Create Info.plist
cat << 'EOF' > "$APP_BUNDLE/Contents/Info.plist"
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleDevelopmentRegion</key>
    <string>en</string>
    <key>CFBundleExecutable</key>
    <string>ISRO-IDR</string>
    <key>CFBundleIconFile</key>
    <string>AppIcon</string>
    <key>CFBundleIdentifier</key>
    <string>org.isro.idr.edge.dashboard</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundleName</key>
    <string>ISRO IDR</string>
    <key>CFBundleDisplayName</key>
    <string>ISRO IDR Edge Navigation</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleShortVersionString</key>
    <string>1.0.0</string>
    <key>CFBundleSignature</key>
    <string>????</string>
    <key>CFBundleVersion</key>
    <string>1.0.0</string>
    <key>LSMinimumSystemVersion</key>
    <string>10.15</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>NSHumanReadableCopyright</key>
    <string>ISRO / SIH PS 26168 — Intelligent Dead Reckoning System</string>
</dict>
</plist>
EOF

# 4. Create PkgInfo
echo -n "APPL????" > "$APP_BUNDLE/Contents/PkgInfo"

# 5. Create the launch executable script inside Contents/MacOS
cat << 'EOF' > "$APP_BUNDLE/Contents/MacOS/ISRO-IDR"
#!/usr/bin/env bash
# ==============================================================================
# ISRO-IDR macOS Application Launcher Executable
# ==============================================================================

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# In standard macOS bundle, DIR is ISRO-IDR.app/Contents/MacOS
APP_DIR="$( cd "$DIR/../../.." && pwd )"

cd "$APP_DIR" || exit 1

# Locate Python 3 interpreter
PYTHON_BIN=""
for candidate in \
    "$(which python3 2>/dev/null || true)" \
    "/Library/Frameworks/Python.framework/Versions/3.14/bin/python3" \
    "/Library/Frameworks/Python.framework/Versions/3.12/bin/python3" \
    "/Library/Frameworks/Python.framework/Versions/3.11/bin/python3" \
    "/opt/homebrew/bin/python3" \
    "/usr/local/bin/python3" \
    "/usr/bin/python3"; do
    if [ -n "$candidate" ] && [ -x "$candidate" ]; then
        PYTHON_BIN="$candidate"
        break
    fi
done

if [ -z "$PYTHON_BIN" ]; then
    osascript -e 'display alert "ISRO IDR Launch Error" message "Python 3 is required to launch the ISRO IDR Edge Engine Dashboard. Please install Python 3."'
    exit 1
fi

# Launch the desktop orchestrator
exec "$PYTHON_BIN" launch_idr_app.py "$@"
EOF

chmod +x "$APP_BUNDLE/Contents/MacOS/ISRO-IDR"

# 6. Generate application icon
ICON_DIR="$APP_BUNDLE/Contents/Resources"
python3 -c "
import os, subprocess
try:
    from PIL import Image, ImageDraw
    img = Image.new('RGBA', (512, 512), color=(255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    # Outer aerospace disc
    draw.ellipse([32, 32, 480, 480], fill=(10, 37, 64, 255), outline=(2, 132, 199, 255), width=16)
    # Rocket / navigation chevron
    draw.polygon([(256, 100), (380, 380), (256, 320), (132, 380)], fill=(2, 132, 199, 255))
    draw.ellipse([232, 232, 280, 280], fill=(245, 158, 11, 255))
    src_png = '$ICON_DIR/AppIcon.png'
    img.save(src_png)
    
    # Generate ICNS if iconutil and sips available
    iconset = '$ICON_DIR/AppIcon.iconset'
    os.makedirs(iconset, exist_ok=True)
    for sz in [16, 32, 64, 128, 256]:
        subprocess.run(['sips', '-z', str(sz), str(sz), src_png, '--out', f'{iconset}/icon_{sz}x{sz}.png'], capture_output=True)
        subprocess.run(['sips', '-z', str(sz*2), str(sz*2), src_png, '--out', f'{iconset}/icon_{sz}x{sz}@2x.png'], capture_output=True)
    subprocess.run(['iconutil', '-c', 'icns', iconset, '-o', '$ICON_DIR/AppIcon.icns'], capture_output=True)
    import shutil
    shutil.rmtree(iconset, ignore_errors=True)
    print('Generated AppIcon.icns')
except Exception as e:
    print(f'Icon generation notice: {e}')
" || true

echo "================================================================="
echo "Build Successful: $APP_BUNDLE"
echo "Launch with: open $APP_NAME   or double-click in Finder"
echo "================================================================="

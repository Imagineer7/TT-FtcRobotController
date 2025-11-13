#!/bin/bash
# Pedro Pathing Quick Setup Script
# Run this script to download and copy Pedro Pathing support files

set -e

echo "╔════════════════════════════════════════════════════════╗"
echo "║   Pedro Pathing 2.0.4 Installation Script             ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

PROJECT_DIR="/home/roguebit/StudioProjects/TT-FtcRobotController"
TEAMCODE_DIR="$PROJECT_DIR/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
TEMP_DIR="/tmp/pedro-install-$$"

# Check if we're in the right directory
if [ ! -f "$PROJECT_DIR/build.dependencies.gradle" ]; then
    echo "❌ Error: Cannot find project directory"
    echo "   Expected: $PROJECT_DIR"
    exit 1
fi

echo "📦 Step 1: Creating temporary directory..."
mkdir -p "$TEMP_DIR"
cd "$TEMP_DIR"

echo "📥 Step 2: Cloning Pedro Pathing Quickstart repository..."
if git clone --depth 1 --quiet https://github.com/Pedro-Pathing/Quickstart.git; then
    echo "   ✓ Repository cloned successfully"
else
    echo "   ❌ Failed to clone repository"
    echo "   Please check your internet connection and try again"
    exit 1
fi

echo "📂 Step 3: Creating pedroPathing directory..."
mkdir -p "$TEAMCODE_DIR/pedroPathing"

echo "📋 Step 4: Copying pedroPathing files..."
if cp -r Quickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/* "$TEAMCODE_DIR/pedroPathing/" 2>/dev/null; then
    echo "   ✓ Files copied successfully"
else
    echo "   ❌ Failed to copy files"
    echo "   Source: Quickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/"
    echo "   Target: $TEAMCODE_DIR/pedroPathing/"
    exit 1
fi

echo ""
echo "📁 Step 5: Verifying installation..."
cd "$TEAMCODE_DIR/pedroPathing"
FILE_COUNT=$(find . -type f -name "*.java" | wc -l)
DIR_COUNT=$(find . -type d | wc -l)

echo "   Found:"
echo "   • $FILE_COUNT Java files"
echo "   • $DIR_COUNT directories"
echo ""

# List main directories
echo "   Main directories:"
for dir in */; do
    if [ -d "$dir" ]; then
        count=$(find "$dir" -type f -name "*.java" | wc -l)
        echo "   • ${dir%/}/ ($count files)"
    fi
done

echo ""
echo "🧹 Step 6: Cleaning up temporary files..."
rm -rf "$TEMP_DIR"

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║            ✅ Installation Complete!                   ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
echo "Next steps:"
echo "1. Open Android Studio and sync Gradle"
echo "2. Configure Constants.java for your robot"
echo "3. Review the PEDRO_PATHING_INSTALLATION.md guide"
echo ""
echo "Key files to configure:"
echo "• pedroPathing/Constants.java - Robot configuration"
echo "• pedroPathing/localization/ - Choose your localizer"
echo ""
echo "Documentation: https://pedropathing.com/docs/pathing"
echo "Support: https://discord.gg/2GfC4qBP5s"
echo ""


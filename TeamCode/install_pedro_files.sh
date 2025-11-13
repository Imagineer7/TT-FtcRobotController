#!/bin/bash
# Script to install Pedro Pathing support files

echo "======================================"
echo "Pedro Pathing Installation Script"
echo "======================================"
echo ""

# Create temp directory
TEMP_DIR="/tmp/pedro-install-$$"
mkdir -p "$TEMP_DIR"

echo "Step 1: Cloning Pedro Pathing Quickstart..."
cd "$TEMP_DIR"
git clone --depth 1 https://github.com/Pedro-Pathing/Quickstart.git

if [ ! -d "Quickstart" ]; then
    echo "ERROR: Failed to clone repository"
    exit 1
fi

echo "Step 2: Copying pedroPathing files to your project..."
TARGET_DIR="/home/roguebit/StudioProjects/TT-FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"

# Create pedroPathing directory if it doesn't exist
mkdir -p "$TARGET_DIR/pedroPathing"

# Copy the pedroPathing folder
cp -r Quickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/* \
   "$TARGET_DIR/pedroPathing/"

if [ $? -eq 0 ]; then
    echo "✓ Successfully copied pedroPathing files"
else
    echo "ERROR: Failed to copy files"
    exit 1
fi

echo ""
echo "Step 3: Listing copied files..."
ls -la "$TARGET_DIR/pedroPathing/"

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo "1. Open Android Studio and sync Gradle files"
echo "2. Configure Constants.java for your robot"
echo "3. Check the PEDRO_PATHING_INSTALLATION.md guide"
echo ""
echo "Cleaning up temporary files..."
rm -rf "$TEMP_DIR"

echo "Done!"


#!/bin/bash
# Fusion Localization System - Build Verification Script
# 
# This script checks for common issues that prevent compilation
# and provides specific guidance for fixing them.

echo "=========================================="
echo "Fusion Localization Build Verification"
echo "=========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ERRORS=0

# Check 1: Verify we're in the right directory
echo "Check 1: Project directory..."
if [ ! -f "build.gradle" ]; then
    echo -e "${RED}✗ FAIL${NC}: Not in project root directory"
    echo "  Please run this script from the TT-FtcRobotController root directory"
    exit 1
fi
echo -e "${GREEN}✓ PASS${NC}"
echo ""

# Check 2: Verify git branch
echo "Check 2: Git branch..."
BRANCH=$(git branch --show-current 2>/dev/null)
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}: Not in a git repository or git not available"
else
    echo "  Current branch: $BRANCH"
    if [ "$BRANCH" != "copilot/add-localization-system" ]; then
        echo -e "${YELLOW}⚠ WARNING${NC}: You may not be on the correct branch"
        echo "  Expected: copilot/add-localization-system"
        echo "  Current:  $BRANCH"
        ERRORS=$((ERRORS + 1))
    else
        echo -e "${GREEN}✓ PASS${NC}"
    fi
fi
echo ""

# Check 3: Verify LimelightVisionHelper exists
echo "Check 3: LimelightVisionHelper.java..."
FILE="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/LimelightVisionHelper.java"
if [ ! -f "$FILE" ]; then
    echo -e "${RED}✗ FAIL${NC}: File not found"
    echo "  Missing: $FILE"
    echo "  Solution: git pull origin copilot/add-localization-system"
    ERRORS=$((ERRORS + 1))
else
    echo -e "${GREEN}✓ PASS${NC}: File exists"
    
    # Verify package declaration
    PACKAGE=$(head -1 "$FILE" | grep "package org.firstinspires.ftc.teamcode.util.aurora.localization")
    if [ -z "$PACKAGE" ]; then
        echo -e "${RED}✗ FAIL${NC}: Package declaration incorrect"
        ERRORS=$((ERRORS + 1))
    else
        echo -e "${GREEN}✓ PASS${NC}: Package declaration correct"
    fi
fi
echo ""

# Check 4: Verify localization package exists
echo "Check 4: Localization package..."
DIR="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization"
if [ ! -d "$DIR" ]; then
    echo -e "${RED}✗ FAIL${NC}: Localization package not found"
    echo "  Missing: $DIR"
    echo "  Solution: git pull origin copilot/add-localization-system"
    ERRORS=$((ERRORS + 1))
else
    echo -e "${GREEN}✓ PASS${NC}: Directory exists"
    
    # Count files
    FILE_COUNT=$(ls -1 "$DIR"/*.java 2>/dev/null | wc -l)
    echo "  Found $FILE_COUNT Java files"
    
    if [ $FILE_COUNT -lt 9 ]; then
        echo -e "${YELLOW}⚠ WARNING${NC}: Expected at least 9 Java files"
        ERRORS=$((ERRORS + 1))
    fi
fi
echo ""

# Check 5: Verify imports in localization files
echo "Check 5: Import statements..."
IMPORT_ERRORS=0
for file in "$DIR"/*.java; do
    if [ -f "$file" ]; then
        IMPORTS=$(grep "import org.firstinspires.ftc.teamcode.util.aurora.LimelightVisionHelper" "$file" 2>/dev/null)
        if [ ! -z "$IMPORTS" ]; then
            filename=$(basename "$file")
            echo "  Checking $filename..."
            # This file imports LimelightVisionHelper, verify it's correct
            BAD_IMPORT=$(echo "$IMPORTS" | grep -v "import org.firstinspires.ftc.teamcode.util.aurora.LimelightVisionHelper;")
            if [ ! -z "$BAD_IMPORT" ]; then
                echo -e "${RED}✗ FAIL${NC}: Incorrect import in $filename"
                echo "$BAD_IMPORT"
                IMPORT_ERRORS=$((IMPORT_ERRORS + 1))
            fi
        fi
    fi
done

if [ $IMPORT_ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ PASS${NC}: All imports correct"
else
    echo -e "${RED}✗ FAIL${NC}: Found $IMPORT_ERRORS import error(s)"
    ERRORS=$((ERRORS + IMPORT_ERRORS))
fi
echo ""

# Check 6: Try to build
echo "Check 6: Gradle build test..."
echo "  Running: ./gradlew :TeamCode:compileDebugJavaWithJavac"
echo "  This may take a minute..."

BUILD_OUTPUT=$(./gradlew :TeamCode:compileDebugJavaWithJavac 2>&1)
BUILD_STATUS=$?

if [ $BUILD_STATUS -eq 0 ]; then
    echo -e "${GREEN}✓ PASS${NC}: Build successful"
else
    echo -e "${RED}✗ FAIL${NC}: Build failed"
    echo ""
    echo "Build errors:"
    echo "$BUILD_OUTPUT" | grep "error:" | head -10
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Summary
echo "=========================================="
echo "Summary"
echo "=========================================="

if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ ALL CHECKS PASSED${NC}"
    echo ""
    echo "Your environment is correctly configured."
    echo "The fusion localization system should compile successfully."
else
    echo -e "${RED}✗ FOUND $ERRORS ISSUE(S)${NC}"
    echo ""
    echo "Recommended actions:"
    echo "1. Run: git fetch origin"
    echo "2. Run: git checkout copilot/add-localization-system"
    echo "3. Run: git pull origin copilot/add-localization-system"
    echo "4. Run: ./gradlew clean"
    echo "5. Run: ./gradlew :TeamCode:compileDebugJavaWithJavac"
    echo ""
    echo "If issues persist, see TROUBLESHOOTING.md in the localization package."
fi

exit $ERRORS

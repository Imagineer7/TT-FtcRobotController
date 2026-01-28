# Troubleshooting Build Issues

## Compilation Error: "cannot find symbol" for LimelightVisionHelper

If you encounter this error:
```
error: cannot find symbol
// LimelightVisionHelper is now in the same package (localization)
```

### Quick Fixes

#### 1. Sync with Repository
```bash
git pull origin copilot/add-localization-system
```

#### 2. Clean and Rebuild
```bash
./gradlew clean
./gradlew :TeamCode:compileDebugJavaWithJavac
```

#### 3. Android Studio Cache Issues
In Android Studio:
- File → Invalidate Caches / Restart
- Choose "Invalidate and Restart"
- Wait for indexing to complete

#### 4. Verify File Exists
Check that this file exists in your project:
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/LimelightVisionHelper.java
```

If missing, ensure you've pulled the latest changes from the branch.

#### 5. Gradle Sync
In Android Studio:
- File → Sync Project with Gradle Files
- Wait for sync to complete

### Verify Build
After trying the fixes above, verify the build:
```bash
./gradlew :TeamCode:compileDebugJavaWithJavac
```

Expected output:
```
BUILD SUCCESSFUL
```

### Still Having Issues?

1. **Check your branch:**
   ```bash
   git branch
   ```
   Ensure you're on `copilot/add-localization-system`

2. **Check for uncommitted changes:**
   ```bash
   git status
   ```

3. **Compare with remote:**
   ```bash
   git fetch origin
   git log HEAD..origin/copilot/add-localization-system
   ```

4. **Full reset (CAUTION - loses local changes):**
   ```bash
   git fetch origin
   git reset --hard origin/copilot/add-localization-system
   ./gradlew clean build
   ```

### Files That Should Exist

All these files should be present in your project:

**Core Localization Package:**
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/FusionLocalizer.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/StateEstimator.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/VisionCorrector.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/MeasurementValidator.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/RobotPose2D.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/PoseHistory.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/Matrix3x3.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/LocalizationConfig.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/PredefinedPoses.java`

**Dependencies (Should Already Exist):**
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/LimelightVisionHelper.java` ← **This is the missing file**
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/AuroraHardwareConfig.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/tool/GoBildaPinpointDriver.java`

**Test OpMode:**
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/FusionLocalizationTestOpMode.java`

### Technical Details

The `LimelightVisionHelper` class is located in:
```
package org.firstinspires.ftc.teamcode.util.aurora;
```

And is imported by the localization classes using:
```java
// LimelightVisionHelper is now in the same package (localization)
```

This is the correct import path. The error typically indicates the file is missing from your local workspace.

### Verification Command

Run this to check if the file exists:
```bash
ls -l TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/localization/LimelightVisionHelper.java
```

If you get "No such file or directory", you need to pull the latest code from the repository.

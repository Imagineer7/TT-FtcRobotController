# Installation Instructions

## ⚠️ Important: FtcRobotController Module Required

This starter kit requires the **FtcRobotController** module from the official FTC SDK. This module contains the base Robot Controller app that your code runs on top of.

---

## 📦 Two Installation Options

### Option 1: Use Official FTC SDK Template (Recommended for Beginners)

The easiest way is to start with the official FTC SDK and copy this starter kit's code into it.

**Steps:**

1. **Clone the official FTC SDK repository:**
   ```bash
   git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git
   cd FtcRobotController
   ```

2. **Copy this starter kit's TeamCode:**
   ```bash
   # Delete the default TeamCode
   rm -rf TeamCode/src/main/java/org/firstinspires/ftc/teamcode/*
   
   # Copy starter kit code
   cp -r /path/to/StarterKit/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/* \
         TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
   ```

3. **Update TeamCode/build.gradle:**
   - Replace the entire file with the one from this starter kit
   - This adds Sloth framework support

4. **Copy documentation:**
   ```bash
   cp -r /path/to/StarterKit/docs ./
   cp /path/to/StarterKit/README.md ./STARTER_KIT_README.md
   ```

5. **Open in Android Studio and build:**
   ```bash
   ./gradlew build
   ```

---

### Option 2: Add FtcRobotController Module to Starter Kit (Advanced)

If you want to use this starter kit as the base project, you need to add the FtcRobotController module.

**Steps:**

1. **Clone this starter kit:**
   ```bash
   git clone <your-starter-kit-repo>
   cd StarterKit
   ```

2. **Add FtcRobotController as a submodule:**
   ```bash
   # Option A: Clone from official FTC SDK
   git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git temp_ftc
   cp -r temp_ftc/FtcRobotController ./
   rm -rf temp_ftc
   
   # Option B: Copy from existing FTC SDK installation
   cp -r /path/to/existing/FtcRobotController ./
   ```

3. **Verify settings.gradle includes both modules:**
   ```gradle
   include ':FtcRobotController'
   include ':TeamCode'
   ```

4. **Build to verify:**
   ```bash
   ./gradlew build
   ```

---

## 🔍 Verify Installation

After setup, your directory structure should look like:

```
YourProject/
├── FtcRobotController/          # FTC SDK base module (required!)
│   ├── build.gradle
│   └── src/
├── TeamCode/                    # Your robot code (starter kit)
│   ├── build.gradle
│   └── src/main/java/.../teamcode/
│       ├── core/
│       ├── teleop/
│       └── auto/
├── docs/
├── build.gradle
├── settings.gradle
└── README.md
```

**Key point:** The `FtcRobotController` module MUST be present for the project to build. It provides the Robot Controller app infrastructure.

---

## 🚀 First Build

After installation:

1. **Open in Android Studio:**
   - File → Open → Select project root directory
   - Wait for Gradle sync (2-5 minutes first time)

2. **Build the project:**
   ```bash
   ./gradlew assembleDebug
   ```

3. **Check for errors:**
   - If successful, you'll see: `BUILD SUCCESSFUL`
   - If errors occur, see troubleshooting below

---

## 🐛 Troubleshooting

### Error: "Project 'FtcRobotController' not found"

**Problem:** The FtcRobotController module is missing.

**Solution:** Follow Option 1 or Option 2 above to add the module.

---

### Error: "Could not resolve dependencies"

**Problem:** Gradle can't download FTC SDK libraries.

**Solution:**
1. Check internet connection
2. Verify repositories in `build.dependencies.gradle`:
   ```gradle
   repositories {
       mavenCentral()
       google()
   }
   ```
3. Try: `./gradlew clean build --refresh-dependencies`

---

### Error: "Duplicate class found"

**Problem:** TeamCode and FtcRobotController have conflicting files.

**Solution:**
1. Don't copy example files from FTC SDK into TeamCode
2. Keep TeamCode focused on your robot code only
3. Remove any duplicate classes

---

### Build is very slow (first time)

**Expected:** First build downloads ~500 MB of dependencies:
- Android SDK components
- FTC SDK libraries  
- Sloth framework
- Build tools

**Solution:** Be patient. Subsequent builds are much faster (30-60 seconds).

---

## 📚 Next Steps

Once installation is complete:

1. Read [README.md](README.md) for project overview
2. Follow [Quick Start guide](README.md#-quick-start-first-time-setup)
3. Review [docs/ADB-Setup-Guide.md](docs/ADB-Setup-Guide.md) for deployment
4. Test on robot hardware!

---

## 💡 Tips

1. **Use Git:** Track your changes with version control
   ```bash
   git init
   git add .
   git commit -m "Initial commit with starter kit"
   ```

2. **Backup regularly:** Before competitions, tag working versions
   ```bash
   git tag -a v1.0-competition -m "Code for first competition"
   ```

3. **Keep SDK updated:** At start of season, update to latest FTC SDK
   ```bash
   cd FtcRobotController
   git pull
   # Then rebuild your project
   ```

4. **Don't modify FtcRobotController:** Keep all your code in TeamCode module

---

## 🆘 Still Having Issues?

- Check [docs/ADB-Setup-Guide.md](docs/ADB-Setup-Guide.md) for Android Studio help
- Search FTC community forums: https://ftcforum.firstinspires.org/
- Ask in FTC Discord: #programming channel
- Review official FTC docs: https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki

---

**Ready to code?** Head back to [README.md](README.md) for the quick start guide! 🚀

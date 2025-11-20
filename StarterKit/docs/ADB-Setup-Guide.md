# Android Debug Bridge (ADB) Setup Guide

This guide walks you through setting up **Android Debug Bridge (ADB)** and **Android Studio** to deploy your robot code to the FTC Robot Controller.

---

## 🎯 What is ADB?

**Android Debug Bridge (ADB)** is a command-line tool that lets you communicate with Android devices (including FTC Robot Controllers).

**What you can do with ADB:**
- Install apps (your robot code)
- View logs (debug crashes)
- Transfer files
- Run shell commands
- Restart the device

**Good news:** Android Studio includes ADB and provides a friendly GUI for most tasks. You rarely need to use command-line ADB unless debugging crashes.

---

## 📱 Prepare Your Robot Controller

### Option 1: REV Control Hub (Recommended)

The REV Control Hub is already configured for ADB by default!

**Steps:**
1. Power on the Control Hub
2. Connect to its Wi-Fi network: `ControlHub-XXXX`
3. That's it! Android Studio can deploy wirelessly.

**Wireless ADB (Control Hub):**
- IP Address: `192.168.43.1` (standard)
- Port: `5555` (default)
- No USB cable needed!

---

### Option 2: Android Phone as Robot Controller

If using an Android phone, you must enable Developer Options and USB Debugging.

#### Step 1: Enable Developer Options

1. Open **Settings** on the phone
2. Go to **About Phone** (usually at the bottom)
3. Find **Build Number**
4. Tap **Build Number** 7 times
   - You'll see a message: "You are now a developer!"

#### Step 2: Enable USB Debugging

1. Go back to main **Settings**
2. Open **Developer Options** (now visible)
3. Enable **USB Debugging**
4. (Optional) Enable **Stay Awake** (keeps screen on while charging)

#### Step 3: Connect USB Cable

1. Use a high-quality USB cable (data capable, not just charging)
2. Connect phone to computer
3. On the phone, you'll see a popup: "Allow USB debugging?"
4. Check "Always allow from this computer"
5. Tap **OK**

**Troubleshooting:**
- If popup doesn't appear, try a different cable
- Make sure USB is in "File Transfer" or "PTP" mode, not just "Charging"
- Some cables are charge-only and won't work for data!

---

## 💻 Install Android Studio

### Download and Install

1. **Visit:** https://developer.android.com/studio
2. **Download** the installer for your OS:
   - Windows: `.exe` installer
   - macOS: `.dmg` disk image
   - Linux: `.tar.gz` archive
3. **Run the installer** - accept default options
4. **First launch:** Android Studio will download additional components (~2-3 GB)

**System Requirements:**
- 8 GB RAM minimum (16 GB recommended)
- 4 GB disk space for Android Studio
- 8+ GB disk space for Android SDK
- Windows 10/11, macOS 10.14+, or Linux (64-bit)

### Initial Setup Wizard

1. Choose **Standard** installation type
2. Select a UI theme (doesn't affect functionality)
3. Verify settings and click **Finish**
4. Wait for SDK download (~10 minutes)

---

## 📂 Open the Starter Kit Project

### Step 1: Clone or Download Repository

**Option A: Git Clone (Recommended)**
```bash
git clone https://github.com/your-team/ftc-decode-starter-kit.git
cd ftc-decode-starter-kit
```

**Option B: Download ZIP**
1. Go to repository on GitHub
2. Click green "Code" button
3. Select "Download ZIP"
4. Extract to a known location (e.g., `Documents/FTC/`)

### Step 2: Open in Android Studio

1. Launch **Android Studio**
2. Click **Open** (not "New Project"!)
3. Navigate to the cloned/extracted folder
4. Select the folder and click **OK**
5. Wait for **Gradle sync** (2-5 minutes first time)

**What's Happening:**
- Gradle downloads dependencies (FTC SDK, Sloth, etc.)
- Android Studio indexes your code
- Build system configures modules

**Progress Bar:** Bottom right corner - wait for it to complete!

---

## 🔌 Connect Your Robot Controller

### Method 1: Wireless ADB (Control Hub - Easiest)

**No cable needed!** Perfect for competition when USB isn't practical.

#### Step 1: Connect to Control Hub Wi-Fi
1. On your computer, connect to Control Hub's Wi-Fi:
   - Network name: `ControlHub-XXXX` or `FTC-XXXX`
   - Password: (found on label or configured during setup)

#### Step 2: Connect ADB Wirelessly
1. Open **Terminal** in Android Studio (bottom toolbar)
2. Run this command:
   ```bash
   adb connect 192.168.43.1:5555
   ```
3. Look for message: "connected to 192.168.43.1:5555"

**Verify Connection:**
```bash
adb devices
```
Expected output:
```
List of devices attached
192.168.43.1:5555    device
```

---

### Method 2: USB Cable (Phone Robot Controller)

**More reliable but requires cable.**

#### Step 1: Connect Phone
1. Plug USB cable into phone
2. Plug other end into computer
3. On phone: Tap "Allow USB debugging?" → **OK**

#### Step 2: Verify Connection in Android Studio
1. Look at the **device dropdown** (top toolbar, next to Run button)
2. You should see your device listed
3. If not, try:
   ```bash
   adb devices
   ```

**Troubleshooting: "No devices found"**

1. **Check cable** - try a different one (must support data transfer)
2. **Check drivers** (Windows only):
   - Device Manager → Look for yellow warning icons
   - Download drivers from phone manufacturer
   - REV Hub drivers: https://docs.revrobotics.com/rev-control-system/
3. **Restart ADB server:**
   ```bash
   adb kill-server
   adb start-server
   adb devices
   ```
4. **Check USB mode on phone:**
   - Swipe down notification panel
   - Tap "USB for file transfer"
   - Select "File Transfer" or "PTP"

---

## 🚀 Deploy Your Code

### Using Android Studio GUI (Recommended)

1. **Select device** from dropdown (top toolbar)
2. **Click green "Run" button** (play icon)
3. **Wait for build** (~30 seconds for incremental builds)
4. **App installs automatically!**

**What's Happening:**
1. Gradle compiles your Java code
2. Packages into an APK (Android Package)
3. ADB transfers APK to Robot Controller
4. App installs and launches

**Build Output:** Check "Build" tab at bottom for errors/warnings

---

### Using Command Line (Alternative)

#### Build APK
```bash
./gradlew assembleDebug
```

#### Find APK
```bash
# APK location:
FtcRobotController/build/outputs/apk/debug/FtcRobotController-debug.apk
```

#### Install via ADB
```bash
adb install -r FtcRobotController/build/outputs/apk/debug/FtcRobotController-debug.apk
```

**Note:** `-r` flag reinstalls and keeps app data

---

## 🐛 Debugging with ADB

### View Live Logs (Logcat)

**In Android Studio:**
1. Click **Logcat** tab (bottom toolbar)
2. Select your device from dropdown
3. See real-time logs from Robot Controller

**Filter logs:**
- Search bar: Type "ERROR" to see only errors
- Tag filter: Enter "FTC" or your class name

**In Terminal:**
```bash
adb logcat
```

**Filter in terminal:**
```bash
# Only show errors
adb logcat *:E

# Filter by tag
adb logcat -s RobotCore

# Save to file
adb logcat > robot_logs.txt
```

---

### Common Log Messages

**"OpMode started"** - Your code is running! ✅

**"java.lang.NullPointerException"** - You're accessing null hardware
- Check hardware config names
- Add null checks in code

**"Unable to find motor 'frontLeft'"** - Hardware config mismatch
- Open hardware config on Robot Controller
- Verify names match your code exactly

**"OpMode stopped"** - Normal end or crash
- Check logs above this message for errors

---

### Extract Files from Device

**Get configuration files:**
```bash
adb pull /sdcard/FIRST/FtcRobotController.xml ./robot-config.xml
```

**Get logs:**
```bash
adb pull /sdcard/FIRST/logs/ ./ftc-logs/
```

---

### Restart Robot Controller App

**From Android Studio / ADB:**
```bash
# Stop app
adb shell am force-stop com.qualcomm.ftcrobotcontroller

# Start app
adb shell am start -n com.qualcomm.ftcrobotcontroller/.FtcRobotControllerActivity
```

**Why restart:**
- Free up memory
- Reset after crashes
- Apply new hardware configuration

---

## 🔧 Advanced ADB Commands

### Check Device Info
```bash
# Battery level
adb shell dumpsys battery | grep level

# Android version
adb shell getprop ro.build.version.release

# Available storage
adb shell df -h
```

### Performance Monitoring
```bash
# CPU usage
adb shell top -n 1

# Memory usage
adb shell dumpsys meminfo com.qualcomm.ftcrobotcontroller
```

### Install Multiple Apps
```bash
# Install Robot Controller
adb install -r RobotController.apk

# Install Driver Station (on separate device)
adb -s <device-serial> install -r DriverStation.apk
```

---

## 🚨 Troubleshooting Common Issues

### Issue: "ADB not recognized as command"

**Windows Solution:**
1. Add Android SDK platform-tools to PATH:
   - Find folder: `C:\Users\YourName\AppData\Local\Android\Sdk\platform-tools`
   - System Properties → Environment Variables
   - Edit PATH → Add the folder above
2. Restart terminal/command prompt

**macOS/Linux Solution:**
```bash
# Add to ~/.bashrc or ~/.zshrc
export PATH=$PATH:~/Library/Android/sdk/platform-tools

# Reload
source ~/.bashrc
```

---

### Issue: "Unauthorized device"

**Solution:**
1. Disconnect USB cable
2. On phone: Settings → Developer Options → Revoke USB debugging authorizations
3. Reconnect cable
4. Approve popup again (check "Always allow")

---

### Issue: "Device offline"

**Solution:**
```bash
adb kill-server
adb start-server
adb devices
```

If still offline:
1. Restart Robot Controller device
2. Restart computer
3. Try different USB cable/port

---

### Issue: Can't connect wirelessly to Control Hub

**Solution:**
1. Verify Wi-Fi connection:
   - Connected to `ControlHub-XXXX`?
   - IP: `192.168.43.1` is correct
2. Check Control Hub is powered on
3. Try connecting via USB first, then switch to wireless
4. Restart Control Hub

**Manual wireless connection:**
```bash
adb tcpip 5555
adb connect 192.168.43.1:5555
```

---

### Issue: "Installation failed: INSTALL_FAILED_INSUFFICIENT_STORAGE"

**Solution:**
1. Free up space on Robot Controller
2. Delete old apps and files
3. Clear app cache:
   ```bash
   adb shell pm clear com.qualcomm.ftcrobotcontroller
   ```

---

### Issue: Very slow builds

**Solution:**
1. Check internet connection (first build downloads dependencies)
2. Close other programs to free RAM
3. Disable antivirus during build (may scan files)
4. Increase Gradle memory:
   - Edit `gradle.properties`
   - Change: `org.gradle.jvmargs=-Xmx4096m`

---

## 📚 Additional Resources

### Official Documentation
- **Android Studio:** https://developer.android.com/studio/intro
- **ADB Reference:** https://developer.android.com/tools/adb
- **REV Hub Docs:** https://docs.revrobotics.com/

### Video Tutorials
- Search YouTube: "FTC Android Studio setup"
- Official FTC channel has deployment tutorials

### Community Support
- **FTC Discord:** #programming channel
- **Chief Delphi:** FTC subforum
- **Reddit:** /r/FTC

---

## ✅ Quick Checklist

Before each practice/competition:

- [ ] Robot Controller fully charged
- [ ] USB cable tested (if using wired)
- [ ] ADB connection verified: `adb devices`
- [ ] Latest code deployed
- [ ] Hardware config names match code
- [ ] Driver Station paired with Robot Controller
- [ ] Test OpMode runs successfully

---

## 💡 Pro Tips

1. **Keep ADB connected during practice**
   - Live logs help debug instantly
   - Faster redeployment of code changes

2. **Use wireless ADB for competition**
   - No cable to trip over
   - More reliable in crowded environments

3. **Save logs after each match**
   - Helps analyze failures
   - Documents what worked

4. **Keep spare cables**
   - USB cables fail frequently
   - Always have backups

5. **Practice deployment under pressure**
   - Time yourself
   - Make sure team knows the process
   - Don't let one person be the only deployer

---

**You're now ready to deploy code like a pro!** 🚀

For issues not covered here, check the main [README.md](../README.md) or ask in the FTC community forums.

Good luck in the DECODE season! 🎉

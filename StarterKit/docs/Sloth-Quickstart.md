# Sloth Framework Quickstart Guide

This guide explains how the **Sloth Framework** is integrated into the FTC DECODE Starter Kit and how to use it in your robot code.

---

## 🦥 What is Sloth?

**Sloth** is an advanced robot control framework developed by Dairy Foundation specifically for FTC robots. It provides:

- **Structured initialization** - Consistent way to set up hardware
- **Lifecycle management** - Automatic handling of OpMode start/stop
- **Dependency injection** - Clean way to share components between subsystems
- **Build-time code generation** - Processes annotations during compilation

Think of Sloth as a "robot operating system layer" that sits between your code and the FTC SDK, making advanced patterns easier to implement.

---

## 🎯 Why This Starter Kit Uses Sloth

1. **Future-Proofing** - As you add features, Sloth's patterns scale better than manual wiring
2. **Industry Patterns** - Teaches professional software engineering concepts
3. **Competition-Tested** - Used by advanced FTC teams with proven reliability
4. **Optional Complexity** - You can use it simply at first, then unlock advanced features later

**Important:** You don't need to understand Sloth deeply to use this starter kit! The framework is already configured and working. This guide explains what's happening under the hood.

---

## 📦 How Sloth is Included

### Gradle Configuration

Sloth is loaded via a Gradle plugin that processes your code at build time. Here's how it's set up:

#### 1. Plugin Declaration (`TeamCode/build.gradle`)

```gradle
buildscript {
    repositories {
        mavenCentral()
        maven {
            url "https://repo.dairy.foundation/releases"
        }
    }
    dependencies {
        // Sloth Load Plugin - processes @Sloth annotations
        classpath "dev.frozenmilk:Load:0.2.4"
    }
}

// Apply the plugin
apply plugin: 'dev.frozenmilk.sinister.sloth.load'
```

**What this does:**
- Downloads the Sloth Load plugin from Dairy Foundation's Maven repository
- Applies the plugin to your TeamCode module
- The plugin runs during Gradle build, before Java compilation

#### 2. Runtime Dependencies (`TeamCode/build.gradle`)

```gradle
dependencies {
    // Sloth Framework - runtime library
    implementation("dev.frozenmilk.sinister:Sloth:0.2.4")
    
    // SlothBoard Dashboard - telemetry and debugging
    implementation("com.acmerobotics.slothboard:dashboard:0.2.4+0.4.17")
}
```

**What this does:**
- Includes Sloth runtime classes in your app
- Adds SlothBoard dashboard for advanced telemetry (optional feature)

#### 3. Repository Access (`build.dependencies.gradle`)

```gradle
repositories {
    mavenCentral()
    google()
    // Dairy Foundation hosts the Sloth framework
    maven { url = "https://repo.dairy.foundation/releases" }
}
```

---

## 🔧 How Sloth Works in This Starter Kit

### Current Usage (Simple Mode)

The starter kit uses Sloth in **infrastructure mode** - it's present and configured, but not heavily utilized in the basic code. This keeps the learning curve gentle.

**What's using Sloth:**
- Build system (plugin processes code)
- Available for future use (dashboard, annotations)

**What's NOT using Sloth yet:**
- Basic subsystems (StarterDrive, StarterShooter) use traditional patterns
- OpModes use standard LinearOpMode structure

This is intentional! You can start with familiar patterns, then adopt Sloth features as you grow.

---

## 🚀 Using Sloth Features (When You're Ready)

### Feature 1: SlothBoard Dashboard

**What it does:** Enhanced telemetry with graphs, field overlay, and configuration UI.

**How to enable:**
```java
// In your OpMode, add this import:
import dev.frozenmilk.slothboard.dashboard.FtcDashboard;

// Then use it:
FtcDashboard dashboard = FtcDashboard.getInstance();
dashboard.telemetry.addData("Key", value);
dashboard.telemetry.update();
```

**Access the dashboard:**
- Connect to Robot Controller Wi-Fi
- Open browser: `http://192.168.49.1:8080/dash`
- See real-time telemetry, graphs, and configuration

---

### Feature 2: Sloth Annotations (Advanced)

Sloth can automatically manage hardware initialization using annotations. This is more advanced but very powerful.

**Example: Auto-wiring motors**

```java
import dev.frozenmilk.sinister.sloth.annotations.Sloth;
import dev.frozenmilk.sinister.sloth.annotations.hardware.Motor;

@Sloth
public class MyDrive {
    @Motor("frontLeft")
    private DcMotor leftFront;
    
    @Motor("frontRight")
    private DcMotor rightFront;
    
    // Sloth automatically initializes these from hardwareMap!
}
```

**Benefits:**
- Less boilerplate code
- Consistent initialization patterns
- Easier to add new hardware

**When to use:**
- When you're comfortable with annotations
- When adding many hardware components
- When refactoring to advanced architecture

---

### Feature 3: Sloth Lifecycle Management

Sloth can manage OpMode lifecycle automatically.

**Example: Automatic cleanup**

```java
@Sloth
public class MySubsystem {
    @OnInit
    public void initialize() {
        // Called when OpMode initializes
    }
    
    @OnStart
    public void start() {
        // Called when START button pressed
    }
    
    @OnStop
    public void cleanup() {
        // Called when OpMode stops
        // Motors automatically stopped, no manual cleanup needed!
    }
}
```

---

## 🔍 Troubleshooting Sloth

### Build Error: "Plugin not found"

**Problem:** Gradle can't download Sloth plugin

**Solution:**
1. Check internet connection (required for first build)
2. Verify repository URL: `https://repo.dairy.foundation/releases`
3. Try: `./gradlew clean build --refresh-dependencies`

### Build Error: "Annotation processor failed"

**Problem:** Sloth plugin can't process your code

**Solution:**
1. Check for syntax errors in Java files
2. Make sure Java version is 8 or higher
3. Clean and rebuild: `./gradlew clean build`

### Runtime Error: "NoClassDefFoundError: Sloth"

**Problem:** Sloth runtime library not included

**Solution:**
1. Check `TeamCode/build.gradle` has Sloth dependency
2. Sync Gradle: Android Studio → File → Sync Project with Gradle Files
3. Rebuild and redeploy

### SlothBoard Dashboard Not Loading

**Problem:** Can't access dashboard in browser

**Solution:**
1. Verify Robot Controller Wi-Fi is active
2. Check IP address: `http://192.168.49.1:8080/dash`
3. Make sure SlothBoard dependency is included
4. Try different browser (Chrome recommended)

---

## 📚 Learning Path

### Stage 1: Use the Starter Kit (You Are Here)
- Sloth is configured and working
- Focus on understanding basic subsystems
- Don't worry about Sloth details yet

### Stage 2: Explore SlothBoard
- Enable dashboard for better telemetry
- Learn to use graphs and field overlay
- Configure subsystems via web UI

### Stage 3: Add Sloth Annotations
- Refactor one subsystem to use @Sloth annotations
- Try @Motor, @Servo automatic wiring
- Implement lifecycle methods (@OnInit, @OnStart, @OnStop)

### Stage 4: Advanced Patterns
- Use Sloth dependency injection
- Create reusable components
- Implement command-based patterns

---

## 🎓 Key Concepts Explained

### Build-Time vs Runtime

**Build-Time (Gradle Plugin):**
- Processes annotations before compilation
- Generates helper code
- No runtime performance cost

**Runtime (Library):**
- Provides base classes and utilities
- Manages lifecycle
- Handles initialization

### Dependency Injection

Instead of manually creating objects:
```java
// Manual way
DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
```

Sloth can inject them:
```java
// Sloth way
@Motor("motor")
private DcMotor motor;  // Automatically initialized!
```

### Lifecycle Management

OpModes have a lifecycle:
```
init → init_loop → start → loop × many → stop
```

Sloth helps you hook into each phase cleanly, without manual state tracking.

---

## 📖 Additional Resources

### Official Documentation
- **Sloth GitHub:** https://github.com/Dairy-Foundation/Sloth
- **Sloth Wiki:** Documentation and examples
- **API Reference:** JavaDocs for Sloth classes

### Community Support
- **FTC Discord:** #programming channel
- **Chief Delphi:** FTC Programming subforum
- **Team Forums:** Ask experienced teams

### Example Projects
- Look at competition code from teams using Sloth
- Study open-source FTC repositories
- Attend workshops and training sessions

---

## ⚠️ Important Notes

### Don't Change These (Unless You Know What You're Doing)

**In `TeamCode/build.gradle`:**
```gradle
classpath "dev.frozenmilk:Load:0.2.4"  // Plugin version
implementation("dev.frozenmilk.sinister:Sloth:0.2.4")  // Library version
```

**Why:** Version mismatches can cause build failures. Only update when officially supported.

### Safe to Customize

- Your own subsystems and OpModes
- Hardware names and configurations
- Telemetry display format
- Control schemes

### When to Update Sloth

- At the start of a new season (check for FTC SDK compatibility)
- When bug fixes are released
- When your team needs new features
- Never during competition season (stick with tested versions)

---

## 🎯 Quick Reference

### Check Sloth Version
```bash
./gradlew dependencies | grep Sloth
```

### Clean Build (If Errors)
```bash
./gradlew clean
./gradlew build --refresh-dependencies
```

### Enable SlothBoard
```java
import dev.frozenmilk.slothboard.dashboard.FtcDashboard;
FtcDashboard dashboard = FtcDashboard.getInstance();
```

### Access Dashboard
```
http://192.168.49.1:8080/dash
```

---

## 💡 Tips for Success

1. **Start Simple**
   - Use basic patterns first
   - Add Sloth features gradually
   - Test each change thoroughly

2. **Read the Errors**
   - Sloth build errors are usually descriptive
   - Check for typos in annotations
   - Verify hardware names match config

3. **Use Version Control**
   - Commit working code before adding Sloth features
   - Easy rollback if something breaks
   - Document what works

4. **Ask for Help**
   - Sloth community is friendly and helpful
   - Post code snippets when asking questions
   - Check existing issues on GitHub

---

## 🔄 Migration Path (Future)

When you're ready to fully leverage Sloth, here's the migration order:

1. **Add SlothBoard Dashboard** (easy, big benefit)
2. **Annotate one subsystem** (learn the pattern)
3. **Convert hardware initialization** (@Motor, @Servo)
4. **Implement lifecycle hooks** (@OnInit, @OnStart, @OnStop)
5. **Explore advanced features** (dependency injection, commands)

Each step is independent - you can stop whenever you want!

---

**Remember:** Sloth is a tool to help you, not a requirement. Use it when it makes your life easier, not because you feel you have to. The starter kit works great with or without deep Sloth usage! 🦥

Good luck with your DECODE season! 🎉

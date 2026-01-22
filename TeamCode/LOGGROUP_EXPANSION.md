# LogGroup Expansion - Aurora System v2

## Summary

Expanded `LogGroup` enum from 14 basic groups to **32 comprehensive groups** organized by functional area for the Aurora System v2.

## Complete Group List

### Core Robot / Runtime (3 groups)
- **OPMODE** - OpMode lifecycle, init/start/stop, mode transitions
- **LOOP** - Loop timing, cycle rate, overruns, heartbeat
- **SAFETY** - Deadman switch, inhibit reasons, interlocks

### Aurora / Game-Specific System (11 groups)
- **AURORA** - General Aurora coordination (high-level)
- **ARTIFACT** - Artifact identity/tracking/ledger changes
- **INDEXING** - Indexing state machine, slot occupancy
- **INTAKE** - Intake roller logic, collection triggers
- **TRANSFER** - Moving artifacts between locations
- **FIRING** - Firing sequences, feed timing
- **EJECT** - Eject/reject flows
- **SHOTPLAN** - Shot plan creation/consumption
- **PLANNEREX** - Planner/executor coordination
- **KEEPALIVE** - Watchdog + keep-alive integration

### Mechanisms (5 groups)
- **SHOOTER** - Shooter control, RPM/ready checks
- **TURRET** - Turret targeting/limits
- **DRIVE** - Drivetrain commands
- **LOCALIZE** - Localization math, pose updates
- **PATHING** - PedroPathing / autonomous builder

### Sensors / Vision (2 groups)
- **SENSORS** - Raw sensor reads, calibration, sanity checks
- **VISION** - Limelight and perception

### Utilities / Debugging (3 groups)
- **MONITOR** - SystemMonitor panel summaries
- **DBG** - General debugging
- **TEST** - Test OpModes and harness logs

### Legacy / Compatibility (3 groups)
- **AUTO** - Autonomous (deprecated, use OPMODE + PATHING)
- **HARDWARE** - Hardware init (deprecated, use OPMODE)
- **SYSTEM** - General system (deprecated, use OPMODE or AURORA)

## Package Mapping

Intuitive mapping to your package structure:

```
opmode/ + opmodes/           → OPMODE, TEST, DRIVE, INDEXING
util/aurora/ (top)           → AURORA, INDEXING, SHOTPLAN, PLANNEREX, 
                                FIRING, INTAKE, TRANSFER, SHOOTER, 
                                TURRET, VISION, LOCALIZE
util/aurora/v3/              → INDEXING, ARTIFACT, PLANNEREX, KEEPALIVE,
                                INTAKE, TRANSFER, FIRING, EJECT, SHOTPLAN
pedroPathing/                → PATHING
util/tool/                   → LOCALIZE (Pinpoint), SENSORS, DBG
```

## Recommended Defaults

### Enabled by Default (INFO+)
```java
Dbg.setGlobalLevel(LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.OPMODE, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.SAFETY, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.INDEXING, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.FIRING, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.SHOTPLAN, LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.PLANNEREX, LogLevel.INFO);
```

### Rate-Limited (DEBUG) - Use with everyMs()
```java
Dbg.setGroupLevel(LogGroup.DRIVE, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.LOCALIZE, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.SENSORS, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.VISION, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.KEEPALIVE, LogLevel.DEBUG);
```

### Off by Default - Enable When Debugging
```java
Dbg.setGroupEnabled(LogGroup.LOOP, false);
Dbg.setGroupEnabled(LogGroup.MONITOR, false);
Dbg.setGroupEnabled(LogGroup.DBG, false);
Dbg.setGroupEnabled(LogGroup.TEST, false);
```

## Usage Examples

### Subsystem Logging Pattern

**One group per decision-maker:**

```java
// FIRING - decisions about WHEN/WHY to fire
public class FiringSequenceCoordinator {
    public void startFiring() {
        Dbg.i(LogGroup.FIRING, "Starting firing sequence");
        Dbg.d(LogGroup.FIRING, "Gates passed: RPM ready, artifact present");
    }
}

// SHOOTER - RPM control details
public class Shooter {
    public void update() {
        Dbg.everyMs(LogGroup.SHOOTER, LogLevel.DEBUG, "rpm", 1000,
                    "RPM: %.0f / %.0f", currentRPM, targetRPM);
    }
}

// INDEXING - state machine decisions
public class IndexingSystem {
    private void transitionTo(SystemState newState) {
        Dbg.i(LogGroup.INDEXING, "State: %s -> %s", currentState, newState);
    }
}

// ARTIFACT - ledger/identity changes
public class SlotLedger {
    public void setSlotOccupied(Slot slot, Artifact artifact) {
        Dbg.d(LogGroup.ARTIFACT, "Slot %s now holds %s", slot, artifact);
    }
}
```

### OpMode Lifecycle

```java
@TeleOp(name="Match OpMode")
public class MatchOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        Dbg.i(LogGroup.OPMODE, "Initializing for match");
        
        // Init hardware
        Dbg.d(LogGroup.HARDWARE, "Hardware configured");
        
        waitForStart();
        Dbg.i(LogGroup.OPMODE, "Match started");
        
        while (opModeIsActive()) {
            // Loop timing
            Dbg.everyMs(LogGroup.LOOP, LogLevel.DEBUG, "timing", 5000,
                        "Loop rate: %.1f Hz", loopRate);
            
            // Safety checks
            if (!deadmanActive) {
                Dbg.w(LogGroup.SAFETY, "Deadman inactive - inhibited");
            }
        }
        
        Dbg.i(LogGroup.OPMODE, "Match ending");
    }
}
```

### Aurora V3 Operations

```java
// PLANNEREX - coordinator decisions
public class PlannerExecutor {
    public void executeNext() {
        Dbg.d(LogGroup.PLANNEREX, "Executing operation: %s", op.getType());
    }
}

// INTAKE - collection logic
public class CollectOperation {
    public void execute() {
        Dbg.i(LogGroup.INTAKE, "Starting collection from %s", side);
    }
}

// TRANSFER - movement between locations
public class TransferOperation {
    public void execute() {
        Dbg.i(LogGroup.TRANSFER, "Transferring %s: %s -> %s", 
              artifact, from, to);
    }
}

// EJECT - rejection flows
public class EjectOperation {
    public void execute() {
        Dbg.w(LogGroup.EJECT, "Ejecting %s, reason: %s", artifact, reason);
    }
}
```

### Debugging Scenarios

**Debug indexing issues:**
```java
Dbg.setGlobalLevel(LogLevel.WARN);                       // Quiet
Dbg.setGroupLevel(LogGroup.INDEXING, LogLevel.TRACE);    // Verbose
Dbg.setGroupLevel(LogGroup.ARTIFACT, LogLevel.TRACE);    // Verbose
Dbg.setGroupLevel(LogGroup.INTAKE, LogLevel.DEBUG);      // Detail
```

**Debug firing sequence:**
```java
Dbg.setGlobalLevel(LogLevel.WARN);
Dbg.setGroupLevel(LogGroup.FIRING, LogLevel.TRACE);
Dbg.setGroupLevel(LogGroup.SHOOTER, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.SHOTPLAN, LogLevel.DEBUG);
```

**Debug autonomous path:**
```java
Dbg.setGlobalLevel(LogLevel.INFO);
Dbg.setGroupLevel(LogGroup.PATHING, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.LOCALIZE, LogLevel.DEBUG);
Dbg.setGroupLevel(LogGroup.DRIVE, LogLevel.DEBUG);
```

## Design Principles

### Separation of Concerns
- **FIRING** logs decisions (when/why)
- **SHOOTER** logs mechanism state (RPM)
- **INDEXING** logs state machine
- **ARTIFACT** logs identity/tracking

### One Group Per Decision-Maker
❌ Don't mix concerns:
```java
// Bad - mixing FIRING decisions with SHOOTER details
Dbg.i(LogGroup.FIRING, "RPM: %.0f", rpm);  // Should be SHOOTER
```

✅ Keep focused:
```java
// Good - FIRING for decisions
Dbg.i(LogGroup.FIRING, "Initiating shot sequence");

// Good - SHOOTER for mechanism
Dbg.d(LogGroup.SHOOTER, "RPM: %.0f", rpm);
```

### Hierarchical Filtering
Enable broad categories, drill down when needed:
```java
// Start broad
Dbg.setGlobalLevel(LogLevel.INFO);  // All systems at INFO

// Narrow to specific issue
Dbg.setGroupLevel(LogGroup.INDEXING, LogLevel.TRACE);  // Deep dive
```

## Migration Guide

### Old Group → New Group

| Old Group | New Group | Notes |
|-----------|-----------|-------|
| `INDEX` | `INDEXING` | Expanded name for clarity |
| `FIRE` | `FIRING` | Expanded name for clarity |
| `SYSTEM` | `OPMODE` | More specific |
| `HARDWARE` | `OPMODE` | Use for init/lifecycle |
| `AUTO` | `OPMODE` + `PATHING` | Split by concern |
| `PLANNER` | `SHOTPLAN` | More descriptive |
| `ODOMETRY` | `LOCALIZE` | Broader scope |

### Updating Existing Code

**Global search and replace:**
```bash
# Replace old names
sed -i 's/LogGroup\.INDEX\b/LogGroup.INDEXING/g' *.java
sed -i 's/LogGroup\.FIRE\b/LogGroup.FIRING/g' *.java
sed -i 's/LogGroup\.SYSTEM/LogGroup.OPMODE/g' *.java
```

**Or manual review:** Check each usage and map to appropriate new group.

## Competition Configuration

### Minimal Logging (Performance)
```java
Dbg.setGlobalLevel(LogLevel.INFO);
Dbg.setGroupEnabled(LogGroup.LOOP, false);
Dbg.setGroupEnabled(LogGroup.MONITOR, false);
Dbg.setGroupEnabled(LogGroup.DBG, false);
Dbg.setGroupEnabled(LogGroup.TEST, false);
Dbg.setEnablePanelsTelemetry(false);
```

### Match Replay (Informative)
```java
Dbg.setGlobalLevel(LogLevel.INFO);
Dbg.setEnablePanelsTelemetry(true);
// All important events logged to Panels for review
```

## Files Modified

1. **`LogGroup.java`** - Expanded from 14 to 32 groups
2. **`DBG_QUICK_REFERENCE.java`** - Updated group list and presets
3. **`DbgLoggingExample.java`** - Updated to use new group names

## Benefits

✅ **Clear organization** - Groups match your architecture  
✅ **Intuitive naming** - Easy to find the right group  
✅ **Granular control** - Filter by specific concern  
✅ **Aurora-optimized** - Designed for your V3 system  
✅ **Backward compatible** - Legacy groups still exist  
✅ **Scalable** - Easy to add custom groups as needed  

## Summary

The LogGroup expansion provides a comprehensive, Aurora-optimized logging taxonomy that maps directly to your package structure and system architecture. Use the recommended defaults to start, then fine-tune based on debugging needs.

---

*Expanded: January 21, 2026*  
*Total Groups: 32 (29 active + 3 legacy)*  
*Categories: 5 (Runtime, Aurora, Mechanisms, Sensors, Utilities)*

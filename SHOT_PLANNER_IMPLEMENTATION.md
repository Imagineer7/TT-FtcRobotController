# Shot Planner and Executor System - Implementation Summary

## Overview
This document describes the Shot Planner and Planner Executor system implemented for the Aurora robot's indexing system. The system intelligently determines the optimal firing order for collected artifacts based on motif patterns and manages physical rearrangement operations.

## Architecture

### Component Separation
The system follows a clean separation of concerns:
- **ShotPlanner**: Pure logic component (WHAT to do)
- **PlannerExecutor**: Physical execution component (WHEN and HOW to do it)
- **IndexingSystem**: Integration layer (coordinates everything)

### ShotPlanner (Pure Logic)
**Location**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/ShotPlanner.java`

**Responsibilities**:
- Runs every loop cycle (unless skipped by rules)
- Decides which artifact should be in center position
- Produces shot plan (ordered list of artifacts to fire)
- Validates artifact location states
- Does NOT move hardware

**Key Features**:
1. **Pattern Scoring Algorithm**:
   - First artifact matching motif[0]: +3 points
   - Second artifact matching motif[1]: +2 points
   - Third artifact matching motif[2]: +1 point
   - Highest scoring order wins
   - Tie-breaker: prefer no rearrangement

2. **Skip Conditions**:
   - All artifacts are green
   - Two artifacts and both are green
   - Artifact count == 1 (can't rearrange)
   - Artifact count == 3 (can't rearrange)
   - manualPushMode == true

3. **Default Order**:
   - When planning is skipped, uses physical position order
   - Order: center → front intake → back intake
   - Ensures shot plans are physically realizable

### PlannerExecutor (Physical Execution)
**Location**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/PlannerExecutor.java`

**Responsibilities**:
- Executes physical rearrangement only when idle
- Ignores new planner requests while busy
- Enforces operation timeouts from IndexConfig
- Manages artifact location state updates
- Handles failure and lockout

**State Machine**:
- **IDLE**: Ready to accept requests
- **REARRANGING**: Executing a push operation
- **FAILED**: Operation failed/timed out, locked out until artifact count changes

**Key Features**:
1. **Timeout Enforcement**: Uses `IndexConfig.operationTimeout`
2. **Lockout Mechanism**: Prevents repeated failures
3. **State Validation**: Ensures rearrangement is physically possible

### Integration with IndexingSystem
**Location**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/IndexingSystem.java`

**New Methods**:
- `updateShotPlanner()`: Called every loop cycle
- `updatePlannerExecutor()`: Executes rearrangements when idle
- `executeRearrangement()`: Performs the physical push operation
- `canExecuteRearrangement()`: Validates request feasibility

**Integration Points**:
1. Planner and Executor instantiated in constructor
2. Both update methods called in main `update()` loop
3. Reuses existing push operation hardware logic
4. Compatible with manual push mode

## Rearrangement Rules

### When Rearrangement is Allowed
✅ Artifact count == 2
✅ Executor is idle
✅ manualPushMode == false
✅ System in READY_TO_FIRE state
✅ No operation in progress

### When Rearrangement is NOT Allowed
❌ Artifact count == 1 (nothing to swap)
❌ Artifact count == 3 (no empty intake for push)
❌ Executor is busy
❌ manualPushMode == true
❌ Rearrangement lockout is active

### Push Operation Sequence
1. Un-pre-position center artifact (retract uptake servos)
2. Reverse uptake servos (duration from IndexConfig)
3. Set destination intake to STORAGE mode (low holding speed)
4. Feed stored artifact into center (pushes center to opposite intake)
5. Re-pre-position new center artifact
6. Update artifact location states
7. Enforce timeout using IndexConfig.operationTimeout

## Testing

### Test Suite
**Location**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/aurora/ShotPlannerTest.java`

**Test Cases**:
1. ✅ Pattern scoring with PPG pattern
2. ✅ Pattern scoring with PGP pattern
3. ✅ Two artifact rearrangement request
4. ✅ Skip planning with one artifact
5. ✅ Skip planning with three artifacts
6. ✅ Skip planning with all green artifacts
7. ✅ Canonical scenario - PPG pattern (3-step sequence)

**Running Tests**:
```bash
cd /home/runner/work/TT-FtcRobotController/TT-FtcRobotController
./gradlew :TeamCode:compileDebugJavaWithJavac -q
java -cp "TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes:FtcRobotController/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes" org.firstinspires.ftc.teamcode.util.aurora.ShotPlannerTest
```

### Test Results
All 7 tests pass successfully, validating:
- Pattern scoring algorithm correctness
- Rearrangement request logic
- Skip condition handling
- Default order generation
- Canonical scenario (PPG with 3 artifacts: P → P → G)

## Usage Examples

### Setting Motif Pattern
```java
indexingSystem.setMotifPattern("PPG");  // Purple, Purple, Green
```

### Getting Shot Plan
```java
ShotPlanner planner = indexingSystem.getShotPlanner();
List<Artifact> shotPlan = planner.getShotPlan();
// Returns ordered list of artifacts to fire
```

### Checking Executor State
```java
PlannerExecutor executor = indexingSystem.getPlannerExecutor();
boolean isBusy = executor.isBusy();
ExecutorState state = executor.getState();
```

### Manual vs Auto Mode
```java
// Enable manual push mode (disables auto-rearrangement)
indexingSystem.setManualPushMode(true);

// Disable manual push mode (enables auto-rearrangement)
indexingSystem.setManualPushMode(false);
```

## Canonical Scenario: PPG Pattern

### Step 1: Green Collected
- Green goes to center
- No rearrangement needed (only 1 artifact)
- Shot plan: [G]

### Step 2: Purple Collected
- Purple goes to front intake
- Planner requests rearrangement (Purple should be first for PPG pattern)
- Executor validates and executes push operation
- Result: Purple in center, Green in back intake
- Shot plan: [P, G]

### Step 3: Second Purple Collected
- Second Purple stored in front intake
- No rearrangement possible (3 artifacts)
- Final shot plan: [P, P, G] ✓

## Key Design Decisions

### 1. Physical Position Priority
Default order uses physical positions (center → front → back) rather than collection order. This ensures:
- Shot plans are always physically realizable
- Aligns with mechanical constraints
- Center artifact is always fired first

### 2. Artifact Location State as Source of Truth
Artifact objects own their location state:
- Updated immediately before hardware operations
- Prevents race conditions
- Enables proper validation

### 3. Executor Lockout on Failure
When an operation times out or fails:
- Executor enters FAILED state
- Rearrangement locked out
- Lockout clears when artifact count changes
- Prevents repeated failures

### 4. Reuse of Existing Hardware Logic
Executor reuses existing push operation code:
- No duplicate hardware control
- Maintains consistency
- Reduces bug surface area

## Future Enhancements

### Potential Improvements
1. **Firing Logic Integration**: Connect shot plan to actual firing mechanism
2. **Advanced Scoring**: Consider artifact quality, distance, etc.
3. **Multi-Step Rearrangement**: For scenarios requiring multiple swaps
4. **Telemetry Visualization**: Show shot plan and scoring on driver station
5. **Performance Metrics**: Track rearrangement success rate and timing

### Extensibility Points
- Pattern scoring algorithm can be enhanced
- Additional skip conditions can be added
- Executor can support new operation types
- Telemetry can be customized

## Troubleshooting

### Common Issues

**Issue**: Rearrangement not happening
- Check: Is artifact count == 2?
- Check: Is manualPushMode disabled?
- Check: Is executor idle?
- Check: Is system in READY_TO_FIRE state?

**Issue**: Unexpected shot plan order
- Check: Is motif pattern set correctly?
- Check: Are artifact colors detected correctly?
- Check: Are artifact locations updated properly?

**Issue**: Executor timeout
- Check: Is IndexConfig.operationTimeout sufficient?
- Check: Are hardware motors/servos working?
- Check: Are artifacts physically stuck?

## Configuration

### Relevant IndexingConfig Parameters
- `operationTimeout`: Maximum time for push operation (default: 4.0s)
- `manualPushMode`: Enable/disable auto-rearrangement (default: true)
- `secondArtifactPushTime`: Duration of push operation (default: 2.5s)
- `storageIntakeAcceptTime`: Time for intake to accept artifact (default: 0.8s)

### Tuning Recommendations
1. Start with default values
2. Monitor operation success rate
3. Adjust timeouts if operations frequently timeout
4. Test with actual robot hardware for accurate timing

## Conclusion

The Shot Planner and Executor system provides intelligent, safe, and extensible artifact management for the Aurora robot. The clean separation of concerns, comprehensive testing, and integration with existing code ensures reliability and maintainability.

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * AutoHelper - Advanced Autonomous Framework (v3.0)
 * 
 * This class provides a comprehensive, easy-to-use framework for creating autonomous OpModes
 * using the AdvancedPositioningHelper system. It features intelligent error detection,
 * recovery mechanisms, and both fluent API and step-based autonomous programming.
 * 
 * Key Features:
 * - Fluent API for intuitive autonomous programming
 * - Smart error detection and automatic recovery
 * - Timeout handling and stuck detection
 * - AprilTag-based relocalization
 * - Dual camera support and automatic switching
 * - Comprehensive telemetry and debugging
 * - Future-ready for collision avoidance
 * 
 * Usage Example (Modern Fluent API):
 * 
 * public class MyAuto extends LinearOpMode {
 *     private AutoHelper autoHelper;
 *     
 *     @Override
 *     public void runOpMode() {
 *         autoHelper = new AutoHelper(this, hardwareMap, telemetry);
 *         autoHelper.initialize();
 *         
 *         waitForStart();
 *         
 *         autoHelper
 *             .moveTo(24, 36, 90, "Move to scoring position")
 *             .waitFor(1000, "Wait for arm to extend")
 *             .moveBy(6, 0, 0, "Approach target")
 *             .turnTo(45, "Turn to basket")
 *             .moveBy(-6, 0, 0, "Back away")
 *             .executeAll();
 *     }
 * }
 * 
 * Usage Example (Advanced with Error Handling):
 * 
 * public class MyAuto extends LinearOpMode {
 *     private AutoHelper autoHelper;
 *     
 *     @Override
 *     public void runOpMode() {
 *         autoHelper = new AutoHelper(this, hardwareMap, telemetry);
 *         autoHelper.setRelocalizationEnabled(true); // Enable AprilTag recovery
 *         autoHelper.setStuckDetectionEnabled(true); // Enable stuck detection
 *         autoHelper.initialize();
 *         
 *         waitForStart();
 *         
 *         autoHelper
 *             .addStep("Drive to specimen area", () -> 
 *                 autoHelper.aph.moveTo(12, 60, 0, 0.8, 3000))
 *             .addStep("Grab specimen", () -> {
 *                 // Custom action code here
 *                 sleep(500);
 *                 return true;
 *             })
 *             .moveTo(24, 36, 90, "Move to basket")
 *             .executeAllWithRecovery();
 *     }
 * }
 * 
 * @author FTC Team
 * @version 3.0
 */
public class AutoHelper {
    
    // Core system components
    public AdvancedPositioningHelper aph = null;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    
    // Step management
    private List<AutoStep> steps = new ArrayList<>();
    private int currentStepIndex = 0;
    
    // Error handling and recovery
    private boolean relocalizationEnabled = false;
    private boolean stuckDetectionEnabled = true;
    private boolean debugTelemetryEnabled = true;
    private ElapsedTime stepTimer = new ElapsedTime();
    private ElapsedTime lastPositionTimer = new ElapsedTime();
    private double lastX = 0, lastY = 0, lastHeading = 0;
    
    // Movement parameters
    private double defaultPower = 0.6;
    private double defaultTimeout = 5000; // 5 seconds
    private double stuckThreshold = 2.0; // inches
    private double stuckTimeThreshold = 2000; // 2 seconds
    private MovementTraversalMode defaultTraversalMode = MovementTraversalMode.DIRECT_VECTOR;
    
    // Execution state
    private boolean allStepsCompleted = false;
    private boolean executionFailed = false;
    private String lastErrorMessage = "";
    
    // Movement traversal modes
    public enum MovementTraversalMode {
        DIRECT_VECTOR,    // Straight line to target (current APH behavior)
        X_THEN_Y,        // Move X first, then Y, then rotate
        Y_THEN_X,        // Move Y first, then X, then rotate
        MANHATTAN_AUTO,   // Choose X or Y first based on which is larger distance
        SEPARATE_PHASES   // X, Y, and rotation as completely separate phases
    }
    
    // Inner class for step management
    public static class AutoStep {
        public final String description;
        public final Supplier<Boolean> action;
        public boolean completed = false;
        public boolean failed = false;
        public long executionTime = 0;
        public String errorMessage = "";
        
        public AutoStep(String description, Supplier<Boolean> action) {
            this.description = description;
            this.action = action;
        }
    }
    
    /**
     * Constructor - Creates AutoHelper with APH system
     * @param opMode The LinearOpMode instance
     * @param hardwareMap The hardware map
     * @param telemetry The telemetry object
     */
    public AutoHelper(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.steps = new ArrayList<>();
    }
    
    // ========== INITIALIZATION METHODS ==========
    
    /**
     * Initialize the AutoHelper with AdvancedPositioningHelper
     * Uses dual camera setup for maximum AprilTag visibility
     */
    public void initialize() {
        initialize("Webcam 1", "Webcam 2");
    }
    
    /**
     * Initialize with single camera
     * @param webcamName Name of webcam for AprilTag detection
     */
    public void initialize(String webcamName) {
        initialize(webcamName, null);
    }
    
    /**
     * Initialize with dual camera setup
     * @param frontWebcamName Front camera name (or null)
     * @param backWebcamName Back camera name (or null)
     */
    public void initialize(String frontWebcamName, String backWebcamName) {
        try {
            aph = new AdvancedPositioningHelper(opMode);
            
            if (frontWebcamName != null && backWebcamName != null) {
                // Dual camera initialization with default camera positions
                aph.initializeDualCamera(frontWebcamName, backWebcamName,
                    6.0, 0.0, 8.0, 0.0,     // Front camera: 6" forward, 8" up, facing forward
                    -6.0, 0.0, 8.0, 180.0); // Back camera: 6" back, 8" up, facing backward
                telemetry.addData("AutoHelper", "Initialized with dual cameras");
                telemetry.addData("Front Camera", frontWebcamName);
                telemetry.addData("Back Camera", backWebcamName);
            } else if (frontWebcamName != null) {
                // Single camera initialization
                aph.initialize(frontWebcamName);
                telemetry.addData("AutoHelper", "Initialized with single camera");
                telemetry.addData("Camera", frontWebcamName);
            } else {
                // Motor encoders only - use initialize with null webcam
                aph.initialize(null, false, false);
                telemetry.addData("AutoHelper", "Initialized with encoders only");
            }
            
            telemetry.addData("Status", "Ready for autonomous");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize: " + e.getMessage());
            telemetry.update();
            throw new RuntimeException("AutoHelper initialization failed", e);
        }
    }
    
    // ========== CONFIGURATION METHODS ==========
    
    /**
     * Enable or disable relocalization using AprilTags when stuck
     */
    public AutoHelper setRelocalizationEnabled(boolean enabled) {
        this.relocalizationEnabled = enabled;
        return this;
    }
    
    /**
     * Enable or disable stuck detection
     */
    public AutoHelper setStuckDetectionEnabled(boolean enabled) {
        this.stuckDetectionEnabled = enabled;
        return this;
    }
    
    /**
     * Enable or disable debug telemetry
     */
    public AutoHelper setDebugTelemetryEnabled(boolean enabled) {
        this.debugTelemetryEnabled = enabled;
        return this;
    }
    
    /**
     * Set default movement parameters
     */
    public AutoHelper setDefaultParameters(double power, double timeoutMs) {
        this.defaultPower = power;
        this.defaultTimeout = timeoutMs;
        return this;
    }
    
    /**
     * Set stuck detection parameters
     */
    public AutoHelper setStuckDetectionParameters(double thresholdInches, double timeThresholdMs) {
        this.stuckThreshold = thresholdInches;
        this.stuckTimeThreshold = timeThresholdMs;
        return this;
    }
    
    /**
     * Set default movement traversal mode
     * @param mode How the robot should traverse to target positions
     */
    public AutoHelper setDefaultTraversalMode(MovementTraversalMode mode) {
        this.defaultTraversalMode = mode;
        return this;
    }
    
    // ========== FLUENT API MOVEMENT METHODS ==========
    
    /**
     * Move to absolute field position using default traversal mode (fluent API)
     */
    public AutoHelper moveTo(double x, double y, double heading, String description) {
        return moveTo(x, y, heading, defaultPower, (long)defaultTimeout, defaultTraversalMode, description);
    }
    
    /**
     * Move to absolute field position with custom parameters (fluent API)
     */
    public AutoHelper moveTo(double x, double y, double heading, double power, long timeoutMs, String description) {
        return moveTo(x, y, heading, power, timeoutMs, defaultTraversalMode, description);
    }
    
    /**
     * Move to absolute field position with specific traversal mode (fluent API)
     */
    public AutoHelper moveTo(double x, double y, double heading, MovementTraversalMode traversalMode, String description) {
        return moveTo(x, y, heading, defaultPower, (long)defaultTimeout, traversalMode, description);
    }
    
    /**
     * Move to absolute field position with full control (fluent API)
     */
    public AutoHelper moveTo(double x, double y, double heading, double power, long timeoutMs, MovementTraversalMode traversalMode, String description) {
        return addStep(description, () -> executeMoveTo(x, y, heading, power, timeoutMs, traversalMode));
    }
    
    /**
     * Move by relative distance (fluent API)
     */
    public AutoHelper moveBy(double deltaX, double deltaY, double deltaHeading, String description) {
        return addStep(description, () -> executeMoveBy(deltaX, deltaY, deltaHeading, defaultPower));
    }
    
    /**
     * Move by relative distance with custom parameters (fluent API)
     */
    public AutoHelper moveBy(double deltaX, double deltaY, double deltaHeading, double power, long timeoutMs, String description) {
        return addStep(description, () -> executeMoveBy(deltaX, deltaY, deltaHeading, power));
    }
    
    /**
     * Turn to absolute heading (fluent API)
     */
    public AutoHelper turnTo(double heading, String description) {
        return addStep(description, () -> executeTurnTo(heading, defaultPower));
    }
    
    /**
     * Turn to absolute heading with custom parameters (fluent API)
     */
    public AutoHelper turnTo(double heading, double power, long timeoutMs, String description) {
        return addStep(description, () -> executeTurnTo(heading, power));
    }
    
    /**
     * Turn by relative angle (fluent API)
     */
    public AutoHelper turnBy(double deltaHeading, String description) {
        return addStep(description, () -> {
            double currentHeading = aph.getCurrentHeading();
            return executeTurnTo(currentHeading + deltaHeading, defaultPower);
        });
    }
    
    /**
     * Wait for specified time (fluent API)
     */
    public AutoHelper waitFor(long milliseconds, String description) {
        return addStep(description, () -> {
            opMode.sleep(milliseconds);
            return true;
        });
    }
    
    /**
     * Wait until a condition is met (fluent API)
     */
    public AutoHelper waitUntil(Supplier<Boolean> condition, String description) {
        return waitUntil(condition, 5000, description);
    }
    
    /**
     * Wait until a condition is met with timeout (fluent API)
     */
    public AutoHelper waitUntil(Supplier<Boolean> condition, long timeoutMs, String description) {
        return addStep(description, () -> {
            ElapsedTime timer = new ElapsedTime();
            while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMs) {
                if (condition.get()) {
                    return true;
                }
                opMode.sleep(50);
            }
            return false; // Timeout
        });
    }
    
    // ========== STEP MANAGEMENT METHODS ==========
    
    /**
     * Add a custom step with action (fluent API)
     */
    public AutoHelper addStep(String description, Supplier<Boolean> action) {
        steps.add(new AutoStep(description, action));
        return this;
    }
    
    /**
     * Add a simple action step (always returns true)
     */
    public AutoHelper addAction(String description, Runnable action) {
        return addStep(description, () -> {
            action.run();
            return true;
        });
    }
    
    /**
     * Clear all steps (useful for dynamic step building)
     */
    public AutoHelper clearSteps() {
        steps.clear();
        currentStepIndex = 0;
        allStepsCompleted = false;
        executionFailed = false;
        return this;
    }

    // ==========================
    // MOVE-BY-INCHES: FLUENT API
    // ==========================

    /**
     * Move N inches in a ROBOT-relative direction (0° = forward, +90° = right).
     * Keeps current heading by default.
     */
    public AutoHelper moveInchesRobot(double inches,
                                    double robotDirectionDeg,
                                    String description) {
        return moveInchesRobot(inches, robotDirectionDeg, defaultPower, (long)defaultTimeout, /*holdHeading=*/true, description);
    }

    public AutoHelper moveInchesRobot(double inches,
                                    double robotDirectionDeg,
                                    double power,
                                    long timeoutMs,
                                    boolean holdHeading,
                                    String description) {
        return addStep(description, () ->
            executeMoveInchesRobot(inches, robotDirectionDeg, power, timeoutMs, holdHeading)
        );
    }

    /**
     * Move N inches in a FIELD-relative direction (0° = +Y toward Blue, +90° = +X toward audience).
     * By default, holds the current heading while translating.
     */
    public AutoHelper moveInchesField(double inches,
                                    double fieldDirectionDeg,
                                    String description) {
        return moveInchesField(inches, fieldDirectionDeg, aph != null ? aph.getCurrentHeading() : 0.0,
                defaultPower, (long)defaultTimeout, description);
    }

    public AutoHelper moveInchesField(double inches,
                                    double fieldDirectionDeg,
                                    double targetHeading,
                                    double power,
                                    long timeoutMs,
                                    String description) {
        return addStep(description, () ->
            executeMoveInchesField(inches, fieldDirectionDeg, targetHeading, power, timeoutMs)
        );
    }

    /**
     * Cardinal robot-frame moves (FORWARD/BACKWARD/LEFT/RIGHT), holding heading.
     */
    public AutoHelper moveCardinal(AdvancedPositioningHelper.MoveDir dir,
                                double inches,
                                String description) {
        return moveCardinal(dir, inches, defaultPower, (long)defaultTimeout, description);
    }

    public AutoHelper moveCardinal(AdvancedPositioningHelper.MoveDir dir,
                                double inches,
                                double power,
                                long timeoutMs,
                                String description) {
        return addStep(description, () ->
            executeMoveCardinal(dir, inches, power, timeoutMs)
        );
    }

    // Convenience sugar
    public AutoHelper forward(double inches, String description) {
        return moveCardinal(AdvancedPositioningHelper.MoveDir.FORWARD, inches, description);
    }
    public AutoHelper back(double inches, String description) {
        return moveCardinal(AdvancedPositioningHelper.MoveDir.BACKWARD, inches, description);
    }
    public AutoHelper strafeRight(double inches, String description) {
        return moveCardinal(AdvancedPositioningHelper.MoveDir.RIGHT, inches, description);
    }
    public AutoHelper strafeLeft(double inches, String description) {
        return moveCardinal(AdvancedPositioningHelper.MoveDir.LEFT, inches, description);
    }

    // ==================================
    // MOVE-BY-INCHES: EXECUTION HELPERS
    // ==================================

    private boolean executeMoveInchesRobot(double inches,
                                        double robotDirectionDeg,
                                        double power,
                                        long timeoutMs,
                                        boolean holdHeading) {
        if (aph == null) {
            telemetry.addData("ERROR", "APH not initialized");
            return false;
        }
        // Convert ms → s for APH method
        double timeoutSec = Math.max(0.1, timeoutMs / 1000.0);

        boolean ok = aph.moveInchesRobot(inches, robotDirectionDeg, clamp01(power), timeoutSec, holdHeading);

        if (!ok) {
            telemetry.addData("moveInchesRobot", "Failed/Timeout: %.1f in @ %.0f° (%.1fs)",
                    inches, robotDirectionDeg, timeoutSec);
        }
        return ok;
    }

    private boolean executeMoveInchesField(double inches,
                                        double fieldDirectionDeg,
                                        double targetHeading,
                                        double power,
                                        long timeoutMs) {
        if (aph == null) {
            telemetry.addData("ERROR", "APH not initialized");
            return false;
        }
        double timeoutSec = Math.max(0.1, timeoutMs / 1000.0);

        boolean ok = aph.moveInchesField(inches, fieldDirectionDeg, targetHeading, clamp01(power), timeoutSec);

        if (!ok) {
            telemetry.addData("moveInchesField", "Failed/Timeout: %.1f in @ field %.0f° → H=%.0f° (%.1fs)",
                    inches, fieldDirectionDeg, targetHeading, timeoutSec);
        }
        return ok;
    }

    private boolean executeMoveCardinal(AdvancedPositioningHelper.MoveDir dir,
                                        double inches,
                                        double power,
                                        long timeoutMs) {
        if (aph == null) {
            telemetry.addData("ERROR", "APH not initialized");
            return false;
        }
        double timeoutSec = Math.max(0.1, timeoutMs / 1000.0);

        boolean ok = aph.moveCardinalRobot(dir, inches, clamp01(power), timeoutSec);

        if (!ok) {
            telemetry.addData("moveCardinal", "Failed/Timeout: %s %.1f in (%.1fs)", dir, inches, timeoutSec);
        }
        return ok;
    }

    // Small utility to keep power sane
    private double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }
    
    // ========== MOVEMENT EXECUTION METHODS ==========
    
    /**
     * Execute movement to target position using specified traversal mode
     */
    private boolean executeMoveTo(double targetX, double targetY, double targetHeading, double power, long timeoutMs, MovementTraversalMode traversalMode) {
        if (aph == null) {
            telemetry.addData("ERROR", "APH not initialized");
            return false;
        }
        
        switch (traversalMode) {
            case DIRECT_VECTOR:
                // Use existing APH method for direct vector movement
                return aph.goToPosition(targetX, targetY, targetHeading, power);
                
            case X_THEN_Y:
                return executeManhattanMovement(targetX, targetY, targetHeading, power, timeoutMs, true);
                
            case Y_THEN_X:
                return executeManhattanMovement(targetX, targetY, targetHeading, power, timeoutMs, false);
                
            case MANHATTAN_AUTO:
                return executeManhattanMovementAuto(targetX, targetY, targetHeading, power, timeoutMs);
                
            case SEPARATE_PHASES:
                return executeSeparatePhaseMovement(targetX, targetY, targetHeading, power, timeoutMs);
                
            default:
                return aph.goToPosition(targetX, targetY, targetHeading, power);
        }
    }
    
    /**
     * Execute Manhattan (L-shaped) movement
     * @param xFirst true to move X first, false to move Y first
     */
    private boolean executeManhattanMovement(double targetX, double targetY, double targetHeading, double power, long timeoutMs, boolean xFirst) {
        ElapsedTime totalTimer = new ElapsedTime();
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        double currentHeading = aph.getCurrentHeading();
        
        double phaseTimeout = timeoutMs / 3.0; // Divide time between X, Y, and rotation phases
        
        if (xFirst) {
            // Phase 1: Move X only
            if (Math.abs(targetX - currentX) > 1.0) { // 1 inch tolerance
                if (!aph.goToPosition(targetX, currentY, currentHeading, power)) {
                    if (totalTimer.milliseconds() > phaseTimeout) return false;
                }
            }
            
            // Phase 2: Move Y only
            if (Math.abs(targetY - aph.getCurrentY()) > 1.0) {
                if (!aph.goToPosition(targetX, targetY, currentHeading, power)) {
                    if (totalTimer.milliseconds() > phaseTimeout * 2) return false;
                }
            }
        } else {
            // Phase 1: Move Y only
            if (Math.abs(targetY - currentY) > 1.0) {
                if (!aph.goToPosition(currentX, targetY, currentHeading, power)) {
                    if (totalTimer.milliseconds() > phaseTimeout) return false;
                }
            }
            
            // Phase 2: Move X only
            if (Math.abs(targetX - aph.getCurrentX()) > 1.0) {
                if (!aph.goToPosition(targetX, targetY, currentHeading, power)) {
                    if (totalTimer.milliseconds() > phaseTimeout * 2) return false;
                }
            }
        }
        
        // Phase 3: Rotate to final heading
        if (Math.abs(targetHeading - aph.getCurrentHeading()) > 2.0) { // 2 degree tolerance
            if (!aph.rotateToHeading(targetHeading, power)) {
                if (totalTimer.milliseconds() > timeoutMs) return false;
            }
        }
        
        return true;
    }
    
    /**
     * Execute Manhattan movement, automatically choosing X or Y first based on larger distance
     */
    private boolean executeManhattanMovementAuto(double targetX, double targetY, double targetHeading, double power, long timeoutMs) {
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        
        double deltaX = Math.abs(targetX - currentX);
        double deltaY = Math.abs(targetY - currentY);
        
        // Move along the axis with the larger distance first
        boolean xFirst = deltaX >= deltaY;
        
        if (debugTelemetryEnabled) {
            telemetry.addData("Manhattan Auto", "Moving %s first (%.1f\" vs %.1f\")", 
                xFirst ? "X" : "Y", deltaX, deltaY);
        }
        
        return executeManhattanMovement(targetX, targetY, targetHeading, power, timeoutMs, xFirst);
    }
    
    /**
     * Execute movement as completely separate phases: X, then Y, then rotation
     */
    private boolean executeSeparatePhaseMovement(double targetX, double targetY, double targetHeading, double power, long timeoutMs) {
        ElapsedTime totalTimer = new ElapsedTime();
        double phaseTimeout = timeoutMs / 3.0;
        
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        double currentHeading = aph.getCurrentHeading();
        
        // Phase 1: X movement only (strafe)
        if (Math.abs(targetX - currentX) > 1.0) {
            ElapsedTime phaseTimer = new ElapsedTime();
            while (opMode.opModeIsActive() && Math.abs(targetX - aph.getCurrentX()) > 1.0) {
                if (phaseTimer.milliseconds() > phaseTimeout) {
                    if (debugTelemetryEnabled) {
                        telemetry.addData("Phase 1", "X movement timeout");
                    }
                    return false;
                }
                
                // Pure strafe movement
                double error = targetX - aph.getCurrentX();
                double strafePower = Math.max(0.2, Math.min(power, Math.abs(error) * 0.1));
                if (error < 0) strafePower = -strafePower;
                
                aph.setDrivePowers(0, strafePower, 0); // Forward=0, Strafe=power, Rotate=0
                aph.updatePosition();
                opMode.sleep(20);
            }
            aph.stopMotors();
        }
        
        // Phase 2: Y movement only (forward/backward)
        if (Math.abs(targetY - aph.getCurrentY()) > 1.0) {
            ElapsedTime phaseTimer = new ElapsedTime();
            while (opMode.opModeIsActive() && Math.abs(targetY - aph.getCurrentY()) > 1.0) {
                if (phaseTimer.milliseconds() > phaseTimeout) {
                    if (debugTelemetryEnabled) {
                        telemetry.addData("Phase 2", "Y movement timeout");
                    }
                    return false;
                }
                
                // Pure forward/backward movement
                double error = targetY - aph.getCurrentY();
                double forwardPower = Math.max(0.2, Math.min(power, Math.abs(error) * 0.1));
                if (error < 0) forwardPower = -forwardPower;
                
                aph.setDrivePowers(forwardPower, 0, 0); // Forward=power, Strafe=0, Rotate=0
                aph.updatePosition();
                opMode.sleep(20);
            }
            aph.stopMotors();
        }
        
        // Phase 3: Rotation only
        if (Math.abs(targetHeading - aph.getCurrentHeading()) > 2.0) {
            if (!aph.rotateToHeading(targetHeading, power)) {
                if (totalTimer.milliseconds() > timeoutMs) return false;
            }
        }
        
        return true;
    }
    
    // ========== EXECUTION METHODS ==========
    
    /**
     * Execute all steps with basic error handling
     */
    public boolean executeAll() {
        return executeSteps(false);
    }
    
    /**
     * Execute all steps with advanced recovery mechanisms
     */
    public boolean executeAllWithRecovery() {
        return executeSteps(true);
    }
    
    /**
     * Execute steps with optional recovery
     */
    private boolean executeSteps(boolean useRecovery) {
        if (steps.isEmpty()) {
            telemetry.addData("AutoHelper", "No steps to execute");
            telemetry.update();
            return true;
        }
        
        telemetry.addData("AutoHelper", "Executing %d steps", steps.size());
        telemetry.update();
        
        currentStepIndex = 0;
        allStepsCompleted = false;
        executionFailed = false;
        
        for (currentStepIndex = 0; currentStepIndex < steps.size() && opMode.opModeIsActive(); currentStepIndex++) {
            AutoStep step = steps.get(currentStepIndex);
            
            if (debugTelemetryEnabled) {
                telemetry.addData("Current Step", "%d/%d: %s", currentStepIndex + 1, steps.size(), step.description);
                telemetry.update();
            }
            
            if (!executeStep(step, useRecovery)) {
                executionFailed = true;
                lastErrorMessage = step.errorMessage;
                
                telemetry.addData("ERROR", "Step %d failed: %s", currentStepIndex + 1, step.errorMessage);
                telemetry.addData("Step", step.description);
                telemetry.update();
                
                return false;
            }
        }
        
        allStepsCompleted = true;
        telemetry.addData("AutoHelper", "All steps completed successfully");
        telemetry.update();
        
        return true;
    }
    
    /**
     * Execute a single step with error handling and recovery
     */
    private boolean executeStep(AutoStep step, boolean useRecovery) {
        stepTimer.reset();
        
        // Store position for stuck detection
        if (stuckDetectionEnabled && aph != null) {
            lastX = aph.getCurrentX();
            lastY = aph.getCurrentY();
            lastHeading = aph.getCurrentHeading();
            lastPositionTimer.reset();
        }
        
        try {
            boolean success = step.action.get();
            step.executionTime = (long)stepTimer.milliseconds();
            
            if (success) {
                step.completed = true;
                return true;
            } else {
                step.failed = true;
                step.errorMessage = "Step returned false (timeout or failure)";
                
                // Attempt recovery if enabled
                if (useRecovery && attemptRecovery(step)) {
                    step.completed = true;
                    step.failed = false;
                    step.errorMessage = "Recovered successfully";
                    return true;
                }
                
                return false;
            }
            
        } catch (Exception e) {
            step.failed = true;
            step.errorMessage = "Exception: " + e.getMessage();
            step.executionTime = (long)stepTimer.milliseconds();
            
            // Attempt recovery if enabled
            if (useRecovery && attemptRecovery(step)) {
                step.completed = true;
                step.failed = false;
                step.errorMessage = "Recovered from exception";
                return true;
            }
            
            return false;
        }
    }
    
    /**
     * Attempt to recover from a failed step
     */
    private boolean attemptRecovery(AutoStep failedStep) {
        telemetry.addData("Recovery", "Attempting recovery for: %s", failedStep.description);
        telemetry.update();
        
        // Check if robot is stuck
        if (stuckDetectionEnabled && isRobotStuck()) {
            telemetry.addData("Recovery", "Robot appears stuck, attempting relocalization");
            telemetry.update();
            
            // Try AprilTag relocalization
            if (relocalizationEnabled && aph != null && aph.attemptAprilTagRelocalization()) {
                telemetry.addData("Recovery", "Relocalization successful");
                telemetry.update();
                opMode.sleep(500); // Brief pause before retry
                
                // Retry the failed step
                try {
                    return failedStep.action.get();
                } catch (Exception e) {
                    telemetry.addData("Recovery", "Retry failed: %s", e.getMessage());
                    telemetry.update();
                }
            }
        }
        
        // Additional recovery strategies can be added here
        // For example: back up slightly, try alternate path, etc.
        
        telemetry.addData("Recovery", "Recovery failed");
        telemetry.update();
        return false;
    }
    
    /**
     * Check if robot appears to be stuck
     */
    private boolean isRobotStuck() {
        if (aph == null || lastPositionTimer.milliseconds() < stuckTimeThreshold) {
            return false;
        }
        
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        double currentHeading = aph.getCurrentHeading();
        
        double distanceMoved = Math.sqrt(
            Math.pow(currentX - lastX, 2) + 
            Math.pow(currentY - lastY, 2)
        );
        
        double headingChange = Math.abs(currentHeading - lastHeading);
        if (headingChange > 180) headingChange = 360 - headingChange;
        
        // Robot is considered stuck if it hasn't moved much in the time threshold
        return distanceMoved < stuckThreshold && headingChange < 10; // 10 degrees
    }
    
    // ========== STATUS AND TELEMETRY METHODS ==========
    
    /**
     * Get current execution status
     */
    public boolean isExecutionComplete() {
        return allStepsCompleted;
    }
    
    /**
     * Get execution failure status
     */
    public boolean hasExecutionFailed() {
        return executionFailed;
    }
    
    /**
     * Get last error message
     */
    public String getLastErrorMessage() {
        return lastErrorMessage;
    }
    
    /**
     * Get current step information
     */
    public String getCurrentStepInfo() {
        if (currentStepIndex >= steps.size()) {
            return "All steps completed";
        }
        AutoStep step = steps.get(currentStepIndex);
        return String.format("Step %d/%d: %s", currentStepIndex + 1, steps.size(), step.description);
    }
    
    /**
     * Display comprehensive telemetry
     */
    public void displayTelemetry() {
        if (!debugTelemetryEnabled) return;
        
        telemetry.addData("AutoHelper Status", allStepsCompleted ? "Complete" : executionFailed ? "Failed" : "Running");
        telemetry.addData("Current Step", getCurrentStepInfo());
        
        if (aph != null) {
            telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°", 
                aph.getCurrentX(), aph.getCurrentY(), aph.getCurrentHeading());
        }
        
        if (executionFailed) {
            telemetry.addData("Last Error", lastErrorMessage);
        }
        
        // Show step summary
        int completed = 0, failed = 0;
        for (AutoStep step : steps) {
            if (step.completed) completed++;
            if (step.failed) failed++;
        }
        
        telemetry.addData("Steps", "Completed: %d, Failed: %d, Total: %d", completed, failed, steps.size());
        telemetry.update();
    }
    
    /**
     * Get detailed execution report
     */
    public String getExecutionReport() {
        StringBuilder report = new StringBuilder();
        report.append("AutoHelper Execution Report\n");
        report.append("==========================\n");
        report.append(String.format("Total Steps: %d\n", steps.size()));
        
        int completed = 0, failed = 0;
        long totalTime = 0;
        
        for (int i = 0; i < steps.size(); i++) {
            AutoStep step = steps.get(i);
            report.append(String.format("\nStep %d: %s\n", i + 1, step.description));
            report.append(String.format("  Status: %s\n", 
                step.completed ? "Completed" : step.failed ? "Failed" : "Not executed"));
            report.append(String.format("  Time: %d ms\n", step.executionTime));
            
            if (step.failed && !step.errorMessage.isEmpty()) {
                report.append(String.format("  Error: %s\n", step.errorMessage));
            }
            
            if (step.completed) completed++;
            if (step.failed) failed++;
            totalTime += step.executionTime;
        }
        
        report.append(String.format("\nSummary: %d completed, %d failed\n", completed, failed));
        report.append(String.format("Total execution time: %d ms\n", totalTime));
        
        return report.toString();
    }
    
    // ========== UTILITY METHODS ==========
    
    /**
     * Emergency stop - stops all motors and clears remaining steps
     */
    public void emergencyStop() {
        if (aph != null) {
            aph.stopMotors();
        }
        
        // Clear remaining steps
        for (int i = currentStepIndex; i < steps.size(); i++) {
            steps.get(i).failed = true;
            steps.get(i).errorMessage = "Emergency stop";
        }
        
        executionFailed = true;
        lastErrorMessage = "Emergency stop activated";
        
        telemetry.addData("AutoHelper", "EMERGENCY STOP ACTIVATED");
        telemetry.update();
    }
    
    /**
     * Get the APH instance for direct access
     */
    public AdvancedPositioningHelper getPositioningHelper() {
        return aph;
    }
    
    /**
     * Check if OpMode is still active (wrapper for convenience)
     */
    public boolean isActive() {
        return opMode.opModeIsActive();
    }
    
    /**
     * Sleep wrapper for convenience
     */
    public void sleep(long milliseconds) {
        opMode.sleep(milliseconds);
    }
    
    // ========== HELPER METHODS FOR MOVEMENT ==========
    
    /**
     * Execute relative movement using current position
     */
    private boolean executeMoveBy(double deltaX, double deltaY, double deltaHeading, double power) {
        if (aph == null) return false;
        
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        double currentHeading = aph.getCurrentHeading();
        
        double targetX = currentX + deltaX;
        double targetY = currentY + deltaY;
        double targetHeading = currentHeading + deltaHeading;
        
        return aph.goToPosition(targetX, targetY, targetHeading, power);
    }
    
    /**
     * Execute turn to absolute heading
     */
    private boolean executeTurnTo(double targetHeading, double power) {
        if (aph == null) return false;
        
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        
        return aph.goToPosition(currentX, currentY, targetHeading, power);
    }
}
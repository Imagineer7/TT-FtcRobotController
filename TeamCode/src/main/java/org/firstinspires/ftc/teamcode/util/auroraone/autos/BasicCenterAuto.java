package org.firstinspires.ftc.teamcode.util.auroraone.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;
import org.firstinspires.ftc.teamcode.util.auroraone.core.StateMachine;
import org.firstinspires.ftc.teamcode.util.auroraone.core.Pose2D;
import org.firstinspires.ftc.teamcode.util.auroraone.utility.Logger;
import org.firstinspires.ftc.teamcode.util.auroraone.core.Blackboard;
import org.firstinspires.ftc.teamcode.util.auroraone.subsystems.localization.VisionLocalizer;

/**
 * Basic Center Auto - Simple autonomous routine
 *
 * This autonomous routine performs a simple two-step movement:
 * 1. Drive to center position (0, 0)
 * 2. Drive +24 inches along the X-axis to position (24, 0)
 *
 * Uses vision-based start position detection with fallback to configured position.
 */
@Autonomous(name = "Basic Center Auto", group = "Aurora", preselectTeleOp = "Aurora TeleOp")
public class BasicCenterAuto extends LinearOpMode {

    // =========================================================================================
    // CONFIGURATION CONSTANTS
    // =========================================================================================

    /**
     * Alliance color selection
     */
    private static final Alliance ALLIANCE = Alliance.RED;

    /**
     * Starting position selection
     */
    private static final StartPosition START_POSITION = StartPosition.LEFT;

    /**
     * Maximum autonomous runtime before timeout (seconds)
     */
    private static final double MAX_AUTO_TIME = 29.5;

    /**
     * Enable debug telemetry and logging
     */
    private static final boolean DEBUG_MODE = true;

    // =========================================================================================
    // ALLIANCE AND POSITION ENUMS
    // =========================================================================================

    public enum Alliance {
        RED("Red", 1),
        BLUE("Blue", -1);

        private final String name;
        private final int multiplier;

        Alliance(String name, int multiplier) {
            this.name = name;
            this.multiplier = multiplier;
        }

        public String getName() { return name; }
        public int getMultiplier() { return multiplier; }
    }

    public enum StartPosition {
        LEFT("Left", new Pose2D(12, 63, 90)),
        RIGHT("Right", new Pose2D(132, 63, 90));

        private final String name;
        private final Pose2D position;

        StartPosition(String name, Pose2D position) {
            this.name = name;
            this.position = position;
        }

        public String getName() { return name; }
        public Pose2D getPosition() { return position; }
    }

    // =========================================================================================
    // AUTONOMOUS SEQUENCE STEPS
    // =========================================================================================

    public enum AutoStep {
        INITIALIZE("Initialize robot systems"),
        DRIVE_TO_CENTER("Drive to center (0,0)"),
        DRIVE_PLUS_24X("Drive +24 inches on X-axis"),
        COMPLETE("Autonomous complete");

        private final String description;

        AutoStep(String description) {
            this.description = description;
        }

        public String getDescription() { return description; }
    }

    // =========================================================================================
    // INSTANCE VARIABLES
    // =========================================================================================

    // Core systems
    private StateMachine stateMachine;
    private RobotMap robotMap;
    private Logger logger;
    private Blackboard blackboard;

    // Timing and performance
    private ElapsedTime autonomousTimer;
    private ElapsedTime stepTimer;

    // Autonomous state tracking
    private AutoStep currentStep;
    private boolean emergencyStop;

    // Current robot position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Target positions for our basic auto
    private final Pose2D CENTER_POSITION = new Pose2D(0, 0, 0);
    private final Pose2D PLUS_24X_POSITION = new Pose2D(24, 0, 0);

    // =========================================================================================
    // MAIN AUTONOMOUS ENTRY POINT
    // =========================================================================================

    @Override
    public void runOpMode() {
        try {
            // Initialize all systems
            initializeRobot();

            // Display initialization status
            displayInitializationInfo();

            // Wait for start
            logger.info("BasicCenterAuto", "Waiting for start...");
            waitForStart();

            // Check if OpMode was stopped during init
            if (isStopRequested()) {
                return;
            }

            // Start autonomous timer
            autonomousTimer.reset();
            logger.info("BasicCenterAuto", String.format(java.util.Locale.US,
                "Starting Basic Center Auto - %s %s", ALLIANCE.getName(), START_POSITION.getName()));

            // Run the autonomous sequence
            runAutonomousSequence();

        } catch (Exception e) {
            logger.error("BasicCenterAuto", "Autonomous failed: " + e.getMessage());
            handleEmergencyStop("Unexpected error: " + e.getMessage());
        } finally {
            // Cleanup and final status
            cleanup();
        }
    }

    // =========================================================================================
    // INITIALIZATION METHODS
    // =========================================================================================

    /**
     * Initialize all robot systems and hardware
     */
    private void initializeRobot() {
        telemetry.addData("Status", "Initializing Aurora One systems...");
        telemetry.update();

        // Initialize core systems
        logger = Logger.getInstance();
        blackboard = Blackboard.getInstance();
        autonomousTimer = new ElapsedTime();
        stepTimer = new ElapsedTime();

        // Initialize robot hardware
        robotMap = new RobotMap(hardwareMap, telemetry);

        // Initialize state machine
        stateMachine = StateMachine.getInstance();
        stateMachine.initialize(robotMap, telemetry);
        stateMachine.setDebugMode(DEBUG_MODE);

        // Setup autonomous-specific states
        setupAutonomousStates();

        // Try to set initial position from vision, fallback to configured position
        if (!trySetStartPositionFromVision()) {
            setInitialPosition();
            logger.info("BasicCenterAuto", "Using configured start position - vision detection failed or unavailable");
        }

        // Initialize tracking variables
        currentStep = AutoStep.INITIALIZE;
        emergencyStop = false;

        logger.info("BasicCenterAuto", "Robot initialization complete");
    }

    /**
     * Setup custom autonomous states in the state machine
     */
    private void setupAutonomousStates() {

        // AUTO_INIT State - Autonomous initialization
        stateMachine.addState(new StateMachine.BaseState(StateMachine.State.AUTO_INIT) {
            @Override
            public void onEntry() {
                super.onEntry();
                currentStep = AutoStep.INITIALIZE;
                stepTimer.reset();

                // Initialize localization
                if (stateMachine.getLocalizationUnifier() != null) {
                    // Set starting position based on alliance and position
                    Pose2D startPos = getAdjustedStartPosition();
                    stateMachine.getLocalizationUnifier().setPosition(
                        startPos.getX(), startPos.getY(), startPos.getTheta());
                }

                blackboard.put("auto.current_step", currentStep.name());
                logger.info("BasicCenterAuto", "Autonomous initialization started");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Complete initialization after brief setup time
                if (getStateDuration() > 0.5) {
                    stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
                }
            }
        });

        // AUTO_RUNNING State - Main autonomous execution
        stateMachine.addState(new StateMachine.BaseState(StateMachine.State.AUTO_RUNNING) {
            @Override
            public void onEntry() {
                super.onEntry();
                logger.info("BasicCenterAuto", "Autonomous sequence started");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Execute current step in autonomous sequence
                executeCurrentStep();

                // Command DriveHandler to move toward target position
                commandDriveToTarget();

                // Check for timeout
                if (autonomousTimer.seconds() > MAX_AUTO_TIME) {
                    logger.warn("BasicCenterAuto", "Autonomous timeout reached");
                    stateMachine.triggerEvent(StateMachine.Event.AUTO_COMPLETE);
                }

                // Check for emergency conditions
                if (emergencyStop || isStopRequested()) {
                    stateMachine.triggerEvent(StateMachine.Event.EMERGENCY_STOP);
                }
            }

            @Override
            public void onExit() {
                super.onExit();
                // Stop driving when exiting AUTO_RUNNING
                if (stateMachine.getDriveHandler() != null) {
                    stateMachine.getDriveHandler().setDriveInputs(0, 0, 0);
                }
            }
        });

        // AUTO_COMPLETE State - Autonomous completion
        stateMachine.addState(new StateMachine.BaseState(StateMachine.State.AUTO_COMPLETE) {
            @Override
            public void onEntry() {
                super.onEntry();
                currentStep = AutoStep.COMPLETE;

                // Stop all robot movement
                if (stateMachine.getDriveHandler() != null) {
                    stateMachine.getDriveHandler().emergencyStop();
                }

                // Log final statistics
                logAutonomousStatistics();

                logger.info("BasicCenterAuto", "Autonomous sequence completed");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();
                // Hold position and display final telemetry
                updateTelemetry();
            }
        });

        // Setup transitions AFTER states are defined
        // Use AUTO_START to go from IDLE to AUTO_INIT (avoids conflict with default SYSTEM_INIT)
        stateMachine.addTransition(StateMachine.State.IDLE,
                                 StateMachine.State.AUTO_INIT,
                                 StateMachine.Event.AUTO_START);

        stateMachine.addTransition(StateMachine.State.AUTO_INIT,
                                 StateMachine.State.AUTO_RUNNING,
                                 StateMachine.Event.START_DRIVING);

        stateMachine.addTransition(StateMachine.State.AUTO_RUNNING,
                                 StateMachine.State.AUTO_COMPLETE,
                                 StateMachine.Event.AUTO_COMPLETE);

        stateMachine.addTransition(StateMachine.State.AUTO_COMPLETE,
                                 StateMachine.State.IDLE,
                                 StateMachine.Event.SYSTEM_READY);
    }

    /**
     * Try to set start position from vision detection
     */
    private boolean trySetStartPositionFromVision() {
        try {
            // Get VisionLocalizer from RobotMap
            VisionLocalizer visionLocalizer = robotMap.getVisionLocalizer();
            if (visionLocalizer == null || !visionLocalizer.isInitialized()) {
                logger.warn("BasicCenterAuto", "VisionLocalizer not available or not initialized");
                return false;
            }

            telemetry.addData("Vision Start", "Searching for AprilTags...");
            telemetry.update();

            // Try to get start position from AprilTags
            final int maxAttempts = 30; // 3 seconds at 100ms intervals
            for (int attempt = 0; attempt < maxAttempts && opModeIsActive(); attempt++) {
                visionLocalizer.update();

                if (visionLocalizer.hasValidPosition()) {
                    double[] position = visionLocalizer.getCurrentPosition();
                    double confidence = visionLocalizer.getPositionConfidence();

                    // Only use high-confidence detections for start position
                    if (confidence >= Tunables.VISION_DETECTION_CONFIDENCE_THRESHOLD) {
                        // Set localization unifier position
                        if (stateMachine.getLocalizationUnifier() != null) {
                            stateMachine.getLocalizationUnifier().setPosition(
                                position[0], position[1], position[2]);
                        }

                        // Store vision-based position in blackboard
                        blackboard.put("auto.start_x", position[0]);
                        blackboard.put("auto.start_y", position[1]);
                        blackboard.put("auto.start_heading", position[2]);
                        blackboard.put("auto.alliance", ALLIANCE.name());
                        blackboard.put("auto.start_position", START_POSITION.name());
                        blackboard.put("auto.vision_start", true);
                        blackboard.put("auto.vision_confidence", confidence);

                        // Update internal tracking
                        robotX = position[0];
                        robotY = position[1];
                        robotHeading = position[2];

                        telemetry.addData("Vision Start", "✅ DETECTED");
                        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°",
                            position[0], position[1], position[2]);
                        telemetry.addData("Confidence", "%.2f", confidence);
                        telemetry.update();

                        logger.info("BasicCenterAuto", String.format(java.util.Locale.US,
                            "Vision start position set: (%.1f, %.1f, %.1f°) confidence: %.2f",
                            position[0], position[1], position[2], confidence));

                        return true;
                    } else {
                        telemetry.addData("Vision Start", "Low confidence: %.2f", confidence);
                    }
                } else {
                    telemetry.addData("Vision Start", "Searching... %d/%d", attempt + 1, maxAttempts);
                }

                telemetry.addData("Vision Status", visionLocalizer.getStatusSummary());
                telemetry.update();
                sleep(100);
            }

            telemetry.addData("Vision Start", "❌ TIMEOUT - No reliable detection");
            telemetry.update();
            logger.warn("BasicCenterAuto", "Vision start position detection timed out");

        } catch (Exception e) {
            logger.error("BasicCenterAuto", "Vision start position detection failed: " + e.getMessage());
            telemetry.addData("Vision Start", "❌ ERROR - " + e.getMessage());
            telemetry.update();
        }

        return false;
    }

    private void setInitialPosition() {
        Pose2D startPos = getAdjustedStartPosition();

        // Store starting position in blackboard
        blackboard.put("auto.start_x", startPos.getX());
        blackboard.put("auto.start_y", startPos.getY());
        blackboard.put("auto.start_heading", startPos.getTheta());
        blackboard.put("auto.alliance", ALLIANCE.name());
        blackboard.put("auto.start_position", START_POSITION.name());

        logger.info("BasicCenterAuto", String.format(java.util.Locale.US,
            "Starting position set: (%.1f, %.1f, %.1f°)",
            startPos.getX(), startPos.getY(), startPos.getTheta()));
    }

    /**
     * Get starting position adjusted for alliance color
     */
    private Pose2D getAdjustedStartPosition() {
        Pose2D basePos = START_POSITION.getPosition();

        // Adjust position based on alliance (mirror for blue alliance)
        if (ALLIANCE == Alliance.BLUE) {
            return new Pose2D(
                Tunables.FIELD_WIDTH - basePos.getX(),
                basePos.getY(),
                180 - basePos.getTheta()
            );
        }

        return basePos;
    }

    // =========================================================================================
    // AUTONOMOUS SEQUENCE EXECUTION
    // =========================================================================================

    /**
     * Run the main autonomous sequence
     */
    private void runAutonomousSequence() {
        // Start the state machine
        stateMachine.start();

        // Directly transition to AUTO_INIT state using the AUTO_START event
        // We don't use SYSTEM_INIT because it conflicts with default transitions
        stateMachine.triggerEvent(StateMachine.Event.AUTO_START);

        // Main autonomous loop
        while (opModeIsActive() && !emergencyStop) {
            // Update state machine
            stateMachine.update();

            // Update position tracking
            updatePositionTracking();

            // Update telemetry
            updateTelemetry();

            // Check for completion
            if (stateMachine.getCurrentState() == StateMachine.State.AUTO_COMPLETE) {
                break;
            }

            // Brief pause to prevent CPU overload
            sleep(50); // 20Hz update rate
        }
    }

    /**
     * Update position tracking from localization system
     */
    private void updatePositionTracking() {
        if (stateMachine.getLocalizationUnifier() != null) {
            // Get current position as array and convert to Pose2D
            double[] posArray = stateMachine.getLocalizationUnifier().getCurrentPosition();
            if (posArray != null && posArray.length >= 3) {
                robotX = posArray[0];
                robotY = posArray[1];
                robotHeading = posArray[2];
            }
        }
    }

    /**
     * Command DriveHandler to move toward target position
     * Uses simple proportional control for autonomous navigation
     */
    private void commandDriveToTarget() {
        if (stateMachine.getDriveHandler() == null) {
            return;
        }

        // Get target position from blackboard
        double targetX = blackboard.get("drive.target.x", robotX);
        double targetY = blackboard.get("drive.target.y", robotY);
        double targetHeading = blackboard.get("drive.target.heading", robotHeading);

        // Calculate error
        double errorX = targetX - robotX;
        double errorY = targetY - robotY;
        double errorHeading = normalizeAngle(targetHeading - robotHeading);

        // Calculate distance to target
        double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);

        // If we're close enough, stop moving
        if (distanceError < Tunables.AUTO_POSITION_TOLERANCE &&
            Math.abs(errorHeading) < Tunables.AUTO_HEADING_TOLERANCE) {
            stateMachine.getDriveHandler().setDriveInputs(0, 0, 0);
            return;
        }

        // Simple proportional control
        double kP_translation = 0.05;  // Proportional gain for translation
        double kP_rotation = 0.02;      // Proportional gain for rotation

        // Calculate drive commands (field-relative)
        double commandX = errorX * kP_translation;
        double commandY = errorY * kP_translation;
        double commandRotation = errorHeading * kP_rotation;

        // Limit max speed
        double maxSpeed = 0.6;  // 60% max speed for autonomous
        double commandMagnitude = Math.sqrt(commandX * commandX + commandY * commandY);
        if (commandMagnitude > maxSpeed) {
            commandX = (commandX / commandMagnitude) * maxSpeed;
            commandY = (commandY / commandMagnitude) * maxSpeed;
        }

        // Limit rotation speed
        double maxRotation = 0.4;  // 40% max rotation for autonomous
        commandRotation = Math.max(-maxRotation, Math.min(maxRotation, commandRotation));

        // Send commands to DriveHandler
        // Note: setDriveInputs expects (axial=forward/back, lateral=left/right, yaw=rotation)
        stateMachine.getDriveHandler().setDriveInputs(commandY, commandX, commandRotation);
    }

    /**
     * Normalize angle to [-180, 180] degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Execute the current step in the autonomous sequence
     */
    private void executeCurrentStep() {
        switch (currentStep) {
            case INITIALIZE:
                // Already handled in state machine
                break;

            case DRIVE_TO_CENTER:
                executeDriveToCenter();
                break;

            case DRIVE_PLUS_24X:
                executeDrivePlus24X();
                break;

            case COMPLETE:
                // Sequence complete
                stateMachine.triggerEvent(StateMachine.Event.AUTO_COMPLETE);
                break;
        }
    }

    /**
     * Execute drive to center (0,0) step
     */
    private void executeDriveToCenter() {
        if (stepTimer.seconds() < 0.1) {
            logger.info("BasicCenterAuto", "Driving to center position (0,0)...");

            // Command drive to center position
            blackboard.put("drive.target.x", CENTER_POSITION.getX());
            blackboard.put("drive.target.y", CENTER_POSITION.getY());
            blackboard.put("drive.target.heading", CENTER_POSITION.getTheta());

            stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
        }

        // Check if arrived at center position
        if (isAtTarget() || stepTimer.seconds() > 10.0) {
            logger.info("BasicCenterAuto", "Arrived at center position");
            advanceToNextStep(AutoStep.DRIVE_PLUS_24X);
        }
    }

    /**
     * Execute drive +24 inches on X-axis step
     */
    private void executeDrivePlus24X() {
        if (stepTimer.seconds() < 0.1) {
            logger.info("BasicCenterAuto", "Driving +24 inches on X-axis to (24,0)...");

            // Command drive to +24X position
            blackboard.put("drive.target.x", PLUS_24X_POSITION.getX());
            blackboard.put("drive.target.y", PLUS_24X_POSITION.getY());
            blackboard.put("drive.target.heading", PLUS_24X_POSITION.getTheta());

            stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
        }

        // Check if arrived at +24X position
        if (isAtTarget() || stepTimer.seconds() > 10.0) {
            logger.info("BasicCenterAuto", "Arrived at +24X position");
            advanceToNextStep(AutoStep.COMPLETE);
        }
    }

    // =========================================================================================
    // UTILITY METHODS
    // =========================================================================================

    /**
     * Advance to the next step in the autonomous sequence
     */
    private void advanceToNextStep(AutoStep nextStep) {
        logger.info("BasicCenterAuto", String.format(java.util.Locale.US,
            "Step transition: %s -> %s (%.2fs)",
            currentStep.name(), nextStep.name(), stepTimer.seconds()));

        currentStep = nextStep;
        stepTimer.reset();
        blackboard.put("auto.current_step", currentStep.name());
    }

    /**
     * Check if robot is at the target position
     */
    private boolean isAtTarget() {
        // Get target position from blackboard
        double targetX = blackboard.get("drive.target.x", 0.0);
        double targetY = blackboard.get("drive.target.y", 0.0);
        double targetHeading = blackboard.get("drive.target.heading", 0.0);

        // Check if within tolerance using current position tracking
        double distanceError = Math.sqrt(Math.pow(robotX - targetX, 2) +
                                       Math.pow(robotY - targetY, 2));
        double headingError = Math.abs(robotHeading - targetHeading);

        return distanceError < Tunables.AUTO_POSITION_TOLERANCE &&
               headingError < Tunables.AUTO_HEADING_TOLERANCE;
    }

    /**
     * Handle emergency stop condition
     */
    private void handleEmergencyStop(String reason) {
        emergencyStop = true;
        logger.error("BasicCenterAuto", "Emergency stop: " + reason);

        // Stop all subsystems
        if (stateMachine != null) {
            stateMachine.triggerEvent(StateMachine.Event.EMERGENCY_STOP);
        }

        telemetry.addData("EMERGENCY", reason);
        telemetry.update();
    }

    // =========================================================================================
    // TELEMETRY AND MONITORING
    // =========================================================================================

    /**
     * Display initialization information while waiting for start
     */
    private void displayInitializationInfo() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.clear();
            telemetry.addData("=== BASIC CENTER AUTO ===", "");
            telemetry.addData("Alliance", ALLIANCE.getName());
            telemetry.addData("Start Position", START_POSITION.getName());
            telemetry.addData("Max Runtime", String.format(java.util.Locale.US, "%.1fs", MAX_AUTO_TIME));
            telemetry.addData("Debug Mode", DEBUG_MODE ? "ENABLED" : "DISABLED");
            telemetry.addData("", "");
            telemetry.addData("Sequence", "Start → Center(0,0) → +24X(24,0)");
            telemetry.addData("", "");
            telemetry.addData("Status", "Ready to start!");
            telemetry.addData("", "Press PLAY to begin autonomous");

            // Add system status
            if (stateMachine != null) {
                telemetry.addData("State Machine", "Initialized");
                telemetry.addData("Current State", stateMachine.getCurrentState().name());
            }

            telemetry.update();
            sleep(100);
        }
    }

    /**
     * Update telemetry during autonomous execution
     */
    private void updateTelemetry() {
        telemetry.clear();

        // Header
        telemetry.addData("=== BASIC CENTER AUTO ===", "");
        telemetry.addData("Alliance", ALLIANCE.getName());
        telemetry.addData("Runtime", String.format(java.util.Locale.US, "%.1fs / %.1fs",
                         autonomousTimer.seconds(), MAX_AUTO_TIME));

        // Current status
        telemetry.addData("Current Step", currentStep.getDescription());
        telemetry.addData("Step Time", String.format(java.util.Locale.US, "%.1fs", stepTimer.seconds()));

        // State machine status
        if (stateMachine != null) {
            telemetry.addData("State", stateMachine.getCurrentState().getShortName());
            telemetry.addData("State Duration", String.format(java.util.Locale.US, "%.1fs",
                             stateMachine.getCurrentStateDuration()));
        }

        // Position information
        telemetry.addData("Current Position", String.format(java.util.Locale.US,
                         "(%.1f, %.1f, %.0f°)", robotX, robotY, robotHeading));

        // Target information
        double targetX = blackboard.get("drive.target.x", 0.0);
        double targetY = blackboard.get("drive.target.y", 0.0);
        if (targetX != 0.0 || targetY != 0.0 || currentStep != AutoStep.INITIALIZE) {
            telemetry.addData("Target Position", String.format(java.util.Locale.US, "(%.1f, %.1f)", targetX, targetY));
            double distanceToTarget = Math.sqrt(
                Math.pow(robotX - targetX, 2) + Math.pow(robotY - targetY, 2)
            );
            telemetry.addData("Distance to Target", String.format(java.util.Locale.US, "%.1f\"", distanceToTarget));
            telemetry.addData("At Target", isAtTarget() ? "YES" : "NO");
        }

        // Progress indicator
        String progress = "";
        switch (currentStep) {
            case INITIALIZE:
                progress = "🔄 Initializing...";
                break;
            case DRIVE_TO_CENTER:
                progress = "🎯 → Center (0,0)";
                break;
            case DRIVE_PLUS_24X:
                progress = "🎯 → +24X (24,0)";
                break;
            case COMPLETE:
                progress = "✅ Complete!";
                break;
        }
        telemetry.addData("Progress", progress);

        // Debug information
        if (DEBUG_MODE && stateMachine != null) {
            telemetry.addData("=== DEBUG INFO ===", "");
            telemetry.addData("State Machine Stats", stateMachine.getStatistics());
        }

        telemetry.update();
    }

    /**
     * Log final autonomous statistics
     */
    private void logAutonomousStatistics() {
        String stats = String.format(java.util.Locale.US,
            "Basic Center Auto Statistics:\n" +
            "  Runtime: %.2f seconds\n" +
            "  Final Step: %s\n" +
            "  Final Position: (%.1f, %.1f, %.1f°)\n" +
            "  Alliance: %s %s",
            autonomousTimer.seconds(),
            currentStep.name(),
            robotX, robotY, robotHeading,
            ALLIANCE.getName(),
            START_POSITION.getName()
        );

        logger.info("BasicCenterAuto", stats);

        // Store statistics in blackboard for post-match analysis
        blackboard.put("auto.final_runtime", autonomousTimer.seconds());
        blackboard.put("auto.final_x", robotX);
        blackboard.put("auto.final_y", robotY);
        blackboard.put("auto.final_heading", robotHeading);
        blackboard.put("auto.completed_successfully", currentStep == AutoStep.COMPLETE);
    }

    /**
     * Cleanup resources and stop all systems
     */
    private void cleanup() {
        try {
            if (stateMachine != null) {
                stateMachine.stop();
            }

            logger.info("BasicCenterAuto", "Autonomous cleanup completed");

        } catch (Exception e) {
            // Log error but don't throw - we're already cleaning up
            logger.error("BasicCenterAuto", "Error during cleanup: " + e.getMessage());
        }
    }
}

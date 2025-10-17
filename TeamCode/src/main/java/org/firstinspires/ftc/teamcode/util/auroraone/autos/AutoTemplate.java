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

/**
 * AURORA ONE - Autonomous Template
 *
 * This class serves as a comprehensive template for creating autonomous routines for the robot.
 * It provides a structured framework using the StateMachine for defining the sequence of actions
 * the robot should perform during the autonomous period.
 *
 * TEMPLATE FEATURES:
 * ==================
 * - StateMachine integration for robust state management
 * - Customizable autonomous sequences with waypoints
 * - Alliance color and starting position selection
 * - Comprehensive error handling and recovery
 * - Performance monitoring and telemetry
 * - Timeout protection and safety measures
 * - Modular action system for reusable behaviors
 *
 * USAGE INSTRUCTIONS:
 * ===================
 * 1. Copy this template and rename it (e.g., "RedLeftAuto.java")
 * 2. Update the @Autonomous annotation with your specific name
 * 3. Customize the autonomous sequence in setupAutonomousSequence()
 * 4. Modify starting positions and alliance-specific logic
 * 5. Add custom states and actions as needed
 * 6. Test thoroughly and tune timing/positions
 *
 * CUSTOMIZATION POINTS:
 * =====================
 * - Alliance color (RED/BLUE)
 * - Starting position (LEFT/RIGHT)
 * - Autonomous sequence steps
 * - Waypoint positions
 * - Action timing and parameters
 * - Scoring strategies
 * - Parking positions
 *
 * @author Tundra Tech Robotics
 * @version 1.0
 * @since 2025-10-16
 */
@Autonomous(name = "Aurora Auto Template", group = "Aurora", preselectTeleOp = "Aurora TeleOp")
public class AutoTemplate extends LinearOpMode {

    // =========================================================================================
    // CONFIGURATION CONSTANTS
    // =========================================================================================

    /**
     * Alliance color selection
     * Change this to BLUE for blue alliance
     */
    private static final Alliance ALLIANCE = Alliance.RED;

    /**
     * Starting position selection
     * Change this to RIGHT for right starting position
     */
    private static final StartPosition START_POSITION = StartPosition.LEFT;

    /**
     * Maximum autonomous runtime before timeout (seconds)
     */
    private static final double MAX_AUTO_TIME = 29.5; // Leave 0.5s buffer

    /**
     * Enable debug telemetry and logging
     */
    private static final boolean DEBUG_MODE = false;

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
        MOVE_TO_SCORING("Move to scoring position"),
        SCORE_PRELOAD("Score preloaded element"),
        COLLECT_ELEMENTS("Collect game elements"),
        SCORE_COLLECTED("Score collected elements"),
        PARK("Park in designated zone"),
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
    private int elementsCollected;
    private int elementsScored;
    private boolean emergencyStop;

    // Current robot position tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Waypoints and positions (customize these for your field strategy)
    private final Pose2D[] waypoints = {
        new Pose2D(24, 72, 0),   // Scoring zone approach
        new Pose2D(48, 48, 45),  // Collection zone 1
        new Pose2D(72, 24, 90),  // Collection zone 2
        new Pose2D(96, 72, 180), // Alternative scoring position
        new Pose2D(120, 24, 270) // Parking zone
    };

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
            logger.info("AutoTemplate", "Waiting for start...");
            waitForStart();

            // Check if OpMode was stopped during init
            if (isStopRequested()) {
                return;
            }

            // Start autonomous timer
            autonomousTimer.reset();
            logger.info("AutoTemplate", String.format(java.util.Locale.US,
                "Starting %s %s autonomous", ALLIANCE.getName(), START_POSITION.getName()));

            // Run the autonomous sequence
            runAutonomousSequence();

        } catch (Exception e) {
            logger.error("AutoTemplate", "Autonomous failed: " + e.getMessage());
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

        // Initialize robot hardware - fixed constructor call
        robotMap = new RobotMap(hardwareMap, telemetry);

        // Initialize state machine
        stateMachine = StateMachine.getInstance();
        stateMachine.initialize(robotMap, telemetry);
        stateMachine.setDebugMode(DEBUG_MODE);

        // Setup autonomous-specific states
        setupAutonomousStates();

        // Configure initial position
        setInitialPosition();

        // Initialize tracking variables
        currentStep = AutoStep.INITIALIZE;
        elementsCollected = 0;
        elementsScored = 0;
        emergencyStop = false;

        logger.info("AutoTemplate", "Robot initialization complete");
    }

    /**
     * Setup custom autonomous states in the state machine
     */
    private void setupAutonomousStates() {
        // Add custom autonomous states to the state machine

        // AUTO_INIT State - Autonomous initialization
        stateMachine.addState(new StateMachine.BaseState(StateMachine.State.AUTO_INIT) {
            @Override
            public void onEntry() {
                super.onEntry();
                currentStep = AutoStep.INITIALIZE;
                stepTimer.reset();

                // Set initial robot configuration
                // stateMachine.getDriveHandler().setCurrentMode(
                //     stateMachine.getDriveHandler().DriveMode.NORMAL);

                // Initialize localization
                if (stateMachine.getLocalizationUnifier() != null) {
                    // Set starting position based on alliance and position
                    Pose2D startPos = getAdjustedStartPosition();
                    stateMachine.getLocalizationUnifier().setPosition(
                        startPos.getX(), startPos.getY(), startPos.getTheta());
                }

                blackboard.put("auto.current_step", currentStep.name());
                logger.info("AutoTemplate", "Autonomous initialization started");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Complete initialization after brief setup time
                if (getStateDuration() > 0.5) {
                    stateMachine.triggerEvent(StateMachine.Event.AUTO_START);
                }
            }
        });

        // AUTO_RUNNING State - Main autonomous execution
        stateMachine.addState(new StateMachine.BaseState(StateMachine.State.AUTO_RUNNING) {
            @Override
            public void onEntry() {
                super.onEntry();
                logger.info("AutoTemplate", "Autonomous sequence started");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();

                // Execute current step in autonomous sequence
                executeCurrentStep();

                // Check for timeout
                if (autonomousTimer.seconds() > MAX_AUTO_TIME) {
                    logger.warn("AutoTemplate", "Autonomous timeout reached");
                    stateMachine.triggerEvent(StateMachine.Event.AUTO_COMPLETE);
                }

                // Check for emergency conditions
                if (emergencyStop || isStopRequested()) {
                    stateMachine.triggerEvent(StateMachine.Event.EMERGENCY_STOP);
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
                // stateMachine.getDriveHandler().emergencyStop();
                // stateMachine.getShooterHandler().emergencyStop();

                // Log final statistics
                logAutonomousStatistics();

                logger.info("AutoTemplate", "Autonomous sequence completed");
            }

            @Override
            public void onUpdate() {
                super.onUpdate();
                // Hold position and display final telemetry
                updateTelemetry();
            }
        });

        // Setup transitions
        stateMachine.addTransition(StateMachine.State.AUTO_INIT,
                                 StateMachine.State.AUTO_RUNNING,
                                 StateMachine.Event.AUTO_START);

        stateMachine.addTransition(StateMachine.State.AUTO_RUNNING,
                                 StateMachine.State.AUTO_COMPLETE,
                                 StateMachine.Event.AUTO_COMPLETE);
    }

    /**
     * Set the initial robot position based on alliance and starting position
     */
    private void setInitialPosition() {
        Pose2D startPos = getAdjustedStartPosition();

        // Store starting position in blackboard
        blackboard.put("auto.start_x", startPos.getX());
        blackboard.put("auto.start_y", startPos.getY());
        blackboard.put("auto.start_heading", startPos.getTheta());
        blackboard.put("auto.alliance", ALLIANCE.name());
        blackboard.put("auto.start_position", START_POSITION.name());

        logger.info("AutoTemplate", String.format(java.util.Locale.US,
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
        stateMachine.triggerEvent(StateMachine.Event.SYSTEM_INIT);

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
     * Execute the current step in the autonomous sequence
     */
    private void executeCurrentStep() {
        switch (currentStep) {
            case INITIALIZE:
                // Already handled in state machine
                break;

            case MOVE_TO_SCORING:
                executeMoveToScoring();
                break;

            case SCORE_PRELOAD:
                executeScorePreload();
                break;

            case COLLECT_ELEMENTS:
                executeCollectElements();
                break;

            case SCORE_COLLECTED:
                executeScoreCollected();
                break;

            case PARK:
                executePark();
                break;

            case COMPLETE:
                // Sequence complete
                stateMachine.triggerEvent(StateMachine.Event.AUTO_COMPLETE);
                break;
        }
    }

    /**
     * Execute move to scoring position step
     */
    private void executeMoveToScoring() {
        if (stepTimer.seconds() < 0.1) {
            logger.info("AutoTemplate", "Moving to scoring position...");

            // Get scoring position based on alliance
            Pose2D scoringPos = getAdjustedWaypoint(0); // First waypoint is scoring zone

            // Command drive to scoring position
            blackboard.put("drive.target.x", scoringPos.getX());
            blackboard.put("drive.target.y", scoringPos.getY());
            blackboard.put("drive.target.heading", scoringPos.getTheta());

            stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
        }

        // Check if arrived at scoring position
        if (isAtTarget() || stepTimer.seconds() > 3.0) {
            advanceToNextStep(AutoStep.SCORE_PRELOAD);
        }
    }

    /**
     * Execute score preload step
     */
    private void executeScorePreload() {
        if (stepTimer.seconds() < 0.1) {
            logger.info("AutoTemplate", "Scoring preloaded element...");

            // Start shooter and prepare to score
            // stateMachine.getShooterHandler().setCurrentPreset(
            //     stateMachine.getShooterHandler().ShooterPreset.HIGH_GOAL);
            stateMachine.triggerEvent(StateMachine.Event.START_SHOOTING);
        }

        // Wait for shooting to complete - check if shooter handler is available
        boolean shootingComplete = true;
        if (stateMachine.getShooterHandler() != null) {
            shootingComplete = !stateMachine.getShooterHandler().isShooting();
        }

        if (shootingComplete || stepTimer.seconds() > 4.0) {
            elementsScored++;
            advanceToNextStep(AutoStep.COLLECT_ELEMENTS);
        }
    }

    /**
     * Execute collect elements step
     */
    private void executeCollectElements() {
        if (stepTimer.seconds() < 0.1) {
            logger.info("AutoTemplate", "Collecting game elements...");

            // Move to collection zone
            Pose2D collectionPos = getAdjustedWaypoint(1); // Second waypoint is collection zone

            blackboard.put("drive.target.x", collectionPos.getX());
            blackboard.put("drive.target.y", collectionPos.getY());
            blackboard.put("drive.target.heading", collectionPos.getTheta());

            stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
            stateMachine.triggerEvent(StateMachine.Event.START_COLLECTION);
        }

        // Check if collection is complete or timeout
        // Note: Using timeout for now since Collector.hasGameElement() doesn't exist yet
        if (stepTimer.seconds() > 5.0) {
            // Assume we collected something for demo purposes
            elementsCollected = 1;
            stateMachine.triggerEvent(StateMachine.Event.STOP_COLLECTION);
            advanceToNextStep(AutoStep.SCORE_COLLECTED);
        }
    }

    /**
     * Execute score collected elements step
     */
    private void executeScoreCollected() {
        if (elementsCollected == 0) {
            // Skip if no elements collected
            advanceToNextStep(AutoStep.PARK);
            return;
        }

        if (stepTimer.seconds() < 0.1) {
            logger.info("AutoTemplate", "Scoring collected elements...");

            // Return to scoring position
            Pose2D scoringPos = getAdjustedWaypoint(0);

            blackboard.put("drive.target.x", scoringPos.getX());
            blackboard.put("drive.target.y", scoringPos.getY());
            blackboard.put("drive.target.heading", scoringPos.getTheta());

            stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
        }

        // Start shooting when in position
        if (isAtTarget() && stepTimer.seconds() > 2.0) {
            stateMachine.triggerEvent(StateMachine.Event.START_SHOOTING);
        }

        // Check if shooting is complete
        boolean shootingComplete = true;
        if (stateMachine.getShooterHandler() != null) {
            shootingComplete = !stateMachine.getShooterHandler().isShooting();
        }

        if (shootingComplete || stepTimer.seconds() > 8.0) {
            elementsScored += elementsCollected;
            advanceToNextStep(AutoStep.PARK);
        }
    }

    /**
     * Execute park step
     */
    private void executePark() {
        if (stepTimer.seconds() < 0.1) {
            logger.info("AutoTemplate", "Parking robot...");

            // Move to parking position
            Pose2D parkingPos = getAdjustedWaypoint(4); // Last waypoint is parking zone

            blackboard.put("drive.target.x", parkingPos.getX());
            blackboard.put("drive.target.y", parkingPos.getY());
            blackboard.put("drive.target.heading", parkingPos.getTheta());

            stateMachine.triggerEvent(StateMachine.Event.START_DRIVING);
        }

        // Check if parked or timeout
        if (isAtTarget() || stepTimer.seconds() > 4.0) {
            logger.info("AutoTemplate", "Parking complete");
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
        logger.info("AutoTemplate", String.format(java.util.Locale.US,
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
     * Get waypoint adjusted for alliance color
     */
    private Pose2D getAdjustedWaypoint(int index) {
        if (index >= waypoints.length) {
            return waypoints[waypoints.length - 1];
        }

        Pose2D waypoint = waypoints[index];

        // Adjust for blue alliance (mirror across field center)
        if (ALLIANCE == Alliance.BLUE) {
            return new Pose2D(
                Tunables.FIELD_WIDTH - waypoint.getX(),
                waypoint.getY(),
                180 - waypoint.getTheta()
            );
        }

        return waypoint;
    }

    /**
     * Handle emergency stop condition
     */
    private void handleEmergencyStop(String reason) {
        emergencyStop = true;
        logger.error("AutoTemplate", "Emergency stop: " + reason);

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
            telemetry.addData("=== AURORA ONE AUTONOMOUS ===", "");
            telemetry.addData("Alliance", ALLIANCE.getName());
            telemetry.addData("Start Position", START_POSITION.getName());
            telemetry.addData("Max Runtime", String.format(java.util.Locale.US, "%.1fs", MAX_AUTO_TIME));
            telemetry.addData("Debug Mode", DEBUG_MODE ? "ENABLED" : "DISABLED");
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
        telemetry.addData("=== AURORA ONE AUTO ===", "");
        telemetry.addData("Alliance", ALLIANCE.getName());
        telemetry.addData("Runtime", String.format(java.util.Locale.US, "%.1fs / %.1fs",
                         autonomousTimer.seconds(), MAX_AUTO_TIME));

        // Current status
        telemetry.addData("Current Step", currentStep.getDescription());
        telemetry.addData("Step Time", String.format(java.util.Locale.US, "%.1fs", stepTimer.seconds()));

        // Performance metrics
        telemetry.addData("Elements Collected", elementsCollected);
        telemetry.addData("Elements Scored", elementsScored);

        // State machine status
        if (stateMachine != null) {
            telemetry.addData("State", stateMachine.getCurrentState().getShortName());
            telemetry.addData("State Duration", String.format(java.util.Locale.US, "%.1fs",
                             stateMachine.getCurrentStateDuration()));
        }

        // Position information
        telemetry.addData("Position", String.format(java.util.Locale.US,
                         "(%.1f, %.1f, %.0f°)", robotX, robotY, robotHeading));

        // Target information
        double targetX = blackboard.get("drive.target.x", 0.0);
        double targetY = blackboard.get("drive.target.y", 0.0);
        if (targetX != 0.0 || targetY != 0.0) {
            telemetry.addData("Target", String.format(java.util.Locale.US, "(%.1f, %.1f)", targetX, targetY));
            double distanceToTarget = Math.sqrt(
                Math.pow(robotX - targetX, 2) + Math.pow(robotY - targetY, 2)
            );
            telemetry.addData("Distance to Target", String.format(java.util.Locale.US, "%.1f\"", distanceToTarget));
            telemetry.addData("At Target", isAtTarget() ? "YES" : "NO");
        }

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
            "Autonomous Statistics:\n" +
            "  Runtime: %.2f seconds\n" +
            "  Elements Collected: %d\n" +
            "  Elements Scored: %d\n" +
            "  Final Step: %s\n" +
            "  Alliance: %s %s",
            autonomousTimer.seconds(),
            elementsCollected,
            elementsScored,
            currentStep.name(),
            ALLIANCE.getName(),
            START_POSITION.getName()
        );

        logger.info("AutoTemplate", stats);

        // Store statistics in blackboard for post-match analysis
        blackboard.put("auto.final_runtime", autonomousTimer.seconds());
        blackboard.put("auto.elements_collected", elementsCollected);
        blackboard.put("auto.elements_scored", elementsScored);
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

            logger.info("AutoTemplate", "Autonomous cleanup completed");

        } catch (Exception e) {
            // Log error but don't throw - we're already cleaning up
            logger.error("AutoTemplate", "Error during cleanup: " + e.getMessage());
        }
    }
}

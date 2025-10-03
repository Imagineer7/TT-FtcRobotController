package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanningSystem;

import java.util.ArrayList;
import java.util.List;

//
// Used to manage overall autonomous routines and strategies for the robot
// Integrates with SensorManager and AuroraPositioningManager
public class AuroraAutoManager {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Subsystem managers
    private AuroraPositioningManager positionManager;
    private ActionManager actionManager;
    private SensorManager sensorManager;
    private VisionManager visionManager;

    // Autonomous state management
    private ElapsedTime autonomousTimer;
    private AutonomousState currentState;
    private FieldMap.Alliance alliance;
    private StartingPosition startingPosition;

    // Strategy configuration
    private boolean useVision = true;
    private boolean useObstacleAvoidance = true;
    private double safetyTimeout = 28.0; // seconds

    // Autonomous states
    public enum AutonomousState {
        INIT,
        MOVE_TO_SCORING,
        SCORE_PRELOAD,
        NAVIGATE_TO_SAMPLES,
        COLLECT_SAMPLE,
        SCORE_SAMPLE,
        PARK,
        EMERGENCY_STOP,
        COMPLETE
    }

    // Starting positions
    public enum StartingPosition {
        OBSERVATION_ZONE,
        NET_ZONE,
        BASKET_ZONE
    }

    /**
     * Initialize Aurora autonomous manager
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param alliance Robot alliance (RED or BLUE)
     * @param startingPos Starting position on field
     */
    public AuroraAutoManager(HardwareMap hardwareMap, Telemetry telemetry,
                           FieldMap.Alliance alliance, StartingPosition startingPos) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.startingPosition = startingPos;

        autonomousTimer = new ElapsedTime();
        currentState = AutonomousState.INIT;

        initializeSubsystems();

        telemetry.addData("AuroraAutoManager", "Initialized for %s alliance from %s",
                         alliance, startingPos);
    }

    /**
     * Initialize all subsystem managers
     */
    private void initializeSubsystems() {
        // Initialize positioning manager with vision if available
        try {
            positionManager = new AuroraPositioningManager(hardwareMap, telemetry,
                                                         alliance, true, "Webcam 1");
            useVision = true;
            telemetry.addData("AuroraAutoManager", "Vision system enabled");
        } catch (Exception e) {
            positionManager = new AuroraPositioningManager(hardwareMap, telemetry, alliance);
            useVision = false;
            telemetry.addData("AuroraAutoManager", "Vision system disabled");
        }

        // Initialize action manager
        actionManager = new ActionManager(hardwareMap, telemetry, positionManager);

        // Initialize sensor manager
        sensorManager = new SensorManager(hardwareMap, telemetry);

        // Set starting position based on configuration
        setStartingPosition();

        telemetry.addData("AuroraAutoManager", "All subsystems initialized");
    }

    /**
     * Set robot starting position on field
     */
    private void setStartingPosition() {
        double startX, startY, startHeading;

        // Define starting positions based on alliance and position
        switch (startingPosition) {
            case OBSERVATION_ZONE:
                if (alliance == FieldMap.Alliance.BLUE) {
                    startX = -36; startY = 62; startHeading = 180;
                } else {
                    startX = 36; startY = -62; startHeading = 0;
                }
                break;

            case NET_ZONE:
                if (alliance == FieldMap.Alliance.BLUE) {
                    startX = 12; startY = 62; startHeading = 180;
                } else {
                    startX = -12; startY = -62; startHeading = 0;
                }
                break;

            case BASKET_ZONE:
            default:
                if (alliance == FieldMap.Alliance.BLUE) {
                    startX = 36; startY = 62; startHeading = 225;
                } else {
                    startX = -36; startY = -62; startHeading = 45;
                }
                break;
        }

        positionManager.setStartingPosition(startX, startY, startHeading);

        telemetry.addData("Starting Position", "(%.1f, %.1f) @ %.0f°",
                         startX, startY, startHeading);
    }

    /**
     * Main autonomous execution loop
     * Call this repeatedly in your autonomous OpMode
     */
    public void runAutonomous() {
        // Update all subsystems
        updateSubsystems();

        // Check for emergency conditions
        if (checkEmergencyConditions()) {
            currentState = AutonomousState.EMERGENCY_STOP;
        }

        // Execute current state
        switch (currentState) {
            case INIT:
                executeInitState();
                break;
            case MOVE_TO_SCORING:
                executeMoveToScoringState();
                break;
            case SCORE_PRELOAD:
                executeScorePreloadState();
                break;
            case NAVIGATE_TO_SAMPLES:
                executeNavigateToSamplesState();
                break;
            case COLLECT_SAMPLE:
                executeCollectSampleState();
                break;
            case SCORE_SAMPLE:
                executeScoreSampleState();
                break;
            case PARK:
                executeParkState();
                break;
            case EMERGENCY_STOP:
                executeEmergencyStopState();
                break;
            case COMPLETE:
                // Autonomous is complete
                break;
        }

        // Add telemetry
        addAutonomousTelemetry();
    }

    /**
     * Update all subsystem managers
     */
    private void updateSubsystems() {
        positionManager.updatePosition();
        sensorManager.updateSensors();

        // Update vision if available
        if (useVision && visionManager != null) {
            visionManager.updateVision();
        }
    }

    /**
     * Check for emergency stop conditions
     */
    private boolean checkEmergencyConditions() {
        // Check timeout
        if (autonomousTimer.seconds() > safetyTimeout) {
            telemetry.addData("EMERGENCY", "Autonomous timeout reached");
            return true;
        }

        // Check if robot is stuck
        if (actionManager.isMoving() && getCurrentSpeed() < 0.1) {
            telemetry.addData("EMERGENCY", "Robot appears stuck");
            return true;
        }

        // Check for unexpected obstacles
        if (useObstacleAvoidance && sensorManager.isNearWall(4.0)) {
            telemetry.addData("WARNING", "Unexpected obstacle detected");
            // Don't emergency stop, just slow down
        }

        return false;
    }

    /**
     * Execute initialization state
     */
    private void executeInitState() {
        telemetry.addData("State", "INITIALIZING");

        // Wait for all systems to be ready
        if (positionManager.isPositionValid() &&
            sensorManager.areSensorsReady() &&
            autonomousTimer.seconds() > 0.5) {

            currentState = AutonomousState.MOVE_TO_SCORING;
            telemetry.addData("State", "Initialization complete");
        }
    }

    /**
     * Execute move to scoring state
     */
    private void executeMoveToScoringState() {
        telemetry.addData("State", "MOVING TO SCORING POSITION");

        // Get scoring position from field map
        double[] scoringPos = positionManager.getFieldPosition("HIGH_BASKET");
        if (scoringPos != null) {
            // Move to scoring position
            if (actionManager.moveToPosition(scoringPos[0] - 12, scoringPos[1], 45, 8.0)) {
                currentState = AutonomousState.SCORE_PRELOAD;
            }
        } else {
            // Fallback position
            if (actionManager.moveToPosition(24, 24, 45, 8.0)) {
                currentState = AutonomousState.SCORE_PRELOAD;
            }
        }
    }

    /**
     * Execute score preload state
     */
    private void executeScorePreloadState() {
        telemetry.addData("State", "SCORING PRELOAD");

        // Raise arm and score preload
        if (actionManager.performAction("score_high")) {
            currentState = AutonomousState.NAVIGATE_TO_SAMPLES;
        }
    }

    /**
     * Execute navigate to samples state
     */
    private void executeNavigateToSamplesState() {
        telemetry.addData("State", "NAVIGATING TO SAMPLES");

        // Move to sample collection area
        double[] samplePos = positionManager.getFieldPosition("SAMPLE_AREA");
        if (samplePos != null) {
            if (actionManager.moveToPosition(samplePos[0], samplePos[1], 0, 10.0)) {
                currentState = AutonomousState.COLLECT_SAMPLE;
            }
        } else {
            // Skip to parking if no samples defined
            currentState = AutonomousState.PARK;
        }
    }

    /**
     * Execute collect sample state
     */
    private void executeCollectSampleState() {
        telemetry.addData("State", "COLLECTING SAMPLE");

        // Look for samples using vision or sensors
        if (useVision && visionManager != null) {
            // Use vision to find and collect samples
            if (collectSampleWithVision()) {
                currentState = AutonomousState.SCORE_SAMPLE;
            } else {
                currentState = AutonomousState.PARK;
            }
        } else {
            // Use basic collection routine
            if (actionManager.performAction("grab")) {
                currentState = AutonomousState.SCORE_SAMPLE;
            } else {
                currentState = AutonomousState.PARK;
            }
        }
    }

    /**
     * Execute score sample state
     */
    private void executeScoreSampleState() {
        telemetry.addData("State", "SCORING SAMPLE");

        // Move back to scoring position and score
        double[] scoringPos = positionManager.getFieldPosition("HIGH_BASKET");
        if (scoringPos != null) {
            if (actionManager.moveToPosition(scoringPos[0] - 12, scoringPos[1], 45, 8.0)) {
                if (actionManager.performAction("score_high")) {
                    currentState = AutonomousState.PARK;
                }
            }
        } else {
            currentState = AutonomousState.PARK;
        }
    }

    /**
     * Execute park state
     */
    private void executeParkState() {
        telemetry.addData("State", "PARKING");

        // Move to observation zone for parking
        double[] parkPos = positionManager.getFieldPosition("OBSERVATION_ZONE");
        if (parkPos != null) {
            if (actionManager.moveToPosition(parkPos[0], parkPos[1], 90, 8.0)) {
                actionManager.performAction("park");
                currentState = AutonomousState.COMPLETE;
            }
        } else {
            // Emergency park in place
            actionManager.stopDriving();
            actionManager.performAction("park");
            currentState = AutonomousState.COMPLETE;
        }
    }

    /**
     * Execute emergency stop state
     */
    private void executeEmergencyStopState() {
        telemetry.addData("State", "EMERGENCY STOP");

        // Stop all movement
        actionManager.stopDriving();
        actionManager.performAction("park");

        currentState = AutonomousState.COMPLETE;
    }

    /**
     * Collect sample using vision system
     */
    private boolean collectSampleWithVision() {
        // This would integrate with vision processing to find and collect samples
        // For now, return basic success/failure
        return actionManager.performAction("grab");
    }

    /**
     * Get current robot speed
     */
    private double getCurrentSpeed() {
        double[] velocity = positionManager.getCurrentVelocity();
        return Math.sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
    }

    /**
     * Configure autonomous strategy
     * @param useVision Enable vision-based navigation
     * @param useObstacleAvoidance Enable obstacle avoidance
     * @param timeout Safety timeout in seconds
     */
    public void configureStrategy(boolean useVision, boolean useObstacleAvoidance, double timeout) {
        this.useVision = useVision;
        this.useObstacleAvoidance = useObstacleAvoidance;
        this.safetyTimeout = timeout;

        if (positionManager != null) {
            positionManager.setVisionCorrectionEnabled(useVision);
        }
    }

    /**
     * Check if autonomous is complete
     * @return True if autonomous has finished
     */
    public boolean isComplete() {
        return currentState == AutonomousState.COMPLETE;
    }

    /**
     * Get current autonomous state
     * @return Current state
     */
    public AutonomousState getCurrentState() {
        return currentState;
    }

    /**
     * Get elapsed time since autonomous start
     * @return Time in seconds
     */
    public double getElapsedTime() {
        return autonomousTimer.seconds();
    }

    /**
     * Emergency stop autonomous execution
     */
    public void emergencyStop() {
        currentState = AutonomousState.EMERGENCY_STOP;
    }

    /**
     * Add comprehensive autonomous telemetry
     */
    public void addAutonomousTelemetry() {
        telemetry.addData("=== AURORA AUTONOMOUS ===", "");
        telemetry.addData("State", currentState);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.addData("Elapsed Time", "%.1f sec", getElapsedTime());
        telemetry.addData("Vision Enabled", useVision ? "YES" : "NO");
        telemetry.addData("Obstacle Avoidance", useObstacleAvoidance ? "YES" : "NO");

        // Add subsystem telemetry
        if (positionManager != null) {
            positionManager.addTelemetry();
        }
        if (actionManager != null) {
            actionManager.addTelemetry();
        }
        if (sensorManager != null) {
            sensorManager.addTelemetry();
        }
        if (visionManager != null) {
            visionManager.addTelemetry();
        }
    }

    /**
     * Start autonomous timer
     * Call this at the beginning of autonomous
     */
    public void startAutonomous() {
        autonomousTimer.reset();
        currentState = AutonomousState.INIT;
        telemetry.addData("AuroraAutoManager", "Autonomous started");
    }
}

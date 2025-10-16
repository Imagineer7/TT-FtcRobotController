package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanningSystem;

import java.util.ArrayList;
import java.util.List;

// Used to manage overall autonomous routines and strategies for the robot
// Integrates seamlessly with the new Aurora Vision System and all positioning components
public class AuroraAutoManager {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Subsystem managers
    private AuroraPositioningManager positionManager;
    private ActionManager actionManager;
    private SensorManager sensorManager;

    // Autonomous state management
    private ElapsedTime autonomousTimer;
    private AutonomousState currentState;
    private FieldMap.Alliance alliance;
    private StartingPosition startingPosition;

    // Strategy configuration
    private boolean useVision = true;
    private boolean useObstacleAvoidance = true;
    private double safetyTimeout = 28.0; // seconds

    // Vision system configuration
    private String primaryCamera = "Webcam 1";
    private String secondaryCamera = null; // Set if dual cameras available

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
     * Initialize Aurora autonomous manager with single camera
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
     * Initialize Aurora autonomous manager with specific camera configuration
     */
    public AuroraAutoManager(HardwareMap hardwareMap, Telemetry telemetry,
                           FieldMap.Alliance alliance, StartingPosition startingPos,
                           String cameraName) {
        this.primaryCamera = cameraName;

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.startingPosition = startingPos;

        autonomousTimer = new ElapsedTime();
        currentState = AutonomousState.INIT;

        initializeSubsystems();

        telemetry.addData("AuroraAutoManager", "Initialized with camera: %s", cameraName);
    }

    /**
     * Initialize Aurora autonomous manager with dual cameras
     */
    public AuroraAutoManager(HardwareMap hardwareMap, Telemetry telemetry,
                           FieldMap.Alliance alliance, StartingPosition startingPos,
                           String primaryCameraName, String secondaryCameraName) {
        this.primaryCamera = primaryCameraName;
        this.secondaryCamera = secondaryCameraName;

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.startingPosition = startingPos;

        autonomousTimer = new ElapsedTime();
        currentState = AutonomousState.INIT;

        initializeSubsystems();

        telemetry.addData("AuroraAutoManager", "Initialized with dual cameras");
    }

    /**
     * Initialize all subsystem managers with new vision system integration
     */
    private void initializeSubsystems() {
        // Initialize positioning manager with the new vision system
        try {
            if (secondaryCamera != null) {
                // Dual camera setup
                positionManager = new AuroraPositioningManager(hardwareMap, telemetry,
                                                             alliance, primaryCamera, secondaryCamera);
            } else {
                // Single camera setup
                positionManager = new AuroraPositioningManager(hardwareMap, telemetry,
                                                             alliance, true, primaryCamera);
            }
            useVision = true;
            telemetry.addData("AuroraAutoManager", "New vision system enabled");
        } catch (Exception e) {
            // Fallback to encoder-only positioning
            positionManager = new AuroraPositioningManager(hardwareMap, telemetry, alliance);
            useVision = false;
            telemetry.addData("AuroraAutoManager", "Vision unavailable, using encoders only");
        }

        // Initialize action manager (enhanced with vision integration)
        actionManager = new ActionManager(hardwareMap, telemetry, positionManager);

        // Initialize sensor manager
        sensorManager = new SensorManager(hardwareMap, telemetry);

        // Set starting position based on configuration
        setStartingPosition();

        telemetry.addData("AuroraAutoManager", "All subsystems initialized");
    }

    /**
     * Set starting position based on alliance and starting zone
     */
    private void setStartingPosition() {
        double startX = 0, startY = 0, startHeading = 0;

        // Define starting positions based on alliance and zone
        switch (alliance) {
            case RED:
                switch (startingPosition) {
                    case OBSERVATION_ZONE:
                        startX = -24; startY = -63; startHeading = 90;
                        break;
                    case NET_ZONE:
                        startX = 0; startY = -63; startHeading = 90;
                        break;
                    case BASKET_ZONE:
                        startX = 24; startY = -63; startHeading = 90;
                        break;
                }
                break;
            case BLUE:
                switch (startingPosition) {
                    case OBSERVATION_ZONE:
                        startX = -24; startY = 63; startHeading = -90;
                        break;
                    case NET_ZONE:
                        startX = 0; startY = 63; startHeading = -90;
                        break;
                    case BASKET_ZONE:
                        startX = 24; startY = 63; startHeading = -90;
                        break;
                }
                break;
        }

        positionManager.setStartingPosition(startX, startY, startHeading);
        telemetry.addData("Starting Position", "(%.1f, %.1f) @ %.1f°", startX, startY, startHeading);
    }

    /**
     * Execute autonomous routine - NON-BLOCKING, call repeatedly
     * @return True if autonomous is complete
     */
    public boolean executeAutonomous() {
        // Update all systems (non-blocking)
        updateSystems();

        // Safety timeout check
        if (autonomousTimer.seconds() > safetyTimeout) {
            currentState = AutonomousState.EMERGENCY_STOP;
        }

        // Execute current state
        switch (currentState) {
            case INIT:
                return executeInitState();
            case MOVE_TO_SCORING:
                return executeMoveToScoringState();
            case SCORE_PRELOAD:
                return executeScorePreloadState();
            case NAVIGATE_TO_SAMPLES:
                return executeNavigateToSamplesState();
            case COLLECT_SAMPLE:
                return executeCollectSampleState();
            case SCORE_SAMPLE:
                return executeScoreSampleState();
            case PARK:
                return executeParkState();
            case EMERGENCY_STOP:
                return executeEmergencyStopState();
            case COMPLETE:
                return true; // Autonomous complete
        }

        return false; // Still running
    }

    /**
     * Update all subsystems - NON-BLOCKING
     */
    private void updateSystems() {
        if (positionManager != null) {
            positionManager.updatePosition();
        }
        if (sensorManager != null) {
            sensorManager.updateSensors();
        }
    }

    /**
     * Execute initialization state
     */
    private boolean executeInitState() {
        // Wait for systems to be ready
        if (!positionManager.isPositionValid()) {
            telemetry.addData("Init", "Waiting for position system...");
            return false; // Still initializing
        }

        if (useVision && !actionManager.isVisionAvailable()) {
            telemetry.addData("Init", "Waiting for vision system...");
            return false; // Still initializing vision
        }

        // Systems ready, advance to first action
        currentState = AutonomousState.MOVE_TO_SCORING;
        telemetry.addData("Init", "Complete - Starting autonomous");
        return false;
    }

    /**
     * Execute move to scoring state
     */
    private boolean executeMoveToScoringState() {
        double targetX = alliance == FieldMap.Alliance.RED ? -48 : 48;
        double targetY = alliance == FieldMap.Alliance.RED ? -48 : 48;
        double targetHeading = alliance == FieldMap.Alliance.RED ? 45 : 135;

        // Use non-blocking movement
        if (actionManager.moveToPositionNonBlocking(targetX, targetY, targetHeading, 10.0)) {
            currentState = AutonomousState.SCORE_PRELOAD;
            telemetry.addData("Move to Scoring", "Complete");
        } else {
            telemetry.addData("Move to Scoring", "In progress...");
        }
        return false;
    }

    /**
     * Execute score preload state
     */
    private boolean executeScorePreloadState() {
        if (actionManager.performAction("score")) {
            currentState = AutonomousState.NAVIGATE_TO_SAMPLES;
            telemetry.addData("Score Preload", "Complete");
        } else {
            telemetry.addData("Score Preload", "In progress...");
        }
        return false;
    }

    /**
     * Execute navigate to samples state
     */
    private boolean executeNavigateToSamplesState() {
        double targetX = 0;
        double targetY = alliance == FieldMap.Alliance.RED ? -24 : 24;
        double targetHeading = 0;

        if (actionManager.moveToPositionNonBlocking(targetX, targetY, targetHeading, 8.0)) {
            currentState = AutonomousState.COLLECT_SAMPLE;
            telemetry.addData("Navigate to Samples", "Complete");
        } else {
            telemetry.addData("Navigate to Samples", "In progress...");
        }
        return false;
    }

    /**
     * Execute collect sample state
     */
    private boolean executeCollectSampleState() {
        if (actionManager.performAction("grab")) {
            currentState = AutonomousState.SCORE_SAMPLE;
            telemetry.addData("Collect Sample", "Complete");
        } else {
            telemetry.addData("Collect Sample", "In progress...");
        }
        return false;
    }

    /**
     * Execute score sample state
     */
    private boolean executeScoreSampleState() {
        // Move back to scoring area
        double targetX = alliance == FieldMap.Alliance.RED ? -48 : 48;
        double targetY = alliance == FieldMap.Alliance.RED ? -48 : 48;

        if (actionManager.moveToPositionNonBlocking(targetX, targetY, 0, 10.0)) {
            if (actionManager.performAction("release")) {
                currentState = AutonomousState.PARK;
                telemetry.addData("Score Sample", "Complete");
            }
        } else {
            telemetry.addData("Score Sample", "Moving to score...");
        }
        return false;
    }

    /**
     * Execute park state
     */
    private boolean executeParkState() {
        // Move to parking area
        double parkX = alliance == FieldMap.Alliance.RED ? 24 : -24;
        double parkY = alliance == FieldMap.Alliance.RED ? -24 : 24;

        if (actionManager.moveToPositionNonBlocking(parkX, parkY, 0, 8.0)) {
            actionManager.performAction("park");
            currentState = AutonomousState.COMPLETE;
            telemetry.addData("Park", "Complete - Autonomous finished");
            return true; // Autonomous complete
        } else {
            telemetry.addData("Park", "Moving to park...");
        }
        return false;
    }

    /**
     * Execute emergency stop state
     */
    private boolean executeEmergencyStopState() {
        actionManager.stopDriving();
        telemetry.addData("Emergency Stop", "Safety timeout reached");
        currentState = AutonomousState.COMPLETE;
        return true;
    }

    /**
     * Get current autonomous state
     */
    public AutonomousState getCurrentState() {
        return currentState;
    }

    /**
     * Get elapsed autonomous time
     */
    public double getElapsedTime() {
        return autonomousTimer.seconds();
    }

    /**
     * Check if vision system is available
     */
    public boolean isVisionAvailable() {
        return useVision && actionManager.isVisionAvailable();
    }

    /**
     * Get number of detected AprilTags
     */
    public int getDetectedTagCount() {
        return actionManager.getDetectedTagCount();
    }

    /**
     * Get current robot position
     */
    public double[] getCurrentPosition() {
        return actionManager.getCurrentPosition();
    }

    /**
     * Set manual camera exposure for better vision
     */
    public boolean setManualExposure(int exposureMS, int gain) {
        return actionManager.setManualExposure(exposureMS, gain);
    }

    /**
     * Force state change (for testing or emergency)
     */
    public void setState(AutonomousState newState) {
        currentState = newState;
        telemetry.addData("State Change", "Forced to: " + newState);
    }

    /**
     * Add comprehensive telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== AURORA AUTO MANAGER ===", "");
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("Elapsed Time", "%.1f seconds", getElapsedTime());
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Starting Position", startingPosition.toString());
        telemetry.addData("Vision System", isVisionAvailable() ? "ACTIVE" : "DISABLED");

        if (isVisionAvailable()) {
            telemetry.addData("AprilTags Detected", getDetectedTagCount());
        }

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
    }

    /**
     * Start autonomous timer
     */
    public void start() {
        autonomousTimer.reset();
        currentState = AutonomousState.INIT;
        telemetry.addData("AuroraAutoManager", "Autonomous started");
    }

    /**
     * Stop autonomous and cleanup
     */
    public void stop() {
        if (actionManager != null) {
            actionManager.stopDriving();
            actionManager.close();
        }
        if (positionManager != null) {
            positionManager.close();
        }
        currentState = AutonomousState.COMPLETE;
        telemetry.addData("AuroraAutoManager", "Autonomous stopped");
    }
}

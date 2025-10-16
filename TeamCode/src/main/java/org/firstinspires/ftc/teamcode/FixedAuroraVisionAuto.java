package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.aurora.auto.AuroraPositioningManager;
import org.firstinspires.ftc.teamcode.util.aurora.auto.ActionManager;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Fixed Aurora Vision Auto - Using Aurora System Components
 *
 * This version uses the Aurora positioning and movement systems
 * instead of custom implementations.
 */
@Autonomous(name="Fixed Aurora Vision Auto", group="Aurora")
public class FixedAuroraVisionAuto extends LinearOpMode {

    // Aurora system components
    private AuroraPositioningManager positionManager;
    private ActionManager actionManager;

    // Timing and control
    private ElapsedTime autonomousTimer = new ElapsedTime();

    // Movement control settings
    private static final double MOVEMENT_TOLERANCE = 6.0; // inches - how close to center before stopping
    private static final double SEARCH_SPEED = 0.15; // Slow search speed

    // Autonomous states
    private enum AutoState {
        INITIALIZE,
        VISION_INIT_DELAY,
        SEARCH_FOR_TAGS,
        WAIT_BEFORE_SEARCH,
        MOVE_TO_CENTER,
        COMPLETE,
        EMERGENCY_STOP
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private static final double MAX_SEARCH_TIME = 8.0; // seconds to search before giving up
    private static final double WAIT_BEFORE_SEARCH_TIME = 1.0; // seconds to wait before starting to turn

    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry.addData("Status", "Initializing Fixed Aurora Vision Auto...");
        telemetry.addData("Debug", "Step 1: Starting initialization");
        telemetry.update();
        sleep(1000);

        // Initialize Aurora systems
        initializeAuroraSystems();

        // Wait for start
        telemetry.addData("Status", "Ready for start");
        telemetry.addData("Target", "Field Center (0,0)");
        telemetry.addData("Debug", "Step 2: Checking Aurora systems...");
        if (positionManager == null || actionManager == null) {
            telemetry.addData("WARNING", "Aurora systems failed to initialize - limited functionality");
            telemetry.addData("Position Manager", positionManager == null ? "NULL" : "OK");
            telemetry.addData("Action Manager", actionManager == null ? "NULL" : "OK");
        } else {
            telemetry.addData("Aurora Status", "Both systems initialized successfully");
        }
        telemetry.update();

        waitForStart();

        telemetry.addData("Debug", "Step 3: waitForStart() completed");
        telemetry.update();
        sleep(1000);

        if (opModeIsActive()) {
            telemetry.addData("Debug", "Step 4: OpMode is active - entering main loop");
            telemetry.update();
            sleep(1000);

            autonomousTimer.reset();
            stateTimer.reset();
            currentState = AutoState.INITIALIZE;

            telemetry.addData("Debug", "Step 5: Timers reset, state set to INITIALIZE");
            telemetry.update();
            sleep(1000);

            // Set starting position only if positioning manager is available
            if (positionManager != null) {
                try {
                    // DON'T set a starting position - let the positioning system get real data from AprilTags first
                    // positionManager.setStartingPosition(0, 48, 0); // Commented out - this was causing premature completion
                    telemetry.addData("Debug", "Step 6: Positioning manager ready - waiting for AprilTag data");
                } catch (Exception e) {
                    telemetry.addData("Error", "Step 6 FAILED: " + e.getMessage());
                }
                telemetry.update();
                sleep(1000);
            } else {
                telemetry.addData("Debug", "Step 6: Position manager is NULL - Aurora system failed to initialize");
                telemetry.update();
                sleep(2000);
            }

            telemetry.addData("Debug", "Step 7: Entering main autonomous loop");
            telemetry.update();
            sleep(1000);

            // Main autonomous loop
            int loopCount = 0;
            while (opModeIsActive() && currentState != AutoState.COMPLETE && currentState != AutoState.EMERGENCY_STOP) {
                loopCount++;

                try {
                    telemetry.addData("Loop Count", loopCount);
                    telemetry.addData("Current State", currentState.toString());

                    // Update position from Aurora system
                    if (positionManager != null) {
                        positionManager.updatePosition();
                        telemetry.addData("Position Update", "Called successfully");
                    } else {
                        telemetry.addData("Position Update", "SKIPPED - positionManager is null");
                    }

                    // Execute current state
                    executeCurrentState();

                    // Update telemetry
                    updateTelemetry();

                    // Safety check - stop if time exceeded
                    if (autonomousTimer.seconds() > 28.0) {
                        telemetry.addData("Debug", "TIMEOUT: 28 seconds exceeded");
                        currentState = AutoState.EMERGENCY_STOP;
                    }

                } catch (Exception e) {
                    telemetry.addData("RUNTIME ERROR", e.getMessage());
                    telemetry.addData("Error in State", currentState.toString());
                    telemetry.addData("Loop Count", loopCount);
                    telemetry.update();
                    sleep(5000); // Give time to read error
                    currentState = AutoState.EMERGENCY_STOP;
                }

                sleep(100); // Increased delay to see what's happening
            }

            telemetry.addData("Debug", "Main loop exited");
            telemetry.addData("Final State", currentState.toString());
            telemetry.addData("OpMode Active", opModeIsActive());
            telemetry.update();
            sleep(2000);

            // Final cleanup
            cleanup();
        } else {
            telemetry.addData("CRITICAL", "opModeIsActive() returned FALSE immediately!");
            telemetry.update();
            sleep(5000);
        }
    }

    /**
     * Initialize Aurora positioning and action systems
     */
    private void initializeAuroraSystems() {
        telemetry.addData("Debug", "Starting Aurora system initialization...");
        telemetry.update();

        try {
            telemetry.addData("Debug", "Initializing positioning manager...");
            telemetry.update();

            // Initialize positioning manager with vision
            positionManager = new AuroraPositioningManager(
                hardwareMap,
                telemetry,
                FieldMap.Alliance.BLUE, // Change to RED if needed
                true,
                "Webcam 1"
            );

            telemetry.addData("Debug", "Positioning manager created successfully");
            telemetry.update();
            sleep(1000); // Give time to see the message

            telemetry.addData("Debug", "Initializing action manager...");
            telemetry.update();

            // Initialize action manager for movement
            actionManager = new ActionManager(hardwareMap, telemetry, positionManager);

            telemetry.addData("Debug", "Action manager created successfully");
            telemetry.update();
            sleep(1000); // Give time to see the message

            telemetry.addData("Aurora Systems", "Initialized successfully");
            telemetry.update();
            sleep(1000); // Give time to see the message

        } catch (Exception e) {
            telemetry.addData("CRITICAL ERROR", "Aurora system failed to initialize");
            telemetry.addData("Error Message", e.getMessage());
            telemetry.addData("Error Type", e.getClass().getSimpleName());
            if (e.getCause() != null) {
                telemetry.addData("Root Cause", e.getCause().getMessage());
            }
            telemetry.update();
            sleep(5000); // Give plenty of time to read the error

            // Set components to null to prevent further issues
            positionManager = null;
            actionManager = null;
        }
    }

    /**
     * Execute the current autonomous state
     */
    private void executeCurrentState() {
        switch (currentState) {
            case INITIALIZE:
                // Start vision initialization delay
                currentState = AutoState.VISION_INIT_DELAY;
                stateTimer.reset();
                telemetry.addData("Action", "Initializing vision system...");
                break;

            case VISION_INIT_DELAY:
                // Wait for a short duration to allow vision system to initialize
                if (stateTimer.seconds() > 2.0) {
                    currentState = AutoState.SEARCH_FOR_TAGS;
                    stateTimer.reset();
                    telemetry.addData("Action", "Vision system initialization complete");
                }
                break;

            case SEARCH_FOR_TAGS:
                // Spin slowly to search for AprilTags
                spinToFindTags();

                // Check for detections using the SAME method as SimpleVisionAuto
                if (positionManager != null) {
                    // Check if we found any AprilTags using direct detection like SimpleVisionAuto
                    boolean foundTag = checkForDirectAprilTagDetection();

                    if (foundTag) {
                        // Found tag(s)! Initialize positioning and move to navigation state
                        boolean positionInitialized = positionManager.initializePositionFromVision();

                        if (positionInitialized) {
                            // Add a delay to see the initialization telemetry
                            telemetry.addData("Action", "Found AprilTag! Position initialized, moving to center");
                            telemetry.addData("DEBUG", "Position initialization successful - showing details...");
                            telemetry.update();
                            sleep(2000); // Give 2 seconds to see the positioning data

                            currentState = AutoState.MOVE_TO_CENTER;
                            if (actionManager != null) {
                                actionManager.stopDriving();
                            }
                        } else {
                            telemetry.addData("Action", "Found AprilTag but failed to initialize position - continuing search");
                        }
                    }
                }

                // Timeout check
                if (stateTimer.seconds() > MAX_SEARCH_TIME) {
                    currentState = AutoState.COMPLETE;
                    if (actionManager != null) {
                        actionManager.stopDriving();
                    }
                    telemetry.addData("Action", "Search timeout - stopping");
                }
                break;

            case WAIT_BEFORE_SEARCH:
                // Wait for a short duration before starting to search
                if (actionManager != null) {
                    actionManager.stopDriving();
                }
                if (stateTimer.seconds() > WAIT_BEFORE_SEARCH_TIME) {
                    currentState = AutoState.SEARCH_FOR_TAGS;
                    stateTimer.reset();
                    telemetry.addData("Action", "Wait complete - resuming search");
                }
                break;

            case MOVE_TO_CENTER:
                // Use Aurora system to navigate toward center
                if (positionManager != null && actionManager != null) {
                    navigateUsingAurora();
                } else {
                    telemetry.addData("Error", "Aurora systems not available for navigation");
                    currentState = AutoState.EMERGENCY_STOP;
                }
                break;

            case COMPLETE:
                if (actionManager != null) {
                    actionManager.stopDriving();
                }
                telemetry.addData("Status", "Autonomous complete");
                break;

            case EMERGENCY_STOP:
                if (actionManager != null) {
                    actionManager.stopDriving();
                }
                telemetry.addData("EMERGENCY", "Autonomous stopped");
                break;
        }
    }

    /**
     * Spin the robot slowly to search for AprilTags
     */
    private void spinToFindTags() {
        telemetry.addData("Action", "Spinning to find AprilTags");

        // Use Aurora's mecanum drive to spin slowly
        if (actionManager != null) {
            actionManager.mechanumDrive(0, 0, SEARCH_SPEED);
        } else {
            telemetry.addData("Error", "Action manager not available - cannot spin");
        }
    }

    /**
     * Navigate toward center using vision-aware movement that keeps tags in view
     * FIXED: Use lateral movements to maintain camera line of sight to AprilTags
     */
    private void navigateUsingAurora() {
        // Check if we have valid position data from Aurora
        if (!positionManager.isPositionValid()) {
            // Lost positioning - go to wait state
            actionManager.stopDriving();
            currentState = AutoState.WAIT_BEFORE_SEARCH;
            stateTimer.reset();
            telemetry.addData("Action", "Lost positioning - waiting before search");
            return;
        }

        // Get current position from Aurora system
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = positionManager.getCurrentHeading();

        // Calculate distance to field center (0,0)
        double distanceToCenter = Math.sqrt(currentX * currentX + currentY * currentY);

        // Add detailed debugging for the navigation logic
        telemetry.addData("=== NAVIGATION DEBUG ===", "");
        telemetry.addData("Current Position", "(%.1f, %.1f)", currentX, currentY);
        telemetry.addData("Distance to Center", "%.1f inches", distanceToCenter);
        telemetry.addData("Movement Tolerance", "%.1f inches", MOVEMENT_TOLERANCE);
        telemetry.addData("Should Complete?", distanceToCenter < MOVEMENT_TOLERANCE ? "YES" : "NO");

        // Check if we've reached the center
        if (distanceToCenter < MOVEMENT_TOLERANCE) {
            actionManager.stopDriving();
            telemetry.addData("Action", "Reached field center!");
            telemetry.addData("COMPLETION", "Distance %.1f < tolerance %.1f", distanceToCenter, MOVEMENT_TOLERANCE);
            telemetry.update();
            sleep(3000); // Give time to see completion reason
            currentState = AutoState.COMPLETE;
            return;
        }

        // NEW: Use vision-aware movement that keeps AprilTags in view
        navigateWithVisionAwareness();
    }

    /**
     * Navigate using lateral movements to keep AprilTags in camera view
     * This approach prioritizes maintaining visual contact with tags
     */
    private void navigateWithVisionAwareness() {
        // Get current position
        double currentX = positionManager.getCurrentX();
        double currentY = positionManager.getCurrentY();
        double currentHeading = positionManager.getCurrentHeading();

        // Calculate movement vector to center
        double deltaX = 0.0 - currentX;  // Target X - Current X
        double deltaY = 0.0 - currentY;  // Target Y - Current Y
        double distanceToCenter = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Check if we still have AprilTag detections
        List<AprilTagDetection> detections = positionManager.getDirectAprilTagDetections();
        boolean hasTagsInView = !detections.isEmpty();

        telemetry.addData("=== VISION-AWARE MOVEMENT ===", "");
        telemetry.addData("Tags in View", hasTagsInView ? "YES (" + detections.size() + ")" : "NO");
        telemetry.addData("Movement Vector", "X: %.1f, Y: %.1f", deltaX, deltaY);
        telemetry.addData("Distance", "%.1f inches", distanceToCenter);

        if (!hasTagsInView) {
            // Lost visual contact - stop and search again
            actionManager.stopDriving();
            currentState = AutoState.WAIT_BEFORE_SEARCH;
            stateTimer.reset();
            telemetry.addData("Action", "Lost AprilTag sight - returning to search");
            return;
        }

        // Calculate desired movement direction (toward center)
        double angleToCenter = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Get tag bearing to understand where tag is relative to robot
        AprilTagDetection primaryTag = detections.get(0);
        double tagBearing = primaryTag.ftcPose.bearing;
        double tagRange = primaryTag.ftcPose.range;

        telemetry.addData("Primary Tag", "ID %d, Bearing %.1f°, Range %.1f\"",
                         primaryTag.id, tagBearing, tagRange);
        telemetry.addData("Angle to Center", "%.1f°", angleToCenter);

        // VISION-AWARE MOVEMENT STRATEGY:
        // 1. If tag is behind us (bearing > 90° or < -90°), use lateral movement only
        // 2. If tag is in front, we can move forward/backward carefully
        // 3. Always prioritize movements that keep tag in view

        double drivePower = 0.0;    // Forward/backward
        double strafePower = 0.0;   // Left/right (lateral)
        double turnPower = 0.0;     // Rotation

        // Slow, controlled movement speeds
        double maxMovementSpeed = 0.25;  // Reduced from fast movement
        double movementGain = 0.02;      // How aggressively to move toward center

        // Convert angle to center to robot-relative coordinates
        double robotRelativeAngle = Math.toRadians(angleToCenter - currentHeading);

        // Calculate desired movement components
        double desiredForward = Math.cos(robotRelativeAngle) * distanceToCenter * movementGain;
        double desiredLateral = Math.sin(robotRelativeAngle) * distanceToCenter * movementGain;

        // Adjust movement based on tag position to maintain visual contact
        if (Math.abs(tagBearing) > 90) {
            // Tag is behind us - use ONLY lateral movement to avoid losing sight
            drivePower = 0.0;  // No forward/backward movement
            strafePower = desiredLateral;
            telemetry.addData("Movement Mode", "LATERAL ONLY (tag behind)");
        } else if (Math.abs(tagBearing) > 45) {
            // Tag is to the side - prefer lateral movement with minimal forward
            drivePower = desiredForward * 0.3;  // Reduced forward movement
            strafePower = desiredLateral;
            telemetry.addData("Movement Mode", "LATERAL PREFERRED (tag to side)");
        } else {
            // Tag is in front - can use both movements but still prefer lateral
            drivePower = desiredForward * 0.6;  // Moderate forward movement
            strafePower = desiredLateral;
            telemetry.addData("Movement Mode", "BALANCED (tag in front)");
        }

        // Limit movement speeds
        drivePower = Math.max(-maxMovementSpeed, Math.min(maxMovementSpeed, drivePower));
        strafePower = Math.max(-maxMovementSpeed, Math.min(maxMovementSpeed, strafePower));

        // Only turn if we need to orient for a specific task (not implemented yet)
        // For now, maintain current heading to keep tag in view
        turnPower = 0.0;

        telemetry.addData("Drive Powers", "F: %.2f, S: %.2f, T: %.2f",
                         drivePower, strafePower, turnPower);

        // Execute the movement
        actionManager.mechanumDrive(drivePower, strafePower, turnPower);

        // Update positioning (includes vision corrections)
        positionManager.updatePosition();
    }

    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        telemetry.addData("=== FIXED AURORA VISION AUTO ===", "");
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Time", "%.1f seconds", autonomousTimer.seconds());

        // Always display robot position using Aurora system
        telemetry.addData("=== ROBOT POSITION ===", "");
        if (positionManager != null && positionManager.isPositionValid()) {
            double currentX = positionManager.getCurrentX();
            double currentY = positionManager.getCurrentY();
            double currentHeading = positionManager.getCurrentHeading();
            double distanceToCenter = Math.sqrt(currentX * currentX + currentY * currentY);

            telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", currentX, currentY);
            telemetry.addData("Robot Heading", "%.1f degrees", currentHeading);
            telemetry.addData("Distance to Center", "%.1f inches", distanceToCenter);
        } else {
            if (positionManager == null) {
                telemetry.addData("Robot Position", "ERROR - Position manager not initialized");
            } else {
                telemetry.addData("Robot Position", "UNKNOWN - No AprilTag detected");
                telemetry.addData("Status", "Waiting for position data...");
            }
        }

        // Show wait state information
        if (currentState == AutoState.WAIT_BEFORE_SEARCH) {
            telemetry.addData("Wait Timer", "%.1f/%.1f seconds", stateTimer.seconds(), WAIT_BEFORE_SEARCH_TIME);
        }

        // Show target information when moving
        if (currentState == AutoState.MOVE_TO_CENTER && positionManager != null && positionManager.isPositionValid()) {
            telemetry.addData("=== NAVIGATION ===", "");
            telemetry.addData("Target Position", "(0.0, 0.0)");
            telemetry.addData("Distance to Target", "%.1f inches", positionManager.getDistanceToTarget());
            telemetry.addData("Angle to Target", "%.1f degrees", positionManager.getAngleToTarget());
        }

        telemetry.update();
    }

    /**
     * Normalize angle to -180 to +180 degrees
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Cleanup and stop all systems
     */
    private void cleanup() {
        telemetry.addData("Status", "Cleaning up...");

        if (actionManager != null) {
            actionManager.stopDriving();
        }
        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }

    /**
     * Check for AprilTag detection using the EXACT same method as SimpleVisionAuto
     * This bypasses all Aurora caching and management layers
     */
    private boolean checkForDirectAprilTagDetection() {
        if (positionManager == null) return false;

        try {
            // Get detections directly - exactly like SimpleVisionAuto does
            List<AprilTagDetection> detections = positionManager.getDirectAprilTagDetections();

            // Check if we have any detections (same logic as SimpleVisionAuto)
            boolean hasDetections = !detections.isEmpty();

            telemetry.addData("Direct Detection Check", "Found %d tags", detections.size());
            telemetry.addData("Vision System Ready", positionManager.isVisionAvailable());

            // Show detected tag IDs for debugging
            if (hasDetections) {
                StringBuilder tagIds = new StringBuilder();
                for (AprilTagDetection detection : detections) {
                    if (tagIds.length() > 0) tagIds.append(", ");
                    tagIds.append(detection.id);
                }
                telemetry.addData("Detected Tag IDs", tagIds.toString());
            }

            return hasDetections;

        } catch (Exception e) {
            telemetry.addData("Detection Error", e.getMessage());
            return false;
        }
    }
}

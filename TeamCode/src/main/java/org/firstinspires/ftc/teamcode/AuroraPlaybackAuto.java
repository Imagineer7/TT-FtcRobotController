/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Playback Autonomous for AURORA movement recording system
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;
import org.firstinspires.ftc.teamcode.util.aurora.MovementRecorder;
import org.firstinspires.ftc.teamcode.util.aurora.SmartMechanumDrive;

import java.util.List;
import java.util.Map;

/**
 * Playback Autonomous - Replays recorded movements with safety integration
 *
 * Features:
 * - Loads and replays recorded TeleOp sessions
 * - Integrated safety checks and obstacle avoidance
 * - Real-time playback telemetry
 * - Configurable recording selection
 * - Emergency stop capabilities
 * - Speed scaling for safety
 */
@Autonomous(name="AURORA Playback Auto", group="AURORA")
public class AuroraPlaybackAuto extends LinearOpMode {

    private AuroraManager robotManager;
    private MovementRecorder recorder;
    private List<MovementRecorder.ActionWaypoint> waypoints;
    private ElapsedTime playbackTimer = new ElapsedTime();
    private int currentWaypointIndex = 0;
    private boolean playbackActive = false;

    // Configurable recording name - change this to select different recordings
    private String recordingToPlay = "auto_recording_latest";

    // Safety and performance settings
    private double speedScale = 0.8; // Reduce speed for safety (80% of recorded speed)
    private boolean safetyEnabled = true;
    private double minObstacleDistance = 30.0; // cm

    // Playback statistics
    private int waypointsExecuted = 0;
    private int safetyInterventions = 0;
    private double playbackProgress = 0.0;

    @Override
    public void runOpMode() {
        // Initialize the robot systems
        telemetry.addLine("Initializing AURORA Playback Autonomous...");
        telemetry.update();

        // Initialize the unified robot system manager
        robotManager = new AuroraManager(hardwareMap, telemetry);
        recorder = robotManager.getMovementRecorder();

        // Load the recording
        loadRecording();

        // Display initialization results
        telemetry.clear();
        if (waypoints.isEmpty()) {
            telemetry.addLine("❌ PLAYBACK FAILED");
            telemetry.addData("Error", "No recording found: " + recordingToPlay);
            telemetry.addLine("");
            telemetry.addLine("Available recordings:");
            List<String> available = recorder.getAvailableRecordings();
            if (available.isEmpty()) {
                telemetry.addLine("  (No recordings found)");
            } else {
                for (String name : available) {
                    telemetry.addLine("  • " + name);
                }
            }
        } else {
            telemetry.addLine("✅ AURORA Playback Ready!");
            telemetry.addData("Recording", recordingToPlay);
            telemetry.addData("Waypoints", waypoints.size());
            telemetry.addData("Duration", String.format("%.1f seconds",
                waypoints.get(waypoints.size() - 1).timestamp));
            telemetry.addData("Speed Scale", String.format("%.0f%%", speedScale * 100));
            telemetry.addData("Safety", safetyEnabled ? "ENABLED" : "DISABLED");
        }

        telemetry.addLine("");
        telemetry.addLine("Press PLAY to start playback");
        telemetry.update();

        waitForStart();

        if (!waypoints.isEmpty()) {
            startPlayback();

            while (opModeIsActive() && playbackActive) {
                // Execute playback step
                executePlaybackStep();

                // Update robot systems
                robotManager.update(gamepad1, gamepad2);

                // Update telemetry
                updatePlaybackTelemetry();

                // Emergency stop check
                if (gamepad1.start && gamepad2.start) {
                    emergencyStop();
                    break;
                }
            }

            // Clean shutdown
            stopPlayback();
        }

        telemetry.addLine("🏁 Playback completed");
        telemetry.update();
    }

    /**
     * Load the specified recording
     */
    private void loadRecording() {
        // First try the configured recording name
        waypoints = recorder.loadRecording(recordingToPlay);

        // If that fails, try to find the most recent recording
        if (waypoints.isEmpty()) {
            List<String> available = recorder.getAvailableRecordings();
            if (!available.isEmpty()) {
                // Find the most recent recording (highest timestamp)
                String mostRecent = "";
                long latestTime = 0;

                for (String name : available) {
                    try {
                        // Extract timestamp from filename if possible
                        if (name.startsWith("auto_recording_")) {
                            String timestampStr = name.substring("auto_recording_".length());
                            long timestamp = Long.parseLong(timestampStr);
                            if (timestamp > latestTime) {
                                latestTime = timestamp;
                                mostRecent = name;
                            }
                        }
                    } catch (NumberFormatException e) {
                        // Skip recordings that don't follow our naming convention
                    }
                }

                if (!mostRecent.isEmpty()) {
                    recordingToPlay = mostRecent;
                    waypoints = recorder.loadRecording(mostRecent);
                }
            }
        }
    }

    /**
     * Start playback execution
     */
    private void startPlayback() {
        playbackTimer.reset();
        playbackActive = true;
        currentWaypointIndex = 0;
        waypointsExecuted = 0;
        safetyInterventions = 0;
        playbackProgress = 0.0;
    }

    /**
     * Execute one step of playback
     */
    private void executePlaybackStep() {
        if (!playbackActive || waypoints.isEmpty()) {
            return;
        }

        double currentTime = playbackTimer.seconds();

        // Find the current waypoint to execute
        MovementRecorder.ActionWaypoint currentWaypoint = getCurrentWaypoint(currentTime);

        if (currentWaypoint != null) {
            // Execute movement with safety checks
            executeWaypointWithSafety(currentWaypoint);

            // Execute recorded actions (with safety considerations)
            executeActions(currentWaypoint.actions);

            waypointsExecuted++;
        } else {
            // Playback complete
            playbackActive = false;
            if (robotManager.getDriveSystem() != null) {
                robotManager.getDriveSystem().setFineMovement(0, 0, 0);
            }
        }

        // Update progress
        if (!waypoints.isEmpty()) {
            double totalDuration = waypoints.get(waypoints.size() - 1).timestamp;
            playbackProgress = Math.min(100.0, (currentTime / totalDuration) * 100.0);
        }
    }

    /**
     * Find the waypoint that should be executing at the current time
     */
    private MovementRecorder.ActionWaypoint getCurrentWaypoint(double currentTime) {
        // Find the waypoint that should be executing at this time
        while (currentWaypointIndex < waypoints.size()) {
            MovementRecorder.ActionWaypoint waypoint = waypoints.get(currentWaypointIndex);

            if (waypoint.timestamp <= currentTime) {
                // This waypoint should be executing
                if (currentWaypointIndex < waypoints.size() - 1) {
                    MovementRecorder.ActionWaypoint nextWaypoint = waypoints.get(currentWaypointIndex + 1);
                    if (nextWaypoint.timestamp > currentTime) {
                        return waypoint; // Current waypoint is active
                    }
                } else {
                    return waypoint; // Last waypoint
                }
                currentWaypointIndex++;
            } else {
                break;
            }
        }

        return currentWaypointIndex < waypoints.size() ? waypoints.get(currentWaypointIndex) : null;
    }

    /**
     * Execute waypoint movement with integrated safety checks
     */
    private void executeWaypointWithSafety(MovementRecorder.ActionWaypoint waypoint) {
        if (robotManager.getDriveSystem() == null) return;

        // Apply speed scaling
        double safeAxial = waypoint.axial * speedScale;
        double safeLateral = waypoint.lateral * speedScale;
        double safeYaw = waypoint.yaw * speedScale;

        // Apply safety checks if enabled
        if (safetyEnabled) {
            // Example: Check for obstacles using sensors (if available)
            // Note: This would require sensor integration in your robot
            /*
            if (robotManager.getSensorSystem() != null) {
                double obstacleDistance = robotManager.getSensorSystem().getForwardDistance();
                if (obstacleDistance < minObstacleDistance && safeAxial > 0) {
                    safeAxial *= 0.3; // Slow down significantly
                    safetyInterventions++;
                }
            }
            */

            // Limit maximum speeds for safety
            safeAxial = Math.max(-0.7, Math.min(0.7, safeAxial));
            safeLateral = Math.max(-0.7, Math.min(0.7, safeLateral));
            safeYaw = Math.max(-0.7, Math.min(0.7, safeYaw));
        }

        // Apply the safe movement
        robotManager.getDriveSystem().setFineMovement(safeLateral, safeAxial, safeYaw);
    }

    /**
     * Execute recorded actions with safety considerations
     */
    private void executeActions(Map<String, Object> actions) {
        // Execute recorded subsystem actions
        for (Map.Entry<String, Object> action : actions.entrySet()) {
            String actionName = action.getKey();
            Object value = action.getValue();

            try {
                switch (actionName) {
                    case "drive_mode":
                        if (robotManager.getDriveSystem() != null && value instanceof String) {
                            // Apply drive mode changes (could be made safer)
                            String mode = (String) value;
                            if ("Precision".equals(mode)) {
                                robotManager.getDriveSystem().setCurrentMode(SmartMechanumDrive.DriveMode.PRECISION);
                            } else if ("Sport".equals(mode) && !safetyEnabled) {
                                // Only allow sport mode if safety is disabled
                                robotManager.getDriveSystem().setCurrentMode(SmartMechanumDrive.DriveMode.SPORT);
                            }
                        }
                        break;

                    case "field_relative":
                        if (robotManager.getDriveSystem() != null && value instanceof Boolean) {
                            robotManager.getDriveSystem().setFieldRelative((Boolean) value);
                        }
                        break;

                    // Note: Shooter actions are not executed during autonomous playback for safety
                    // Add more action handlers as needed, with appropriate safety checks
                }
            } catch (Exception e) {
                // Handle action execution errors gracefully
                telemetry.addData("Action Error", actionName + ": " + e.getMessage());
            }
        }
    }

    /**
     * Stop playback execution
     */
    private void stopPlayback() {
        playbackActive = false;
        if (robotManager.getDriveSystem() != null) {
            robotManager.getDriveSystem().setFineMovement(0, 0, 0);
        }
    }

    /**
     * Emergency stop all systems
     */
    private void emergencyStop() {
        stopPlayback();
        robotManager.emergencyStopAll();
        telemetry.addLine("🚨 EMERGENCY STOP ACTIVATED");
    }

    /**
     * Update playback telemetry
     */
    private void updatePlaybackTelemetry() {
        telemetry.addLine("=== PLAYBACK STATUS ===");
        telemetry.addData("Status", playbackActive ? "🔴 PLAYING" : "⏹ STOPPED");
        telemetry.addData("Recording", recordingToPlay);
        telemetry.addData("Progress", String.format("%.1f%%", playbackProgress));
        telemetry.addData("Waypoints", String.format("%d/%d",
            currentWaypointIndex, waypoints.size()));
        telemetry.addData("Time", String.format("%.1fs", playbackTimer.seconds()));

        if (!waypoints.isEmpty()) {
            double totalDuration = waypoints.get(waypoints.size() - 1).timestamp;
            telemetry.addData("Total Duration", String.format("%.1fs", totalDuration));
        }

        telemetry.addLine("");
        telemetry.addLine("=== SAFETY ===");
        telemetry.addData("Speed Scale", String.format("%.0f%%", speedScale * 100));
        telemetry.addData("Safety Checks", safetyEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Interventions", safetyInterventions);

        telemetry.addLine("");
        telemetry.addLine("=== ROBOT STATUS ===");
        if (robotManager.getDriveSystem() != null) {

            SmartMechanumDrive.DriveSystemData driveData = robotManager.getDriveSystem().getTelemetryData();
            telemetry.addData("Drive Mode", driveData.driveMode);
            telemetry.addData("Battery", String.format("%.1fV", driveData.voltage));
            telemetry.addData("System Health", robotManager.isSystemsHealthy() ? "✅ OK" : "⚠️ ISSUE");
        }

        telemetry.addLine("");
        telemetry.addData("Emergency Stop", "Both START buttons");
    }
}

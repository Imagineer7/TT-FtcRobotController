/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Recording TeleOp for AURORA movement recording system
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AuroraManager;
import org.firstinspires.ftc.teamcode.util.MovementRecorder;
import org.firstinspires.ftc.teamcode.util.SmartTelemetryManager;
import org.firstinspires.ftc.teamcode.util.SmartMechanumDrive;

/**
 * Recording TeleOp - Specialized TeleOp for recording robot movements and actions
 *
 * Recording Controls (Letter Buttons):
 * Y: Start/Stop Recording (creates new recording or stops current)
 * X: Pause/Resume Recording (toggles pause state)
 * B: Start Over (stops current recording and starts fresh)
 * A: Quick Save (saves current recording with timestamp)
 *
 * Normal driving controls remain the same as AURORA TeleOp
 * Semi-auto actions are DISABLED during recording mode
 *
 * Recording Features:
 * - High-fidelity recording at 20Hz
 * - Automatic timestamping and naming
 * - Real-time recording statistics
 * - Safe file handling with error recovery
 * - Visual feedback for recording state
 */
@TeleOp(name="AURORA Recording TeleOp", group="AURORA")
public class AuroraRecordingTeleOp extends LinearOpMode {

    private AuroraManager robotManager;
    private SmartTelemetryManager smartTelemetry;
    private MovementRecorder recorder;
    private final ElapsedTime runtime = new ElapsedTime();

    // Recording control state tracking
    private boolean prevYButton = false;
    private boolean prevXButton = false;
    private boolean prevBButton = false;
    private boolean prevAButton = false;

    // Recording session management
    private String currentRecordingName = "";
    private boolean recordingStarted = false;
    private ElapsedTime recordingSessionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the robot systems
        telemetry.addLine("Initializing AURORA Recording TeleOp...");
        telemetry.update();

        // Initialize the unified robot system manager
        robotManager = new AuroraManager(hardwareMap, telemetry);
        robotManager.setRecordingMode(true); // Enable recording mode

        // Get recorder reference
        recorder = robotManager.getMovementRecorder();

        // Initialize the smart telemetry manager
        smartTelemetry = new SmartTelemetryManager(telemetry, robotManager);

        telemetry.clear();
        telemetry.addLine("🎬 AURORA Recording TeleOp Ready!");
        telemetry.addLine("");
        telemetry.addLine("=== RECORDING CONTROLS ===");
        telemetry.addLine("Y: Start/Stop Recording");
        telemetry.addLine("X: Pause/Resume Recording");
        telemetry.addLine("B: Start Over (New Recording)");
        telemetry.addLine("A: Quick Save Current Recording");
        telemetry.addLine("");
        telemetry.addLine("=== DRIVING CONTROLS ===");
        telemetry.addLine("Left Stick: Forward/Strafe");
        telemetry.addLine("Right Stick: Rotate");
        telemetry.addLine("D-Pad: Fine Movement (20%)");
        telemetry.addLine("Bumpers: Fine Rotation (20%)");
        telemetry.addLine("");
        telemetry.addLine("⚠️ Semi-auto actions DISABLED in recording mode");
        telemetry.addLine("");
        telemetry.addLine("Press PLAY to start!");
        telemetry.update();

        waitForStart();
        runtime.reset();
        recordingSessionTimer.reset();

        while (opModeIsActive()) {
            // Handle recording controls FIRST (before normal robot update)
            handleRecordingControls();

            // Update robot systems (with recording data collection)
            robotManager.update(gamepad1, gamepad2);

            // Update recording system if active
            if (robotManager.isRecordingMode()) {
                robotManager.updateRecording(gamepad1, gamepad2);
            }

            // Update telemetry with recording-focused display
            updateRecordingTelemetry();

            // Standard telemetry update
            smartTelemetry.update();
        }

        // Clean shutdown - stop any active recording
        if (recorder != null && recorder.isRecording()) {
            recorder.stopRecording();
            telemetry.addLine("🛑 Recording stopped and saved on OpMode end");
            telemetry.update();
        }
    }

    /**
     * Handle recording control inputs using letter buttons
     */
    private void handleRecordingControls() {
        if (recorder == null) return;

        // Y Button: Start/Stop Recording
        boolean currentYButton = gamepad1.y;
        if (currentYButton && !prevYButton) {
            if (recorder.isRecording()) {
                // Stop current recording
                boolean saved = recorder.stopRecording();
                recordingStarted = false;
                telemetry.addLine(saved ? "🛑 Recording stopped and saved!" : "❌ Failed to save recording!");
            } else {
                // Start new recording
                currentRecordingName = generateRecordingName();
                recorder.startRecording(currentRecordingName);
                recordingStarted = true;
                recordingSessionTimer.reset();
                telemetry.addLine("🎬 Recording started: " + currentRecordingName);
            }
        }
        prevYButton = currentYButton;

        // X Button: Pause/Resume Recording
        boolean currentXButton = gamepad1.x;
        if (currentXButton && !prevXButton && recorder.isRecording()) {
            recorder.togglePause();
            telemetry.addLine(recorder.isPaused() ? "⏸ Recording paused" : "▶ Recording resumed");
        }
        prevXButton = currentXButton;

        // B Button: Start Over (Stop current and start new)
        boolean currentBButton = gamepad1.b;
        if (currentBButton && !prevBButton) {
            if (recorder.isRecording()) {
                // Stop current recording without saving
                recorder.stopRecording();
            }
            // Start fresh recording
            currentRecordingName = generateRecordingName();
            recorder.startRecording(currentRecordingName);
            recordingStarted = true;
            recordingSessionTimer.reset();
            telemetry.addLine("🔄 Started new recording: " + currentRecordingName);
        }
        prevBButton = currentBButton;

        // A Button: Quick Save (save current with timestamp)
        boolean currentAButton = gamepad1.a;
        if (currentAButton && !prevAButton && recorder.isRecording()) {
            boolean saved = recorder.stopRecording();
            if (saved) {
                telemetry.addLine("💾 Quick save completed: " + currentRecordingName);
                // Immediately start a new recording
                currentRecordingName = generateRecordingName();
                recorder.startRecording(currentRecordingName);
                recordingSessionTimer.reset();
            } else {
                telemetry.addLine("❌ Quick save failed!");
            }
        }
        prevAButton = currentAButton;
    }

    /**
     * Generate a unique recording name with timestamp
     */
    private String generateRecordingName() {
        long timestamp = System.currentTimeMillis();
        return "auto_recording_" + timestamp;
    }

    /**
     * Update telemetry with recording-focused information
     */
    private void updateRecordingTelemetry() {
        if (recorder == null) return;

        MovementRecorder.RecordingTelemetryData recordingData = recorder.getTelemetryData();

        telemetry.addLine("=== RECORDING STATUS ===");

        if (recordingData.isRecording) {
            String status = recordingData.isPaused ? "⏸ PAUSED" : "🔴 RECORDING";
            telemetry.addData("Status", status);
            telemetry.addData("Name", recordingData.currentRecording);
            telemetry.addData("Duration", String.format("%.1fs", recordingData.duration));
            telemetry.addData("Waypoints", recordingData.waypointCount);
            telemetry.addData("Rate", String.format("%.0f Hz", recordingData.waypointCount / Math.max(recordingData.duration, 0.1)));
        } else {
            telemetry.addData("Status", "⏹ STOPPED");
            telemetry.addData("Available Recordings", recordingData.availableRecordings);
        }

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Y", recorder.isRecording() ? "Stop Recording" : "Start Recording");
        telemetry.addData("X", recorder.isRecording() ? (recorder.isPaused() ? "Resume" : "Pause") : "---");
        telemetry.addData("B", "Start Over");
        telemetry.addData("A", recorder.isRecording() ? "Quick Save" : "---");

        telemetry.addLine("");
        telemetry.addLine("=== SESSION INFO ===");
        telemetry.addData("Session Time", String.format("%.1fs", recordingSessionTimer.seconds()));
        telemetry.addData("Recording Mode", "ENABLED");
        telemetry.addData("Semi-Auto", "DISABLED");

        // Show drive system status
        if (robotManager.getDriveSystem() != null) {
            telemetry.addLine("");
            telemetry.addLine("=== DRIVE STATUS ===");
            SmartMechanumDrive.DriveSystemData driveData = robotManager.getDriveSystem().getTelemetryData();
            telemetry.addData("Drive Mode", driveData.driveMode);
            telemetry.addData("Field Relative", driveData.fieldRelative ? "ON" : "OFF");
            telemetry.addData("Battery", String.format("%.1fV", driveData.voltage));
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IntakePerception;

import java.util.Locale;

/**
 * IntakePerceptionTest - Test OpMode for V3 sensor fusion
 *
 * Tests the IntakePerception class with real hardware sensors.
 * Displays all derived signals and raw sensor values on telemetry.
 *
 * Sensor Layout (per intake):
 * - Confirmation sensor: goBILDA distance sensor (0-1000mm analog)
 * - Left sensor: REV Color Sensor V3 (color + proximity)
 * - Right sensor: REV Color Sensor V3 (color + proximity)
 *
 * Controls:
 * - A: Toggle between front/back intake display
 * - X: Show detailed sensor readings
 * - DPAD UP/DOWN: Recalibrate (deprecated, no-op)
 *
 * Usage:
 * 1. Run this OpMode
 * 2. Place artifacts in front/back intakes
 * 3. Observe sensor fusion results on telemetry
 * 4. Verify presence detection and color classification work correctly
 */
@TeleOp(name="V3: Intake Perception Test", group="Testing")
public class IntakePerceptionTest extends LinearOpMode {

    private AuroraHardwareConfig hardware;
    private IndexingConfig config;
    private IntakePerception frontPerception;
    private IntakePerception backPerception;

    private boolean showDetailedReadings = false;
    private boolean showingFront = true;
    private boolean lastAButton = false;
    private boolean lastXButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Check hardware initialization
        if (!hardware.isIndexingSystemInitialized()) {
            telemetry.addData("❌ ERROR", "Indexing system not initialized");
            telemetry.addData("Details", hardware.getInitializationSummary());
            telemetry.update();
            
            while (opModeIsActive() || !isStarted()) {
                sleep(100);
            }
            return;
        }

        // Create config
        config = new IndexingConfig();

        // Create IntakePerception instances for both intakes
        // New API: confirmation (goBILDA), left (REV Color V3), right (REV Color V3)
        try {
            frontPerception = new IntakePerception(
                IntakePerception.IntakeSide.FRONT,
                hardware.getFrontDistanceSensor(),           // Confirmation sensor (goBILDA)
                hardware.getFrontIntakeColorLeft(),          // Left sensor (REV Color V3)
                hardware.getFrontIntakeColorRight(),         // Right sensor (REV Color V3)
                config
            );
            telemetry.addData("✓", "Front perception initialized");
        } catch (Exception e) {
            telemetry.addData("❌", "Front perception error: " + e.getMessage());
            frontPerception = null;
        }

        try {
            backPerception = new IntakePerception(
                IntakePerception.IntakeSide.BACK,
                hardware.getBackDistanceSensor(),            // Confirmation sensor (goBILDA)
                hardware.getBackIntakeColorLeft(),           // Left sensor (REV Color V3)
                hardware.getBackIntakeColorRight(),          // Right sensor (REV Color V3)
                config
            );
            telemetry.addData("✓", "Back perception initialized");
        } catch (Exception e) {
            telemetry.addData("❌", "Back perception error: " + e.getMessage());
            backPerception = null;
        }

        telemetry.addData("Status", "Ready!");
        telemetry.addData("Controls", "A=Switch intake, X=Details, DPAD UP/DOWN=Recalibrate");
        telemetry.update();

        waitForStart();

        // Enable color sampling for continuous color detection in test mode
        if (frontPerception != null) {
            frontPerception.enableColorSampling();
        }
        if (backPerception != null) {
            backPerception.enableColorSampling();
        }

        // Main loop
        while (opModeIsActive()) {
            // Handle button inputs
            handleButtons();

            // Update perceptions
            if (frontPerception != null) {
                frontPerception.update();
            }
            if (backPerception != null) {
                backPerception.update();
            }

            // Display telemetry
            displayTelemetry();

            telemetry.update();
        }
    }

    /**
     * Handle button inputs
     */
    @SuppressWarnings("deprecation")
    private void handleButtons() {
        // Toggle intake display (A button)
        boolean currentA = gamepad1.a;
        if (currentA && !lastAButton) {
            showingFront = !showingFront;
        }
        lastAButton = currentA;

        // Toggle detailed readings (X button)
        boolean currentX = gamepad1.x;
        if (currentX && !lastXButton) {
            showDetailedReadings = !showDetailedReadings;
        }
        lastXButton = currentX;

        // Recalibrate front (DPAD UP)
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !lastDpadUp) {
            if (frontPerception != null) {
                frontPerception.calibrateRevSensorBaseline();
                telemetry.addData("✓", "Front REV sensor recalibrated");
            }
        }
        lastDpadUp = currentDpadUp;

        // Recalibrate back (DPAD DOWN)
        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !lastDpadDown) {
            if (backPerception != null) {
                backPerception.calibrateRevSensorBaseline();
                telemetry.addData("✓", "Back REV sensor recalibrated");
            }
        }
        lastDpadDown = currentDpadDown;
    }

    /**
     * Display telemetry
     */
    private void displayTelemetry() {
        telemetry.addData("═══════════════════════════════════", "");
        telemetry.addData("Mode", showDetailedReadings ? "DETAILED" : "SUMMARY");
        telemetry.addData("Viewing", showingFront ? "FRONT INTAKE" : "BACK INTAKE");
        telemetry.addData("═══════════════════════════════════", "");

        // Display selected intake
        IntakePerception perception = showingFront ? frontPerception : backPerception;
        if (perception != null) {
            displayPerception(perception);
        } else {
            telemetry.addData("ERROR", "Perception not initialized");
        }

        // Display controls
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  A", "Switch intake (currently: " + (showingFront ? "FRONT" : "BACK") + ")");
        telemetry.addData("  X", "Toggle details (currently: " + (showDetailedReadings ? "ON" : "OFF") + ")");
        telemetry.addData("  DPAD UP", "Recalibrate front REV sensor");
        telemetry.addData("  DPAD DOWN", "Recalibrate back REV sensor");
    }

    /**
     * Display perception telemetry
     */
    private void displayPerception(IntakePerception perception) {
        if (showDetailedReadings) {
            // Detailed mode: show full snapshot
            String snapshot = perception.getTelemetrySnapshot();
            String[] lines = snapshot.split("\n");
            for (String line : lines) {
                telemetry.addLine(line);
            }
        } else {
            // Summary mode: show key signals only
            telemetry.addData("Artifact Hint", perception.getArtifactHint() ? "✓ YES" : "✗ NO");
            telemetry.addData("Presence Confidence", perception.getPresenceConfidence());
            telemetry.addData("", "");
            
            telemetry.addData("Sensor Signals", "");
            telemetry.addData("  Confirmation (goBILDA)", perception.isConfirmationDetected() ? "✓" : "✗");
            telemetry.addData("  Left Proximity (REV)", perception.isLeftProximityDetected() ? "✓" : "✗");
            telemetry.addData("  Right Proximity (REV)", perception.isRightProximityDetected() ? "✓" : "✗");
            telemetry.addData("  Color (Left)", perception.colorSeesArtifact_Left() ? "✓" : "✗");
            telemetry.addData("  Color (Right)", perception.colorSeesArtifact_Right() ? "✓" : "✗");
            telemetry.addData("", "");
            
            telemetry.addData("Best Color", perception.getBestColorClass());
            telemetry.addData("Color Confidence", String.format(Locale.US, "%.2f", perception.getBestColorConfidence()));

            // Add raw color values in summary mode
            telemetry.addData("", "");
            telemetry.addData("Raw Color Values", "");
            double[] leftRaw = perception.getLeftColorRaw();
            if (leftRaw != null) {
                telemetry.addData("  Left RGB", String.format(Locale.US, "R:%.3f G:%.3f B:%.3f",
                    leftRaw[0], leftRaw[1], leftRaw[2]));
            }
            double[] rightRaw = perception.getRightColorRaw();
            if (rightRaw != null) {
                telemetry.addData("  Right RGB", String.format(Locale.US, "R:%.3f G:%.3f B:%.3f",
                    rightRaw[0], rightRaw[1], rightRaw[2]));
            }

            // Add color scores
            double[] leftScores = perception.getLeftColorScores();
            if (leftScores != null) {
                telemetry.addData("  Left Scores", String.format(Locale.US, "Purple:%.2f Green:%.2f",
                    leftScores[0], leftScores[1]));
            }
            double[] rightScores = perception.getRightColorScores();
            if (rightScores != null) {
                telemetry.addData("  Right Scores", String.format(Locale.US, "Purple:%.2f Green:%.2f",
                    rightScores[0], rightScores[1]));
            }

            // Add multiple artifact detection
            telemetry.addData("", "");
            telemetry.addData("Multiple Artifact Detection", "");
            if (perception.hasMultipleDifferentColors()) {
                telemetry.addData("  Status", "✓ DIFFERENT COLORS (reliable)");
                telemetry.addData("  Pattern", "Left and right sensors see opposite colors");
            } else if (perception.hasMultipleSameColor()) {
                telemetry.addData("  Status", "⚠ SAME COLOR (less reliable)");
                telemetry.addData("  Pattern", "Both proximity sensors < 5cm");
            } else {
                telemetry.addData("  Status", "✗ SINGLE OR NONE");
            }
        }
    }
}

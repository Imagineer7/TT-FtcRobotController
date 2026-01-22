package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IntakePerception;

/**
 * IntakePerceptionTest - Test OpMode for V3 sensor fusion
 *
 * Tests the IntakePerception class with real hardware sensors.
 * Displays all derived signals and raw sensor values on telemetry.
 *
 * Controls:
 * - DPAD UP/DOWN: Recalibrate REV sensor baseline
 * - A: Toggle between front/back intake display
 * - X: Show detailed sensor readings
 *
 * Usage:
 * 1. Run this OpMode
 * 2. Place artifacts in front/back intakes
 * 3. Observe sensor fusion results on telemetry
 * 4. Verify hysteresis, debounce, and color detection work correctly
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
        try {
            frontPerception = new IntakePerception(
                IntakePerception.IntakeSide.FRONT,
                hardware.getFrontDistanceSensor(),
                hardware.getFrontLeftDistanceSensor(),
                hardware.getFrontCenterColorSensor(),  // SWAPPED: mouth sensor is outward
                hardware.getFrontRightColorSensor(),   // SWAPPED: right sensor is at mouth
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
                hardware.getBackDistanceSensor(),
                hardware.getBackRightDistanceSensor(),
                hardware.getBackCenterColorSensor(),
                hardware.getBackRightColorSensor(),
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
            telemetry.addData("  Front Blocked", perception.isFrontBlocked() ? "✓" : "✗");
            telemetry.addData("  Mouth Occupied", perception.isMouthOccupied() ? "✓" : "✗");
            telemetry.addData("  Color (Outward)", perception.colorSeesArtifact_Outward() ? "✓" : "✗");
            telemetry.addData("  Color (Mouth)", perception.colorSeesArtifact_Mouth() ? "✓" : "✗");
            telemetry.addData("", "");
            
            telemetry.addData("Best Color", perception.getBestColorClass());
            telemetry.addData("Color Confidence", String.format("%.2f", perception.getBestColorConfidence()));

            // Add raw color values in summary mode
            telemetry.addData("", "");
            telemetry.addData("Raw Color Values", "");
            double[] outwardRaw = perception.getOutwardColorRaw();
            if (outwardRaw != null) {
                telemetry.addData("  Outward RGB", String.format("%.3f, %.3f, %.3f",
                    outwardRaw[0], outwardRaw[1], outwardRaw[2]));
            }
            double[] mouthRaw = perception.getMouthColorRaw();
            if (mouthRaw != null) {
                telemetry.addData("  Mouth RGB", String.format("%.3f, %.3f, %.3f",
                    mouthRaw[0], mouthRaw[1], mouthRaw[2]));
            }

            // Add color scores
            double[] outwardScores = perception.getOutwardColorScores();
            if (outwardScores != null) {
                telemetry.addData("  Outward Scores", String.format("P:%.2f G:%.2f",
                    outwardScores[0], outwardScores[1]));
            }
            double[] mouthScores = perception.getMouthColorScores();
            if (mouthScores != null) {
                telemetry.addData("  Mouth Scores", String.format("P:%.2f G:%.2f",
                    mouthScores[0], mouthScores[1]));
            }
        }
    }
}

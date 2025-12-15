package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingSystem;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Artifact;

/**
 * IndexingSystemExample - Example OpMode demonstrating the push-based indexing system
 *
 * This OpMode shows how to:
 * - Initialize the indexing system
 * - Detect and collect artifacts
 * - Fire artifacts
 * - Monitor system status
 *
 * CONTROLS:
 * Gamepad1:
 *   A - Simulate front intake artifact detection (PURPLE)
 *   B - Simulate back intake artifact detection (GREEN)
 *   X - Fire signal
 *   Y - Reset indexing system
 *   
 *   Right Bumper - Enable shooter
 *   Left Bumper - Disable shooter
 *   
 *   D-Pad Up - Change to LONG_RANGE preset
 *   D-Pad Down - Change to SHORT_RANGE preset
 *   D-Pad Left/Right - Change to MID_RANGE preset
 */
@TeleOp(name="Indexing System Example", group="Examples")
public class IndexingSystemExample extends LinearOpMode {

    // System components
    private AuroraHardwareConfig hardware;
    private ShooterConfig shooterConfig;
    private Shooter shooter;
    private IndexingConfig indexingConfig;
    private IndexingSystem indexingSystem;

    // Button state tracking (for edge detection)
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastXButton = false;
    private boolean lastYButton = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;

    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry.setAutoClear(false);
        telemetry.addLine("🤖 Initializing Indexing System Example...");
        telemetry.update();

        try {
            // Initialize hardware
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initializeWithOdometry();

            // Initialize shooter configuration and shooter
            shooterConfig = new ShooterConfig();
            shooterConfig.setPreset(ShooterConfig.ShooterPreset.LONG_RANGE);
            shooter = new Shooter(hardware, shooterConfig, telemetry);

            // Initialize indexing configuration
            indexingConfig = new IndexingConfig();
            indexingConfig.setDebugTelemetry(true);

            // Initialize indexing system
            indexingSystem = new IndexingSystem(hardware, indexingConfig, shooter, telemetry);

            telemetry.clear();
            telemetry.addLine("✅ Indexing System Ready!");
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("  A - Front intake (PURPLE)");
            telemetry.addLine("  B - Back intake (GREEN)");
            telemetry.addLine("  X - Fire");
            telemetry.addLine("  Y - Reset");
            telemetry.addLine("  R Bumper - Enable shooter");
            telemetry.addLine("  L Bumper - Disable shooter");
            telemetry.addLine("  D-Pad - Change presets");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Initialization Error: " + e.getMessage());
            telemetry.update();
            return;
        }

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            try {
                // Update systems
                shooter.update();
                indexingSystem.update();

                // Handle button inputs
                handleControls();

                // Display status
                displayTelemetry();

            } catch (Exception e) {
                telemetry.addLine("❌ Loop Error: " + e.getMessage());
            }

            telemetry.update();
            sleep(20); // Small delay for telemetry update
        }

        // Cleanup
        if (shooter != null) {
            shooter.stop();
        }
        if (hardware != null) {
            hardware.stopAllMotors();
        }
    }

    /**
     * Handle gamepad controls
     */
    private void handleControls() {
        // A Button - Simulate front intake detection (PURPLE artifact)
        if (gamepad1.a && !lastAButton) {
            Artifact artifact = new Artifact(
                Artifact.Color.PURPLE,
                Artifact.Location.UNKNOWN,
                0  // Will be set by indexing system
            );
            boolean success = indexingSystem.onArtifactDetected(
                artifact, 
                IndexingSystem.IntakeSource.FRONT
            );
            if (!success) {
                telemetry.addLine("⚠️ Failed to start collection from front intake");
            }
        }
        lastAButton = gamepad1.a;

        // B Button - Simulate back intake detection (GREEN artifact)
        if (gamepad1.b && !lastBButton) {
            Artifact artifact = new Artifact(
                Artifact.Color.GREEN,
                Artifact.Location.UNKNOWN,
                0  // Will be set by indexing system
            );
            boolean success = indexingSystem.onArtifactDetected(
                artifact, 
                IndexingSystem.IntakeSource.BACK
            );
            if (!success) {
                telemetry.addLine("⚠️ Failed to start collection from back intake");
            }
        }
        lastBButton = gamepad1.b;

        // X Button - Fire signal
        if (gamepad1.x && !lastXButton) {
            boolean success = indexingSystem.onFireSignal();
            if (!success) {
                telemetry.addLine("⚠️ Failed to fire");
            }
        }
        lastXButton = gamepad1.x;

        // Y Button - Reset system
        if (gamepad1.y && !lastYButton) {
            indexingSystem.reset();
            telemetry.addLine("🔄 System Reset");
        }
        lastYButton = gamepad1.y;

        // Right Bumper - Enable shooter
        if (gamepad1.right_bumper && !lastRightBumper) {
            shooter.enable();
            shooter.spinUp();
            telemetry.addLine("🚀 Shooter Enabled");
        }
        lastRightBumper = gamepad1.right_bumper;

        // Left Bumper - Disable shooter
        if (gamepad1.left_bumper && !lastLeftBumper) {
            shooter.disable();
            telemetry.addLine("🛑 Shooter Disabled");
        }
        lastLeftBumper = gamepad1.left_bumper;

        // D-Pad Up - Long range preset
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooter.setPreset(ShooterConfig.ShooterPreset.LONG_RANGE);
            telemetry.addLine("📏 Long Range Preset");
        }
        lastDpadUp = gamepad1.dpad_up;

        // D-Pad Down - Short range preset
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooter.setPreset(ShooterConfig.ShooterPreset.SHORT_RANGE);
            telemetry.addLine("📏 Short Range Preset");
        }
        lastDpadDown = gamepad1.dpad_down;

        // D-Pad Left/Right - Mid range preset
        if ((gamepad1.dpad_left || gamepad1.dpad_right) && !lastDpadLeft) {
            shooter.setPreset(ShooterConfig.ShooterPreset.MID_RANGE);
            telemetry.addLine("📏 Mid Range Preset");
        }
        lastDpadLeft = gamepad1.dpad_left || gamepad1.dpad_right;
    }

    /**
     * Display system status on telemetry
     */
    private void displayTelemetry() {
        telemetry.clear();
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("🤖 INDEXING SYSTEM EXAMPLE");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("");

        // Indexing system status
        telemetry.addLine("📦 INDEXING STATUS");
        telemetry.addData("State", indexingSystem.getCurrentState());
        telemetry.addData("Artifacts", String.format("%d/%d", 
            indexingSystem.getArtifactCount(), 
            IndexingConfig.MAX_ARTIFACTS));
        telemetry.addData("Ready to Fire", indexingSystem.isReadyToFire() ? "✅ YES" : "❌ NO");
        telemetry.addLine("");

        // Artifact locations
        telemetry.addLine("📍 ARTIFACT LOCATIONS");
        telemetry.addData("Center", formatArtifact(indexingSystem.getArtifactInCenter()));
        telemetry.addData("Front Intake", formatArtifact(indexingSystem.getArtifactInFrontIntake()));
        telemetry.addData("Back Intake", formatArtifact(indexingSystem.getArtifactInBackIntake()));
        telemetry.addLine("");

        // Shot planning
        if (indexingSystem.getPlannedSecondShot() != null || 
            indexingSystem.getPlannedThirdShot() != null) {
            telemetry.addLine("🎯 SHOT PLAN");
            if (indexingSystem.getPlannedSecondShot() != null) {
                telemetry.addData("2nd Shot", formatArtifact(indexingSystem.getPlannedSecondShot()));
            }
            if (indexingSystem.getPlannedThirdShot() != null) {
                telemetry.addData("3rd Shot", formatArtifact(indexingSystem.getPlannedThirdShot()));
            }
            telemetry.addLine("");
        }

        // Shooter status
        telemetry.addLine("🚀 SHOOTER STATUS");
        telemetry.addData("Enabled", shooter.isEnabled() ? "✅ YES" : "❌ NO");
        telemetry.addData("State", shooter.getState());
        telemetry.addData("RPM", String.format("%.0f / %.0f (%.0f%%)", 
            shooter.getCurrentRPM(),
            shooter.getTargetRPM(),
            shooter.getRPMPercentage() * 100));
        telemetry.addData("Ready", shooter.isReadyToFire() ? "✅ YES" : "❌ NO");
        telemetry.addLine("");

        // Errors
        if (indexingSystem.getErrorCount() > 0) {
            telemetry.addLine("⚠️ ERRORS");
            telemetry.addData("Last Error", indexingSystem.getLastError());
            telemetry.addData("Error Count", indexingSystem.getErrorCount());
            telemetry.addLine("");
        }

        // Controls reminder
        telemetry.addLine("🎮 CONTROLS");
        telemetry.addLine("A=Front(PURPLE) | B=Back(GREEN) | X=Fire | Y=Reset");
        telemetry.addLine("RB=Enable Shooter | LB=Disable | DPad=Presets");
    }

    /**
     * Format an artifact for display
     */
    private String formatArtifact(Artifact artifact) {
        if (artifact == null) {
            return "Empty";
        }
        return String.format("%s #%d", artifact.getColor(), artifact.getCollectionOrder());
    }
}

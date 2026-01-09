package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingSystem;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Artifact;

/**
 * IndexingSystemTest - Testing OpMode for Indexing and Intake Systems
 *
 * This OpMode provides comprehensive testing capabilities for the artifact indexing
 * and intake systems. It includes manual control, automated testing sequences, and
 * real-time telemetry for debugging.
 *
 * INTAKE SYSTEM BEHAVIOR:
 * ────────────────────────────────────────────────────────────────────────────
 * - Both intake rollers run CONTINUOUSLY at full power (from config)
 * - When artifact detected, collection starts automatically
 * - During collection, intake continues at full power
 * - When intake stores an artifact, power reduces to 50% to retain it
 * - System automatically manages intake speeds - no manual control needed
 *
 * CONTROLS:
 * ────────────────────────────────────────────────────────────────────────────
 * Gamepad 1:
 *   [A] - Simulate artifact detection at FRONT intake
 *   [B] - Simulate artifact detection at BACK intake
 *   [X] - Fire artifact (if ready)
 *   [Y] - Emergency stop all motors/servos
 *
 *   [LEFT_BUMPER]  - Toggle manual servo mode (for troubleshooting)
 *   [RIGHT_BUMPER] - Run injector servos (manual mode only)
 *   [LEFT_TRIGGER] - Run uptake servos (manual mode only)
 *   [RIGHT_TRIGGER] - Run transfer servos (manual mode only)
 *
 *   [START] - Reset indexing system
 *   [BACK]  - Toggle debug telemetry
 *
 * Gamepad 2:
 *   [A] - Spin up shooter
 *   [B] - Stop shooter
 *
 * TESTING MODES:
 * ────────────────────────────────────────────────────────────────────────────
 * 1. Manual Mode (Default)
 *    - Full manual control of all systems
 *    - Real-time telemetry display
 *    - Intakes run automatically based on system state
 *
 * 2. Manual Servo Mode (LEFT_BUMPER)
 *    - For troubleshooting individual servos only
 *    - Does NOT affect intake roller operation
 */
@TeleOp(name = "🔧 Indexing System Test", group = "Testing")
public class IndexingSystemTest extends LinearOpMode {

    // Hardware and Systems
    private AuroraHardwareConfig hardware;
    private IndexingSystem indexingSystem;
    private IndexingConfig indexingConfig;
    private Shooter shooter;
    private ShooterConfig shooterConfig;

    // Test Mode State
    private enum TestMode {
        MANUAL,
        AUTO_SINGLE_ARTIFACT,
        AUTO_THREE_ARTIFACTS,
        RUNNING_TEST
    }
    private TestMode currentMode = TestMode.MANUAL;
    private long testStartTime = 0;

    // Manual Control State
    private boolean manualMode = false;
    private boolean debugTelemetry = true;

    // Button state tracking (for edge detection)
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastStart = false;
    private boolean lastBack = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // Gamepad 2 state
    private boolean lastG2A = false;
    private boolean lastG2B = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🔧 INDEXING SYSTEM TEST");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        // Initialize hardware
        try {
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initialize();

            if (!hardware.isIndexingSystemInitialized()) {
                telemetry.addLine("⚠️ WARNING: Indexing system not fully initialized");
                telemetry.addLine(hardware.getIndexingInitError());
            }

            if (!hardware.isShooterSystemInitialized()) {
                telemetry.addLine("⚠️ WARNING: Shooter system not fully initialized");
                telemetry.addLine(hardware.getShooterInitError());
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Hardware initialization failed:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            throw e;
        }

        // Initialize configurations
        indexingConfig = new IndexingConfig();
        indexingConfig.setDebugTelemetry(true);

        shooterConfig = new ShooterConfig();

        // Initialize shooter first (needed by indexing system)
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        shooter.enable();

        // Initialize indexing system (requires shooter)
        indexingSystem = new IndexingSystem(hardware, indexingConfig, shooter, telemetry);

        telemetry.addLine("✅ Initialization complete!");
        telemetry.addLine("");
        telemetry.addLine("Press [START] to begin");
        telemetry.addLine("See OpMode comments for controls");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🤖 INDEXING SYSTEM TEST - ACTIVE");
        telemetry.addLine("════════════════════════════════════");
        telemetry.update();

        // Main control loop
        while (opModeIsActive()) {
            // Update all systems
            indexingSystem.update();
            shooter.update();

            // Handle button inputs
            handleGamepadInputs();

            // Update telemetry
            updateTelemetry();

            // Small delay to prevent loop overrun
            sleep(20);
        }

        // Cleanup
        hardware.stopAllMotors();
    }

    /**
     * Handle all gamepad inputs
     */
    private void handleGamepadInputs() {
        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 1 - Indexing System Controls
        // ═══════════════════════════════════════════════════════════════

        // [A] - Collect from FRONT intake
        if (gamepad1.a && !lastA) {
            if (currentMode == TestMode.MANUAL) {
                Artifact artifact = new Artifact(
                    Artifact.Color.UNKNOWN,
                    Artifact.Location.UNKNOWN,
                    0  // Collection order will be set by system
                );
                indexingSystem.onArtifactDetected(artifact, IndexingSystem.IntakeSource.FRONT);
                telemetry.addLine("▶ Collecting from FRONT intake");
            }
        }
        lastA = gamepad1.a;

        // [B] - Collect from BACK intake
        if (gamepad1.b && !lastB) {
            if (currentMode == TestMode.MANUAL) {
                Artifact artifact = new Artifact(
                    Artifact.Color.UNKNOWN,
                    Artifact.Location.UNKNOWN,
                    0  // Collection order will be set by system
                );
                indexingSystem.onArtifactDetected(artifact, IndexingSystem.IntakeSource.BACK);
                telemetry.addLine("▶ Collecting from BACK intake");
            }
        }
        lastB = gamepad1.b;

        // [X] - Fire artifact
        if (gamepad1.x && !lastX) {
            boolean fired = indexingSystem.onFireSignal();
            if (fired) {
                telemetry.addLine("🔥 Firing artifact!");
            } else {
                telemetry.addLine("⚠️ Cannot fire - no artifact ready");
            }
        }
        lastX = gamepad1.x;

        // [Y] - Emergency stop
        if (gamepad1.y && !lastY) {
            hardware.stopAllMotors();
            indexingSystem.reset();
            telemetry.addLine("🛑 EMERGENCY STOP");
        }
        lastY = gamepad1.y;

        // [START] - Reset system
        if (gamepad1.start && !lastStart) {
            indexingSystem.reset();
            telemetry.addLine("🔄 System reset");
        }
        lastStart = gamepad1.start;

        // [BACK] - Toggle debug telemetry
        if (gamepad1.back && !lastBack) {
            debugTelemetry = !debugTelemetry;
            indexingConfig.setDebugTelemetry(debugTelemetry);
            telemetry.addLine(debugTelemetry ? "📊 Debug ON" : "📊 Debug OFF");
        }
        lastBack = gamepad1.back;

        // Bumpers - Manual servo control
        if (gamepad1.left_bumper && !lastLeftBumper) {
            manualMode = !manualMode;
            telemetry.addLine(manualMode ? "🔧 Manual Servo Mode ON" : "🔧 Manual Servo Mode OFF");
        }
        lastLeftBumper = gamepad1.left_bumper;

        // Manual servo control when in manual mode
        if (manualMode) {
            // Right bumper - Injector servos
            if (gamepad1.right_bumper) {
                if (hardware.getInjectorServoLeft() != null) {
                    hardware.getInjectorServoLeft().setPower(1.0);
                }
                if (hardware.getInjectorServoRight() != null) {
                    hardware.getInjectorServoRight().setPower(1.0);
                }
            } else {
                if (hardware.getInjectorServoLeft() != null) {
                    hardware.getInjectorServoLeft().setPower(0.0);
                }
                if (hardware.getInjectorServoRight() != null) {
                    hardware.getInjectorServoRight().setPower(0.0);
                }
            }

            // Left trigger - Uptake servos
            if (gamepad1.left_trigger > 0.1) {
                if (hardware.getUptakeServoL() != null) {
                    hardware.getUptakeServoL().setPower(1.0);
                }
                if (hardware.getUptakeServoR() != null) {
                    hardware.getUptakeServoR().setPower(1.0);
                }
            } else {
                if (hardware.getUptakeServoL() != null) {
                    hardware.getUptakeServoL().setPower(0.0);
                }
                if (hardware.getUptakeServoR() != null) {
                    hardware.getUptakeServoR().setPower(0.0);
                }
            }

            // Right trigger - Transfer servos
            if (gamepad1.right_trigger > 0.1) {
                if (hardware.getFrontTransferServo() != null) {
                    hardware.getFrontTransferServo().setPower(gamepad1.right_trigger);
                }
                if (hardware.getBackTransferServo() != null) {
                    hardware.getBackTransferServo().setPower(gamepad1.right_trigger);
                }
            } else {
                if (hardware.getFrontTransferServo() != null) {
                    hardware.getFrontTransferServo().setPower(0.0);
                }
                if (hardware.getBackTransferServo() != null) {
                    hardware.getBackTransferServo().setPower(0.0);
                }
            }
        }

        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 2 - Shooter Controls
        // ═══════════════════════════════════════════════════════════════

        // [A] - Spin up shooter
        if (gamepad2.a && !lastG2A) {
            shooter.spinUp();
            telemetry.addLine("🔄 Shooter spinning up");
        }
        lastG2A = gamepad2.a;

        // [B] - Stop shooter
        if (gamepad2.b && !lastG2B) {
            shooter.stop();
            telemetry.addLine("⏹️ Shooter stopped");
        }
        lastG2B = gamepad2.b;
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.clear();

        // Header
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🔧 INDEXING SYSTEM TEST");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("");

        // System Status
        telemetry.addLine("📊 SYSTEM STATUS:");
        telemetry.addData("  Mode", currentMode);
        telemetry.addData("  Manual Servo Control", manualMode ? "ON" : "OFF");
        telemetry.addData("  Debug Telemetry", debugTelemetry ? "ON" : "OFF");
        telemetry.addLine("");

        // Indexing System Status
        telemetry.addLine("🤖 INDEXING SYSTEM:");
        telemetry.addData("  State", indexingSystem.getCurrentState());
        telemetry.addData("  Artifacts", indexingSystem.getArtifactCount() + "/3");
        telemetry.addData("  Ready to Fire", indexingSystem.isReadyToFire() ? "YES ✓" : "NO");

        // Intake Status
        telemetry.addLine("");
        telemetry.addLine("🔄 INTAKE STATUS:");
        telemetry.addLine("  Front: " + getIntakeStatus(indexingSystem.getArtifactInFrontIntake()));
        telemetry.addLine("  Back: " + getIntakeStatus(indexingSystem.getArtifactInBackIntake()));
        telemetry.addLine("  (Intakes run continuously - full power or 50% if storing)");

        // Artifact Locations
        telemetry.addLine("");
        telemetry.addLine("📍 ARTIFACT LOCATIONS:");
        Artifact centerArtifact = indexingSystem.getArtifactInCenter();
        Artifact frontArtifact = indexingSystem.getArtifactInFrontIntake();
        Artifact backArtifact = indexingSystem.getArtifactInBackIntake();

        telemetry.addData("  Center", centerArtifact != null ?
            centerArtifact.getColor() + " #" + centerArtifact.getCollectionOrder() : "Empty");
        telemetry.addData("  Front", frontArtifact != null ?
            frontArtifact.getColor() + " #" + frontArtifact.getCollectionOrder() : "Empty");
        telemetry.addData("  Back", backArtifact != null ?
            backArtifact.getColor() + " #" + backArtifact.getCollectionOrder() : "Empty");

        // Shooter Status
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOOTER:");
        telemetry.addData("  Status", shooter.isEnabled() ? "Enabled" : "Disabled");
        telemetry.addData("  Ready", shooter.isReadyToFire() ? "YES ✓" : "NO");
        telemetry.addData("  At Speed", shooter.isAtTargetRPM() ? "YES ✓" : "NO");

        // Configuration Info
        if (debugTelemetry) {
            telemetry.addLine("");
            telemetry.addLine("⚙️ TIMING CONFIG:");
            telemetry.addData("  Intake Time", String.format("%.1fs", indexingConfig.getIntakeRollerTime()));
            telemetry.addData("  Transfer Time", String.format("%.1fs", indexingConfig.getTransferServoTime()));
            telemetry.addData("  Fire Time", String.format("%.1fs", indexingConfig.getFireFeedTime()));
        }

        // Controls
        telemetry.addLine("");
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addLine("  [A] Detect Front  [B] Detect Back  [X] Fire");
        telemetry.addLine("  [Y] Emergency Stop  [START] Reset");
        if (manualMode) {
            telemetry.addLine("  SERVO MANUAL MODE:");
            telemetry.addLine("  [RB] Injector [LT] Uptake [RT] Transfer");
        }

        // Errors
        String lastError = indexingSystem.getLastError();
        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine("⚠️ ERROR:");
            telemetry.addLine("  " + lastError);
        }

        telemetry.update();
    }

    /**
     * Get human-readable intake status
     */
    private String getIntakeStatus(Artifact artifact) {
        if (artifact != null) {
            return "STORING " + artifact.getColor() + " #" + artifact.getCollectionOrder() + " (50% power)";
        } else {
            return "Empty (100% power - ready to collect)";
        }
    }
}


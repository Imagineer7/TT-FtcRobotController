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
 * FullSystemTest - Comprehensive test OpMode for complete indexing and shooting system
 *
 * This OpMode provides integrated testing of both the indexing system and shooter system
 * with comprehensive telemetry for shot planning, firing sequences, and system coordination.
 *
 * FEATURES:
 * ────────────────────────────────────────────────────────────────────────────
 * - Complete indexing system with manual push mode
 * - Full shooter system with RPM control and warm-up
 * - Integrated shot planning and firing sequences
 * - Comprehensive telemetry pages for debugging
 * - Real-time system status monitoring
 * - Shot sequence visualization and planning
 *
 * SHOT PLANNING RULES:
 * ────────────────────────────────────────────────────────────────────────────
 * - 3 Artifacts: Center fires first (no planning), 2nd/3rd planned by motif
 * - 2 Artifacts (auto mode): First shot can be planned using empty intake
 * - 2 Artifacts (manual mode): Center fires first, second planned by motif
 * - Motif pattern determines optimal shot order (PPG, PGP, GPP)
 *
 * FIRING SEQUENCE:
 * ────────────────────────────────────────────────────────────────────────────
 * 1. Shooter spins up to target RPM and stabilizes
 * 2. First artifact fired from center, marked as FIRED
 * 3. Second planned artifact transferred to center
 * 4. Second artifact fired immediately (if shooter ready)
 * 5. Third planned artifact transferred and fired
 * 6. System returns to IDLE when complete
 *
 * CONTROLS:
 * ────────────────────────────────────────────────────────────────────────────
 * Gamepad 1 - Indexing Controls:
 *   [A] - Simulate front intake artifact
 *   [B] - Simulate back intake artifact
 *   [X] - Start firing sequence
 *   [Y] - Emergency stop
 *   [START] - Reset systems
 *   [BACK] - Toggle debug telemetry
 *   [L-STICK] - Toggle manual push mode
 *   [R-STICK] - Execute manual push
 *   [DPAD] - Navigate telemetry pages
 *
 * Gamepad 2 - Shooter & Advanced:
 *   [A] - Short Range Shooting (2000 RPM) + AUTO-FIRE
 *   [B] - Mid-Range Shooting (2300 RPM) + AUTO-FIRE
 *   [Y] - Long Range Shooting (2800 RPM) + AUTO-FIRE
 *   [X] - Cycle motif pattern (PPG/PGP/GPP)
 *   [L-TRIGGER] - Warmup mode (65% of target RPM, no auto-fire)
 *   [R-BUMPER] - Spin up to current target (manual)
 *   [R-TRIGGER] - Manual fire shot
 *   [START] - Stop shooter and cancel firing
 *   [BACK] - Clear shooter error
 *
 *   AUTO-FIRE: Hold A/B/Y to spin up and automatically fire all artifacts
 *   AUTO-CANCEL: Release A/B/Y to cancel firing and stop shooter
 *   MANUAL MODE: R-Bumper or RPM adjustment keeps shooter running
 *
 *   DPAD Controls:
 *   [DPAD-UP] - Run uptake servos up (feed into shooter)
 *   [DPAD-DOWN] - Run uptake servos down (retract from shooter)
 *   [L-BUMPER + DPAD] - RPM adjustment mode:
 *     LB+UP/DOWN: ±100 RPM, LB+LEFT/RIGHT: ±500 RPM
 */
@TeleOp(name = "🚀 Full System Test", group = "Testing")
public class FullSystemTest extends LinearOpMode {

    // Hardware and Systems
    private AuroraHardwareConfig hardware;
    private IndexingSystem indexingSystem;
    private IndexingConfig indexingConfig;
    private Shooter shooter;
    private ShooterConfig shooterConfig;

    // System State Tracking
    private enum SystemMode {
        IDLE,                   // Both systems idle, ready for commands
        COLLECTING,             // Indexing system collecting artifacts
        READY_TO_SHOOT,         // Has artifacts, shooter can be spun up
        FIRING_SEQUENCE,        // Active firing sequence in progress
        SHOOTER_SPINUP,         // Shooter spinning up for firing
        ERROR                   // System error state
    }
    private SystemMode currentMode = SystemMode.IDLE;

    // Telemetry Page System
    private enum TelemetryPage {
        OVERVIEW,               // Main system status and shot planning
        INDEXING,               // Detailed indexing system status
        SHOOTER,                // Detailed shooter system status
        SHOT_PLANNING,          // Shot planning analysis and motif
        FIRING_SEQUENCE,        // Active firing sequence status
        DIAGNOSTICS,            // System health and performance
        DEBUG,                  // Debug messages for uptake servo tracking
        CONTROLS,               // Control help and mappings
        FIRING_TREE,            // Hierarchical firing conditions tree
        FIRING_ACTIVATION       // Why firingSequenceActive never becomes true
    }
    private TelemetryPage currentPage = TelemetryPage.OVERVIEW;

    // Shot Planning and Firing State
    private boolean firingSequenceActive = false;
    private boolean showTelemetryPages = true;
    private int currentShotNumber = 1;
    private long firingSequenceStartTime = 0;

    // Motif Pattern Management
    private String[] motifPatterns = {"PPG", "PGP", "GPP"};
    private int currentMotifIndex = 0;

    // Shooter Control Management
    private enum ControlMode {
        PRESET,  // Using preset buttons
        MANUAL   // Using manual RPM adjustment
    }
    private ControlMode controlMode = ControlMode.PRESET;
    private double manualTargetRPM = 2300; // Start at mid-range

    // Auto-cancel state tracking
    private boolean presetButtonsWereActive = false;

    // Button State Tracking
    private boolean lastA1 = false, lastB1 = false, lastX1 = false, lastY1 = false;
    private boolean lastStart1 = false, lastBack1 = false;
    private boolean lastLeftStick1 = false, lastRightStick1 = false;
    private boolean lastDpadUp1 = false, lastDpadDown1 = false;
    private boolean lastDpadLeft1 = false, lastDpadRight1 = false;

    private boolean lastA2 = false, lastB2 = false, lastX2 = false, lastY2 = false;
    private boolean lastStart2 = false, lastBack2 = false;
    private boolean lastRightBumper2 = false, lastRightTrigger2 = false;
    private boolean lastDpadUp2 = false, lastDpadDown2 = false;
    private boolean lastDpadLeft2 = false, lastDpadRight2 = false;
    private float lastLeftTrigger2 = 0.0f;

    // Performance Tracking
    private long loopStartTime = 0;
    private long maxLoopTime = 0;
    private long totalLoops = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("🚀 FULL SYSTEM TEST");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("Initializing systems...");
        telemetry.update();

        // Initialize hardware
        try {
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initialize();
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

        // Initialize systems
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        indexingSystem = new IndexingSystem(hardware, indexingConfig, shooter, telemetry);

        telemetry.addLine("✅ Systems initialized!");
        telemetry.addLine("");
        telemetry.addLine("🤖 Manual Push Mode: " + (indexingConfig.isManualPushMode() ? "ENABLED" : "DISABLED"));
        telemetry.addLine("🎯 Shooter System: Ready");
        telemetry.addLine("📦 Indexing System: Ready");
        telemetry.addLine("");
        telemetry.addLine("Press [START] to begin testing");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Enable systems
        shooter.enable();
        indexingSystem.enable();

        telemetry.clear();
        telemetry.addLine("🚀 FULL SYSTEM TEST - ACTIVE");
        telemetry.update();

        // Main control loop
        while (opModeIsActive()) {
            loopStartTime = System.currentTimeMillis();

            // Basic button detection debug (always shows on OVERVIEW)
            if (gamepad2.a || gamepad2.b || gamepad2.y || gamepad1.x) {
                String buttons = "";
                if (gamepad2.a) buttons += "GP2.A ";
                if (gamepad2.b) buttons += "GP2.B ";
                if (gamepad2.y) buttons += "GP2.Y ";
                if (gamepad1.x) buttons += "GP1.X ";
                telemetry.addLine("🎮 BASIC BUTTON DETECTION: " + buttons.trim());
            }

            // Handle gamepad inputs
            handleGamepadInputs();

            // Update all systems
            indexingSystem.update();
            shooter.update();

            // Update system mode based on current state
            updateSystemMode();

            // Handle active firing sequence
            if (firingSequenceActive) {
                handleFiringSequence();
            }

            // Update telemetry
            updateTelemetryDisplay();

            // Performance tracking
            updatePerformanceMetrics();

            sleep(20); // 50Hz update rate
        }

        // Cleanup
        firingSequenceActive = false;
        shooter.disable();
        indexingSystem.disable();
        hardware.stopAllMotors();
    }

    /**
     * Handle all gamepad inputs
     */
    private void handleGamepadInputs() {
        // Debug: Show this method is being called
        if (gamepad2.a || gamepad2.b || gamepad2.y || gamepad1.x) {
            telemetry.addLine("🎮 handleGamepadInputs() CALLED with buttons pressed");
        }
        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 1 - Indexing System Controls
        // ═══════════════════════════════════════════════════════════════

        // [A] - Simulate front intake artifact
        if (gamepad1.a && !lastA1) {
            Artifact artifact = new Artifact(
                Math.random() < 0.5 ? Artifact.Color.PURPLE : Artifact.Color.GREEN,
                Artifact.Location.UNKNOWN,
                0
            );
            indexingSystem.onArtifactDetected(artifact, IndexingSystem.IntakeSource.FRONT);
            telemetry.addLine("🔄 Simulated " + artifact.getColor() + " artifact at FRONT");
        }
        lastA1 = gamepad1.a;

        // [B] - Simulate back intake artifact
        if (gamepad1.b && !lastB1) {
            Artifact artifact = new Artifact(
                Math.random() < 0.5 ? Artifact.Color.PURPLE : Artifact.Color.GREEN,
                Artifact.Location.UNKNOWN,
                0
            );
            indexingSystem.onArtifactDetected(artifact, IndexingSystem.IntakeSource.BACK);
            telemetry.addLine("🔄 Simulated " + artifact.getColor() + " artifact at BACK");
        }
        lastB1 = gamepad1.b;

        // [X] - Start firing sequence (hold to spin up and fire, release to cancel)
        if (gamepad1.x) {
            if (indexingSystem.getArtifactCount() > 0) {
                // Set default preset if none is set
                if (shooter.getTargetRPM() <= 0) {
                    shooter.setPreset(ShooterConfig.ShooterPreset.MID_RANGE);  // Default to mid-range
                    if (!lastX1) {
                        telemetry.addLine("🎯 Using default MID_RANGE preset for firing");
                    }
                }

                // Spin up shooter
                shooter.spinUp();

                // Auto-start firing sequence if shooter ready and artifacts available
                if (!firingSequenceActive &&
                    isShooterReadyForAutoFiring() &&
                    indexingSystem.isReadyToFire() &&
                    !indexingSystem.isOperationInProgress()) {
                    startFiringSequence();
                }
            } else {
                // No artifacts available
            }
        } else if (lastX1) {
            // X button released after being held - cancel firing and stop shooter
            if (firingSequenceActive) {
                completeFiringSequence();
            }

            // Stop shooter
            shooter.stopMotors();
        }
        lastX1 = gamepad1.x;

        // [Y] - Emergency stop
        if (gamepad1.y && !lastY1) {
            emergencyStop();
        }
        lastY1 = gamepad1.y;

        // [START] - Reset systems
        if (gamepad1.start && !lastStart1) {
            resetSystems();
        }
        lastStart1 = gamepad1.start;

        // [BACK] - Toggle debug telemetry
        if (gamepad1.back && !lastBack1) {
            boolean debug = !indexingConfig.isDebugTelemetry();
            indexingConfig.setDebugTelemetry(debug);
            telemetry.addLine("🔧 Debug telemetry " + (debug ? "ENABLED" : "DISABLED"));
        }
        lastBack1 = gamepad1.back;

        // [L-STICK] - Toggle manual push mode
        if (gamepad1.left_stick_button && !lastLeftStick1) {
            boolean manual = !indexingSystem.isManualPushMode();
            indexingSystem.setManualPushMode(manual);
            telemetry.addLine("🔄 Manual Push Mode " + (manual ? "ENABLED" : "DISABLED"));
        }
        lastLeftStick1 = gamepad1.left_stick_button;

        // [R-STICK] - Execute manual push
        if (gamepad1.right_stick_button && !lastRightStick1) {
            boolean pushed = indexingSystem.onManualPush();
            telemetry.addLine(pushed ? "🔄 Manual push executed" : "⚠️ Cannot manual push");
        }
        lastRightStick1 = gamepad1.right_stick_button;

        // DPAD - Navigate telemetry pages
        if (gamepad1.dpad_up && !lastDpadUp1) {
            currentPage = getPreviousPage(currentPage);
        }
        lastDpadUp1 = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown1) {
            currentPage = getNextPage(currentPage);
        }
        lastDpadDown1 = gamepad1.dpad_down;

        if (gamepad1.dpad_left && !lastDpadLeft1) {
            currentPage = TelemetryPage.OVERVIEW;
        }
        lastDpadLeft1 = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight1) {
            currentPage = TelemetryPage.SHOT_PLANNING;
        }
        lastDpadRight1 = gamepad1.dpad_right;

        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 2 - Shooter & Advanced Controls
        // ═══════════════════════════════════════════════════════════════

        if (gamepad2.a || gamepad2.b || gamepad2.y) {
            telemetry.addLine("🎮 About to call handleShooterPresetControls()");
        }
        handleShooterPresetControls();
        if (gamepad2.a || gamepad2.b || gamepad2.y) {
            telemetry.addLine("🎮 Returned from handleShooterPresetControls()");
        }
        handleShooterDpadControls();
    }

    /**
     * Update system mode based on current state of both systems
     */
    private void updateSystemMode() {
        if (firingSequenceActive) {
            currentMode = SystemMode.FIRING_SEQUENCE;
        } else if (indexingSystem.getCurrentState() == IndexingSystem.SystemState.ERROR ||
                   shooter.isError()) {
            currentMode = SystemMode.ERROR;
        } else if (indexingSystem.isOperationInProgress()) {
            currentMode = SystemMode.COLLECTING;
        } else if (shooter.isRunning() && !shooter.isAtTargetRPM()) {
            currentMode = SystemMode.SHOOTER_SPINUP;
        } else if (indexingSystem.getArtifactCount() > 0) {
            currentMode = SystemMode.READY_TO_SHOOT;
        } else {
            currentMode = SystemMode.IDLE;
        }
    }

    /**
     * Start the automated firing sequence
     */
    private void startFiringSequence() {
        telemetry.addLine("🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀");
        telemetry.addLine("🚀🚀🚀 startFiringSequence() CALLED! 🚀🚀🚀");
        telemetry.addLine("🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀");
        telemetry.addData("  Artifact Count", indexingSystem.getArtifactCount());

        if (indexingSystem.getArtifactCount() == 0) {
            telemetry.addLine("⚠️ No artifacts to fire");
            return;
        }

        telemetry.addData("  Setting firingSequenceActive", "TRUE");
        firingSequenceActive = true;
        firingSequenceStartTime = System.currentTimeMillis();
        currentShotNumber = 1;

        // Ensure shooter is spinning up
        boolean shooterAtTarget = shooter.isAtTargetRPM();
        telemetry.addData("  Shooter at target", shooterAtTarget);
        if (!shooterAtTarget) {
            telemetry.addLine("  Calling shooter.spinUp()");
            shooter.spinUp();
        }

        telemetry.addLine("🚀 FIRING SEQUENCE STARTED");
        telemetry.addLine(String.format("   Artifacts: %d", indexingSystem.getArtifactCount()));
        telemetry.addLine(String.format("   Motif: %s", indexingSystem.getMotifPattern()));
        telemetry.addData("  firingSequenceActive", firingSequenceActive);
        telemetry.addData("  currentShotNumber", currentShotNumber);
        telemetry.addData("  firingSequenceStartTime", firingSequenceStartTime);
        telemetry.addLine("🚀🚀🚀 END OF startFiringSequence() 🚀🚀🚀");
    }

    /**
     * Handle the active firing sequence
     */
    private void handleFiringSequence() {
        telemetry.addLine("🔍 handleFiringSequence() ENTRY:");
        telemetry.addData("  firingSequenceActive", firingSequenceActive);

        if (!firingSequenceActive) {
            telemetry.addData("  RESULT", "Exiting - not active");
            return;
        }

        // Check if we still have artifacts to fire
        telemetry.addData("  Artifact Count", indexingSystem.getArtifactCount());
        if (indexingSystem.getArtifactCount() == 0) {
            telemetry.addLine("  RESULT: No artifacts - completing sequence");
            completeFiringSequence();
            return;
        }

        // Wait for shooter to be ready
        boolean shooterAtTarget = shooter.isAtTargetRPM();
        telemetry.addData("  Shooter At Target", shooterAtTarget);
        if (!shooterAtTarget) {
            telemetry.addData("  RESULT", "Waiting for shooter to spin up");
            return; // Wait for shooter to spin up
        }

        // Check if indexing system is ready to fire
        boolean indexingReady = indexingSystem.isReadyToFire();
        boolean noOperation = !indexingSystem.isOperationInProgress();
        telemetry.addData("  Indexing Ready", indexingReady);
        telemetry.addData("  No Operation", noOperation);

        if (indexingReady && noOperation) {
            telemetry.addLine("🚀 CALLING indexingSystem.onFireSignal()!");
            // Fire the current shot
            boolean fired = indexingSystem.onFireSignal();
            telemetry.addData("  onFireSignal() returned", fired);
            if (fired) {
                telemetry.addLine(String.format("🔥 Shot %d fired!", currentShotNumber));
                currentShotNumber++;
            } else {
                telemetry.addLine("⚠️ onFireSignal() returned false!");
            }
        } else {
            telemetry.addData("  WAITING FOR", !indexingReady ? "Indexing Ready" : "Operation to finish");
        }

        // Safety timeout (30 seconds)
        long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
        telemetry.addData("  Elapsed Time", elapsed + "ms");
        if (elapsed > 30000) {
            telemetry.addLine("⚠️ Firing sequence timeout");
            completeFiringSequence();
        }
    }

    /**
     * Complete the firing sequence
     */
    private void completeFiringSequence() {
        firingSequenceActive = false;
        currentShotNumber = 1;

        telemetry.addLine("✅ FIRING SEQUENCE COMPLETE");
        telemetry.addLine("   All artifacts fired successfully");
    }

    /**
     * Emergency stop all systems
     */
    private void emergencyStop() {
        firingSequenceActive = false;
        shooter.stop();
        indexingSystem.reset();
        hardware.stopAllMotors();

        telemetry.addLine("🛑 EMERGENCY STOP");
        telemetry.addLine("   All systems stopped");
    }

    /**
     * Reset both systems to initial state
     */
    private void resetSystems() {
        firingSequenceActive = false;
        currentShotNumber = 1;
        indexingSystem.reset();
        shooter.stop();

        telemetry.addLine("🔄 SYSTEMS RESET");
        telemetry.addLine("   Ready for new operation");
    }

    /**
     * Handle preset shooting controls (A, B, Y buttons and left trigger)
     */
    private void handleShooterPresetControls() {
        if (gamepad2.a || gamepad2.b || gamepad2.y) {
            telemetry.addLine("🎮 INSIDE handleShooterPresetControls() with buttons pressed");
        }
        boolean anyPresetActive = false;

        // A Button - Short Range
        if (gamepad2.a) {
            showTelemetryPages = false; // Ensure telemetry pages are hidden to see this message
            shooter.setPreset(ShooterConfig.ShooterPreset.SHORT_RANGE);
            shooter.spinUp();
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
            if (!lastA2) {
                telemetry.addLine("🎯 Short Range (2000 RPM) - Auto-firing when ready");
            }

            // Auto-start firing sequence if shooter ready and artifacts available
            telemetry.addLine("🔴 GP2.A PRESSED - About to call handleAutoFiring()");
            handleAutoFiring();
            telemetry.addLine("🔴 GP2.A PRESSED - Returned from handleAutoFiring()");
        }
        lastA2 = gamepad2.a;

        // B Button - Mid Range
        if (gamepad2.b) {
            showTelemetryPages = false; // Ensure telemetry pages are hidden to see this message
            shooter.setPreset(ShooterConfig.ShooterPreset.MID_RANGE);
            shooter.spinUp();
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
            if (!lastB2) {
                telemetry.addLine("🎯 Mid Range (2300 RPM) - Auto-firing when ready");
            }

            // Auto-start firing sequence if shooter ready and artifacts available
            telemetry.addLine("🔵 GP2.B PRESSED - About to call handleAutoFiring()");
            handleAutoFiring();
            telemetry.addLine("🔵 GP2.B PRESSED - Returned from handleAutoFiring()");
        }
        lastB2 = gamepad2.b;

        // Y Button - Long Range
        if (gamepad2.y) {
            showTelemetryPages = false; // Ensure telemetry pages are hidden to see this message
            shooter.setPreset(ShooterConfig.ShooterPreset.LONG_RANGE);
            shooter.spinUp();
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
            if (!lastY2) {
                telemetry.addLine("🎯 Long Range (2800 RPM) - Auto-firing when ready");
            }

            // Auto-start firing sequence if shooter ready and artifacts available
            telemetry.addLine("🟡 GP2.Y PRESSED - About to call handleAutoFiring()");
            handleAutoFiring();
            telemetry.addLine("🟡 GP2.Y PRESSED - Returned from handleAutoFiring()");
        }
        lastY2 = gamepad2.y;

        // Check if preset buttons were released - cancel auto-firing and stop shooter
        if (!anyPresetActive && controlMode == ControlMode.PRESET && presetButtonsWereActive) {
            // Preset button released after being held - cancel auto-firing and stop shooter
            if (firingSequenceActive) {
                completeFiringSequence();
                telemetry.addLine("🛑 AUTO-FIRE CANCELLED: Preset button released");
            }

            // Stop shooter unless it was activated by manual controls
            if (!wasActivatedManually()) {
                shooter.stopMotors();
                telemetry.addLine("⏹️ Shooter motors stopped: Preset button released");
            }
        }

        // Update preset button state tracking
        presetButtonsWereActive = anyPresetActive;

        // X Button - Cycle motif pattern
        if (gamepad2.x && !lastX2) {
            currentMotifIndex = (currentMotifIndex + 1) % motifPatterns.length;
            String newPattern = motifPatterns[currentMotifIndex];
            indexingSystem.setMotifPattern(newPattern);
            telemetry.addLine("🎯 Motif pattern: " + newPattern);
        }
        lastX2 = gamepad2.x;

        // Left Trigger - Warmup Mode (no auto-firing in warmup)
        boolean warmupActive = gamepad2.left_trigger > 0.3;
        boolean lastWarmupActive = lastLeftTrigger2 > 0.3;

        if (warmupActive) {
            // Enable warmup at 65% of current target RPM
            if (!shooter.isRunning() || !shooter.isAtTargetRPM()) {
                double targetRPM = (controlMode == ControlMode.MANUAL) ? manualTargetRPM : shooter.getTargetRPM();
                shooter.setTargetRPM(targetRPM * 0.65);
                shooter.spinUp();
                telemetry.addLine(String.format("🔥 Warmup Mode (%.0f RPM)", targetRPM * 0.65));
            }
        } else if (lastWarmupActive && !warmupActive) {
            // Warmup trigger released - stop shooter unless other controls are active
            if (!wasActivatedManually() && !anyPresetActive) {
                shooter.stopMotors();
                telemetry.addLine("⏹️ Warmup ended: Shooter motors stopped");
            }
        }

        // Store last trigger state for release detection
        lastLeftTrigger2 = gamepad2.left_trigger;
    }

    /**
     * Check if shooter was activated by manual controls (not preset buttons)
     * Returns true if shooter should keep running even when preset buttons are released
     */
    private boolean wasActivatedManually() {
        // Check if R-Bumper is currently held (manual spin-up)
        boolean manualSpinUp = gamepad2.right_bumper;

        // Check if we're in manual RPM adjustment mode (indicates manual control)
        boolean inManualMode = controlMode == ControlMode.MANUAL;

        // Don't include warmup here since warmup has its own release detection
        return manualSpinUp || inManualMode;
    }

    /**
     * Handle automatic firing when holding preset buttons
     * Only fires if shooter is ready and not already in a firing sequence
     */
    private void handleAutoFiring() {
        // CRITICAL DEBUG: Always show this method is being called but won't show actually because only telemetry pages show it
        telemetry.addLine("🟢 handleAutoFiring() METHOD ENTRY - ALWAYS SHOWS"); //Will never show because the pages override it

        // Debug: Show that this method is being called
        boolean hasArtifacts = indexingSystem.getArtifactCount() > 0;
        boolean notFiring = !firingSequenceActive;
        boolean shooterReady = isShooterReadyForAutoFiring();
        boolean indexingReady = indexingSystem.isReadyToFire();
        boolean noOperation = !indexingSystem.isOperationInProgress();

        telemetry.addLine("🔍 handleAutoFiring() BOOLEAN EVALUATION:");
        telemetry.addData("  1. hasArtifacts", hasArtifacts + " (" + indexingSystem.getArtifactCount() + " artifacts)");
        telemetry.addData("  2. notFiring", notFiring + " (firingSequenceActive=" + firingSequenceActive + ")");
        telemetry.addData("  3. shooterReady", shooterReady);
        telemetry.addData("  4. indexingReady", indexingReady);
        telemetry.addData("  5. noOperation", noOperation);

        // CRITICAL: Show each part of the compound condition
        boolean part1 = hasArtifacts && notFiring;
        boolean part2 = part1 && shooterReady;
        boolean part3 = part2 && indexingReady;
        boolean finalCondition = part3 && noOperation;

        telemetry.addLine("🔍 COMPOUND BOOLEAN BREAKDOWN:");
        telemetry.addData("  (hasArtifacts && notFiring)", part1);
        telemetry.addData("  + shooterReady", part2);
        telemetry.addData("  + indexingReady", part3);
        telemetry.addData("  + noOperation = FINAL", finalCondition);

        // Show the exact if-condition evaluation
        telemetry.addLine("🚨 CRITICAL IF-CONDITION CHECK:");
        telemetry.addData("  Will enter if-block?", finalCondition ? "YES - SHOULD FIRE" : "NO - BLOCKED");

        // Complete firing conditions including IndexingSystem
        if (hasArtifacts && notFiring && shooterReady && indexingReady && noOperation) {
            telemetry.addLine("🚀 INSIDE IF-BLOCK - CALLING startFiringSequence()!");
            telemetry.addLine("🚀🚀🚀 THIS SHOULD SHOW IF CONDITIONS ARE MET 🚀🚀🚀");
            // Start the automated firing sequence
            startFiringSequence();
            telemetry.addLine("🚀 AFTER startFiringSequence() CALL - SHOULD SEE BOTH MESSAGES");
        } else {
            telemetry.addLine("❌ NOT ENTERING IF-BLOCK - CONDITIONS FAILED");
            telemetry.addData("  Reason", "One or more conditions is false");

            // Show which specific condition failed
            if (!hasArtifacts) telemetry.addData("  Failed", "hasArtifacts = false");
            if (!notFiring) telemetry.addData("  Failed", "notFiring = false (already firing)");
            if (!shooterReady) telemetry.addData("  Failed", "shooterReady = false");
            if (!indexingReady) telemetry.addData("  Failed", "indexingReady = false");
            if (!noOperation) telemetry.addData("  Failed", "noOperation = false (operation in progress)");
        }

        telemetry.addLine("🟢 handleAutoFiring() METHOD EXIT - ALWAYS SHOWS");
    }

     /**
      * Check if shooter is ready for auto-firing
      * Now uses the corrected shooter.isReadyToFire() method with fixed tolerances
      */
    private boolean isShooterReadyForAutoFiring() {
        // Use the shooter's built-in readiness check with corrected tolerances
        return shooter.isReadyToFire();
    }

    /**
     * Handle DPAD controls for uptake servos and RPM adjustment
     */
    private void handleShooterDpadControls() {
        boolean rpmAdjustMode = gamepad2.left_bumper;

        if (rpmAdjustMode) {
            // RPM ADJUSTMENT MODE (Left Bumper + DPAD)

            // DPAD UP - Increase by 100
            if (gamepad2.dpad_up && !lastDpadUp2) {
                manualTargetRPM += 100;
                manualTargetRPM = Math.min(manualTargetRPM, 6000);
                controlMode = ControlMode.MANUAL;
                telemetry.addLine(String.format("🎯 Manual RPM: %.0f (+100)", manualTargetRPM));
            }

            // DPAD DOWN - Decrease by 100
            if (gamepad2.dpad_down && !lastDpadDown2) {
                manualTargetRPM -= 100;
                manualTargetRPM = Math.max(manualTargetRPM, 0);
                controlMode = ControlMode.MANUAL;
                telemetry.addLine(String.format("🎯 Manual RPM: %.0f (-100)", manualTargetRPM));
            }

            // DPAD RIGHT - Increase by 500
            if (gamepad2.dpad_right && !lastDpadRight2) {
                manualTargetRPM += 500;
                manualTargetRPM = Math.min(manualTargetRPM, 6000);
                controlMode = ControlMode.MANUAL;
                telemetry.addLine(String.format("🎯 Manual RPM: %.0f (+500)", manualTargetRPM));
            }

            // DPAD LEFT - Decrease by 500
            if (gamepad2.dpad_left && !lastDpadLeft2) {
                manualTargetRPM -= 500;
                manualTargetRPM = Math.max(manualTargetRPM, 0);
                controlMode = ControlMode.MANUAL;
                telemetry.addLine(String.format("🎯 Manual RPM: %.0f (-500)", manualTargetRPM));
            }

        } else {
            // DEFAULT MODE - Uptake Servo Control
            // IMPORTANT: Only allow manual control when IndexingSystem is not managing servos

            boolean indexingSystemControlling = indexingSystem.isUptakeServoPrePositioned() ||
                                              indexingSystem.getCurrentState() == IndexingSystem.SystemState.FIRING;

            if (indexingSystemControlling) {
                // IndexingSystem is controlling servos - do not interfere
                // This prevents manual control from overriding pre-positioning or firing
            } else {
                // Safe to allow manual control - IndexingSystem is not using servos

                // DPAD UP - Run uptake servos forward (feed into shooter)
                if (gamepad2.dpad_up) {
                    // Direct hardware access for uptake servos during manual control
                    if (hardware.getUptakeServoL() != null) hardware.getUptakeServoL().setPower(1.0);
                    if (hardware.getUptakeServoR() != null) hardware.getUptakeServoR().setPower(1.0);
                }
                // DPAD DOWN - Run uptake servos reverse (retract from shooter)
                else if (gamepad2.dpad_down) {
                    if (hardware.getUptakeServoL() != null) hardware.getUptakeServoL().setPower(-1.0);
                    if (hardware.getUptakeServoR() != null) hardware.getUptakeServoR().setPower(-1.0);
                }
                // No DPAD UP/DOWN - Stop uptake servos (only when safe to do so)
                else {
                    if (hardware.getUptakeServoL() != null) hardware.getUptakeServoL().setPower(0);
                    if (hardware.getUptakeServoR() != null) hardware.getUptakeServoR().setPower(0);
                }
            }
        }

        // Update button state tracking
        lastDpadUp2 = gamepad2.dpad_up;
        lastDpadDown2 = gamepad2.dpad_down;
        lastDpadLeft2 = gamepad2.dpad_left;
        lastDpadRight2 = gamepad2.dpad_right;
    }

    /**
     * Update telemetry display based on current page
     */
    private void updateTelemetryDisplay() {
        telemetry.clear();

        // Common header
        telemetry.addLine("🚀 FULL SYSTEM TEST");
        telemetry.addData("Page", currentPage.toString() + " (" + (currentPage.ordinal() + 1) + "/10)");
        telemetry.addData("Mode", currentMode.toString());
        telemetry.addLine("════════════════════════════════════");

        switch (currentPage) {
            case OVERVIEW:
                displayOverviewPage();
                break;
            case INDEXING:
                displayIndexingPage();
                break;
            case SHOOTER:
                displayShooterPage();
                break;
            case SHOT_PLANNING:
                displayShotPlanningPage();
                break;
            case FIRING_SEQUENCE:
                displayFiringSequencePage();
                break;
            case DIAGNOSTICS:
                displayDiagnosticsPage();
                break;
            case DEBUG:
                displayDebugPage();
                break;
            case CONTROLS:
                displayControlsPage();
                break;
            case FIRING_TREE:
                displayFiringTreePage();
                break;
            case FIRING_ACTIVATION:
                displayFiringActivationPage();
                break;
        }

        telemetry.addLine("────────────────────────────────────");
        telemetry.addLine("DPAD ↑↓ Navigate | ← Overview | → Planning");
        telemetry.update();
    }

    /**
     * Display main overview page
     */
    private void displayOverviewPage() {
        telemetry.addLine("");

        // System Status
        telemetry.addLine("🤖 SYSTEM STATUS:");
        telemetry.addData("  Overall Mode", currentMode);
        telemetry.addData("  Firing Sequence", firingSequenceActive ? "ACTIVE" : "IDLE");

        // Auto-firing status indicator
        boolean autoFireActive = (gamepad2.a || gamepad2.b || gamepad2.y) &&
                                indexingSystem.getArtifactCount() > 0 &&
                                shooter.isRunning();
        if (autoFireActive) {
            String activePreset = gamepad2.a ? "SHORT" : (gamepad2.b ? "MID" : "LONG");
            telemetry.addData("  Auto-Fire Mode", activePreset + " RANGE ACTIVE");
        }

        if (firingSequenceActive) {
            telemetry.addData("  Current Shot", currentShotNumber);
        }

        // Indexing System
        telemetry.addLine("");
        telemetry.addLine("📦 INDEXING SYSTEM:");
        telemetry.addData("  State", indexingSystem.getCurrentState());
        telemetry.addData("  Artifacts", indexingSystem.getArtifactCount() + "/3");
        telemetry.addData("  Manual Push", indexingSystem.isManualPushMode() ? "ON" : "OFF");
        telemetry.addData("  Ready to Fire", indexingSystem.isReadyToFire() ? "YES" : "NO");

        // Shooter System
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOOTER SYSTEM:");
        telemetry.addData("  State", shooter.getState().toString());
        telemetry.addData("  RPM", String.format("%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM()));

        // Show tolerance-based readiness instead of exact match
        double currentRPM = shooter.getCurrentRPM();
        double targetRPM = shooter.getTargetRPM();
        double tolerance = shooterConfig.getRpmTolerance();
        boolean inTolerance = Math.abs(currentRPM - targetRPM) <= tolerance;

        telemetry.addData("  In Tolerance", inTolerance ?
            String.format("YES (±%.0f RPM)", tolerance) :
            String.format("NO (±%.0f RPM)", tolerance));
        telemetry.addData("  Ready to Fire", shooter.isReadyToFire() ? "YES" : "NO");

        // Auto-Fire Debug Information - show when any auto-fire button is held
        boolean showAutoFireDebug = gamepad2.a || gamepad2.b || gamepad2.y || gamepad1.x;
        if (showAutoFireDebug) {
            telemetry.addLine("");
            telemetry.addLine("🔍 AUTO-FIRE DEBUG:");
            telemetry.addData("  Has Artifacts", indexingSystem.getArtifactCount() > 0 ?
                String.format("YES (%d)", indexingSystem.getArtifactCount()) : "NO");
            telemetry.addData("  Not Firing", !firingSequenceActive ? "YES" : "NO");
            telemetry.addData("  Shooter Ready", shooter.isReadyToFire() ? "YES" : "NO");
            telemetry.addData("  Indexing Ready", indexingSystem.isReadyToFire() ? "YES" : "NO");
            telemetry.addData("  No Operation", !indexingSystem.isOperationInProgress() ? "YES" : "NO");

            boolean allFireConditionsMet = indexingSystem.getArtifactCount() > 0 && !firingSequenceActive &&
                shooter.isReadyToFire() && indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress();
            telemetry.addData("  🚀 WILL FIRE", allFireConditionsMet ? "YES" : "NO");

            // Show firing sequence status
            telemetry.addLine("");
            telemetry.addLine("🎯 FIRING SEQUENCE STATUS:");
            telemetry.addData("  Active", firingSequenceActive ? "YES" : "NO");
            if (firingSequenceActive) {
                telemetry.addData("  Current Shot #", currentShotNumber);
                long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
                telemetry.addData("  Elapsed", elapsed + "ms");
                telemetry.addData("  Timeout in", (30000 - elapsed) + "ms");
            }

            // Detailed shooter readiness breakdown
            telemetry.addLine("");
            telemetry.addLine("🎯 SHOOTER READINESS BREAKDOWN:");
            telemetry.addData("  Enabled", shooter.isEnabled() ? "YES" : "NO");
            telemetry.addData("  At Target RPM", shooter.isAtTargetRPM() ? "YES" : "NO");
            telemetry.addData("  Running", shooter.isRunning() ? "YES" : "NO");
            telemetry.addData("  Error State", shooter.isError() ? "YES" : "NO");
            telemetry.addData("  Current State", shooter.getState().toString());

            // Detailed breakdown of isReadyToFire() conditions
            telemetry.addLine("");
            telemetry.addLine("🔍 isReadyToFire() CONDITIONS:");
            telemetry.addData("  1. Enabled", shooter.isEnabled() ? "PASS" : "FAIL");
            telemetry.addData("  2. At Target RPM", shooter.isAtTargetRPM() ? "PASS" : "FAIL");

            // Check spinup time manually
            long currentTime = System.currentTimeMillis();
            // We can't access lastSpinupTime directly, so estimate based on state
            String spinupStatus = "UNKNOWN";
            if (shooter.getState().toString().equals("READY")) {
                spinupStatus = "PASS (in READY state)";
            } else if (shooter.getState().toString().equals("SPINNING_UP")) {
                spinupStatus = "WAITING (still spinning up)";
            } else {
                spinupStatus = "FAIL (not spinning)";
            }
            telemetry.addData("  3. Spinup Time", spinupStatus);
            telemetry.addData("  4. State = READY", shooter.getState().toString().equals("READY") ? "PASS" : "FAIL");

            // Final result - this will trigger the internal debugging
            boolean finalResult = shooter.isReadyToFire(); // This call will show debug info
            telemetry.addData("  FINAL RESULT", finalResult ? "READY TO FIRE" : "NOT READY");

            // Show RPM details with corrected tolerances
            double shooterCurrentRPM = shooter.getCurrentRPM();
            double shooterTargetRPM = shooter.getTargetRPM();
            double rpmDifference = Math.abs(shooterCurrentRPM - shooterTargetRPM);

            // Show corrected tolerance levels
            boolean withinRPMTolerance = rpmDifference <= 75.0;  // Corrected DecodeHelper tolerance
            double syncError = shooter.getSyncError();
            boolean withinSyncTolerance = syncError <= 150.0;    // Corrected sync tolerance

            telemetry.addData("  RPM Details", String.format("%.0f/%.0f", shooterCurrentRPM, shooterTargetRPM));
            telemetry.addData("  RPM Difference", String.format("%.0f RPM", rpmDifference));
            telemetry.addData("  Within 75 RPM", withinRPMTolerance ? "YES" : "NO");
            telemetry.addData("  Sync Error", String.format("%.0f/150 RPM %s", syncError, withinSyncTolerance ? "OK" : "POOR"));

            // Show what's preventing READY state
            if (shooter.getState().toString().equals("SPINNING_UP")) {
                telemetry.addData("  Issue", "Still spinning up to target");
            } else if (!shooter.isAtTargetRPM()) {
                telemetry.addData("  Issue", "RPM or sync not meeting requirements");

                // Show available diagnostic info with corrected tolerances
                double rpmPercentage = shooter.getRPMPercentage() * 100;

                telemetry.addData("    RPM Percentage", String.format("%.1f%% of target", rpmPercentage));

                if (syncError > 150.0) {
                    telemetry.addData("    Problem", "Motor sync too poor (>150 RPM)");
                } else if (!withinRPMTolerance) {
                    telemetry.addData("    Problem", "RPM not within 75 RPM tolerance");
                } else {
                    telemetry.addData("    Problem", "Stabilization time not reached");
                }

            } else if (!shooter.getState().toString().equals("READY")) {
                telemetry.addData("  Issue", "State is " + shooter.getState() + " not READY");
            } else {
                telemetry.addData("  Status", "All conditions met - should be firing");
            }
            boolean allConditionsMet = indexingSystem.getArtifactCount() > 0 && !firingSequenceActive &&
                shooter.isReadyToFire(); // Temporarily simplified conditions
            telemetry.addData("  🚀 WILL FIRE", allConditionsMet ? "YES" : "NO");

            // Show what was bypassed for debugging
            telemetry.addData("  Note", "IndexingSystem conditions bypassed for testing");

            String activeButtons = "";
            if (gamepad2.a) activeButtons += "GP2.A ";
            if (gamepad2.b) activeButtons += "GP2.B ";
            if (gamepad2.y) activeButtons += "GP2.Y ";
            if (gamepad1.x) activeButtons += "GP1.X ";
            telemetry.addData("  Active Buttons", activeButtons.trim());
        }

        // Artifact Positions
        telemetry.addLine("");
        telemetry.addLine("📍 ARTIFACT POSITIONS:");
        Artifact center = indexingSystem.getArtifactInCenter();
        Artifact front = indexingSystem.getArtifactInFrontIntake();
        Artifact back = indexingSystem.getArtifactInBackIntake();

        telemetry.addData("  Center", center != null ?
            String.format("%s #%d", center.getColor(), center.getCollectionOrder()) : "Empty");
        telemetry.addData("  Front", front != null ?
            String.format("%s #%d", front.getColor(), front.getCollectionOrder()) : "Empty");
        telemetry.addData("  Back", back != null ?
            String.format("%s #%d", back.getColor(), back.getCollectionOrder()) : "Empty");

        // Quick Shot Plan
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOT PLAN:");
        telemetry.addData("  Motif Pattern", indexingSystem.getMotifPattern());

        Artifact plannedFirst = indexingSystem.getPlannedFirstShot();
        Artifact plannedSecond = indexingSystem.getPlannedSecondShot();
        Artifact plannedThird = indexingSystem.getPlannedThirdShot();

        if (plannedFirst != null) {
            telemetry.addData("  1st Shot", String.format("%s #%d",
                plannedFirst.getColor(), plannedFirst.getCollectionOrder()));
        }
        if (plannedSecond != null) {
            telemetry.addData("  2nd Shot", String.format("%s #%d",
                plannedSecond.getColor(), plannedSecond.getCollectionOrder()));
        }
        if (plannedThird != null) {
            telemetry.addData("  3rd Shot", String.format("%s #%d",
                plannedThird.getColor(), plannedThird.getCollectionOrder()));
        }
    }

    /**
     * Display detailed indexing system page
     */
    private void displayIndexingPage() {
        telemetry.addLine("");
        telemetry.addLine("📦 INDEXING SYSTEM DETAILS:");
        telemetry.addData("  State", indexingSystem.getCurrentState());
        telemetry.addData("  Operation Active", indexingSystem.isOperationInProgress());
        telemetry.addData("  Auto Detection", indexingSystem.isAutoDetectionEnabled());
        telemetry.addData("  Manual Push Mode", indexingSystem.isManualPushMode());

        // Show all artifacts with details
        telemetry.addLine("");
        telemetry.addLine("📋 ALL ARTIFACTS:");
        for (Artifact artifact : indexingSystem.getAllArtifacts()) {
            if (artifact.getLocation() != Artifact.Location.FIRED) {
                telemetry.addData("  #" + artifact.getCollectionOrder(),
                    String.format("%s at %s", artifact.getColor(), artifact.getLocation()));
            }
        }

        // Error info
        String error = indexingSystem.getLastError();
        if (error != null && !error.isEmpty()) {
            telemetry.addLine("");
            telemetry.addData("⚠️ Last Error", error);
        }
    }

    /**
     * Display detailed shooter system page
     */
    private void displayShooterPage() {
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOOTER SYSTEM DETAILS:");
        telemetry.addData("  State", shooter.getState().toString());
        telemetry.addData("  Enabled", shooter.isEnabled());
        telemetry.addData("  Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
        telemetry.addData("  Target RPM", String.format("%.0f", shooter.getTargetRPM()));
        telemetry.addData("  Running", shooter.isRunning());
        telemetry.addData("  Error", shooter.isError());

        // RPM Tolerance Information
        telemetry.addLine("");
        telemetry.addLine("📊 RPM TOLERANCE & PERFORMANCE:");
        double currentRPM = shooter.getCurrentRPM();
        double targetRPM = shooter.getTargetRPM();
        double tolerance = shooterConfig.getRpmTolerance();
        double rpmError = Math.abs(currentRPM - targetRPM);
        boolean inTolerance = rpmError <= tolerance;

        telemetry.addData("  RPM Error", String.format("%.0f RPM", rpmError));
        telemetry.addData("  Tolerance", String.format("±%.0f RPM", tolerance));
        telemetry.addData("  Within Tolerance", inTolerance ? "YES ✓" : "NO");
        telemetry.addData("  Strict At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
        telemetry.addData("  RPM Percentage", String.format("%.1f%%", shooter.getRPMPercentage() * 100));
        telemetry.addData("  Sync Error", String.format("%.0f RPM", shooter.getSyncError()));

        // Stability Requirements
        telemetry.addLine("");
        telemetry.addLine("⏱️ STABILITY REQUIREMENTS:");
        telemetry.addData("  Required Stable Time", String.format("%.2fs", shooterConfig.getRpmStabilityTime()));
        telemetry.addData("  Shooting Tolerance", String.format("±%.0f RPM", shooterConfig.getShootingRpmTolerance()));

        // Uptake Servo Coordination
        telemetry.addLine("");
        telemetry.addLine("🔧 UPTAKE COORDINATION:");
        telemetry.addData("  Manual Control", "DPAD UP/DOWN for direct control");
        telemetry.addData("  Auto Pre-Position", "When artifact in center");
        telemetry.addData("  Auto Retract", "During push operations");
        telemetry.addData("  Firing Feed Time", String.format("%.1fs", ShooterConfig.UPTAKE_FEED_TIME_MS / 1000.0));
    }

    /**
     * Display shot planning analysis page
     */
    private void displayShotPlanningPage() {
        telemetry.addLine("");
        telemetry.addLine("🎯 SHOT PLANNING ANALYSIS:");

        // Planning context
        telemetry.addData("  Artifact Count", indexingSystem.getArtifactCount());
        telemetry.addData("  Manual Push Mode", indexingSystem.isManualPushMode());
        telemetry.addData("  Motif Pattern", indexingSystem.getMotifPattern());

        // Planning rules explanation
        telemetry.addLine("");
        telemetry.addLine("📋 PLANNING RULES:");
        int count = indexingSystem.getArtifactCount();
        if (count == 3) {
            telemetry.addLine("  • 3 Artifacts: Center fires first");
            telemetry.addLine("  • 2nd/3rd planned by motif");
        } else if (count == 2) {
            if (indexingSystem.isManualPushMode()) {
                telemetry.addLine("  • Manual Mode: Center fires first");
                telemetry.addLine("  • 2nd planned by motif");
            } else {
                telemetry.addLine("  • Auto Mode: 1st shot plannable");
                telemetry.addLine("  • Can rearrange for motif");
            }
        } else if (count == 1) {
            telemetry.addLine("  • Single artifact: Fire center");
        } else {
            telemetry.addLine("  • No artifacts to plan");
        }

        // Planned shots
        telemetry.addLine("");
        telemetry.addLine("🎯 PLANNED SHOTS:");

        Artifact plannedFirst = indexingSystem.getPlannedFirstShot();
        Artifact plannedSecond = indexingSystem.getPlannedSecondShot();
        Artifact plannedThird = indexingSystem.getPlannedThirdShot();

        if (plannedFirst != null) {
            telemetry.addData("  1st Shot", String.format("%s #%d from %s",
                plannedFirst.getColor(),
                plannedFirst.getCollectionOrder(),
                plannedFirst.getLocation()));
        } else {
            telemetry.addData("  1st Shot", "None planned");
        }

        if (plannedSecond != null) {
            telemetry.addData("  2nd Shot", String.format("%s #%d from %s",
                plannedSecond.getColor(),
                plannedSecond.getCollectionOrder(),
                plannedSecond.getLocation()));
        } else {
            telemetry.addData("  2nd Shot", "None planned");
        }

        if (plannedThird != null) {
            telemetry.addData("  3rd Shot", String.format("%s #%d from %s",
                plannedThird.getColor(),
                plannedThird.getCollectionOrder(),
                plannedThird.getLocation()));
        } else {
            telemetry.addData("  3rd Shot", "None planned");
        }

        // Motif analysis
        telemetry.addLine("");
        telemetry.addLine("🎨 MOTIF ANALYSIS:");
        String motif = indexingSystem.getMotifPattern();
        if (motif != null && motif.length() == 3) {
            telemetry.addData("  Desired 1st", motif.charAt(0) == 'P' ? "Purple" : "Green");
            telemetry.addData("  Desired 2nd", motif.charAt(1) == 'P' ? "Purple" : "Green");
            telemetry.addData("  Desired 3rd", motif.charAt(2) == 'P' ? "Purple" : "Green");
        } else {
            telemetry.addLine("  No motif pattern set");
        }
    }

    /**
     * Display firing sequence status page
     */
    private void displayFiringSequencePage() {
        telemetry.addLine("");

        if (firingSequenceActive) {
            telemetry.addLine("🚀 FIRING SEQUENCE ACTIVE:");
            telemetry.addData("  Current Shot", currentShotNumber);
            telemetry.addData("  Elapsed Time", String.format("%.1fs",
                (System.currentTimeMillis() - firingSequenceStartTime) / 1000.0));
            telemetry.addData("  Remaining Artifacts", indexingSystem.getArtifactCount());

            // Current status
            telemetry.addLine("");
            telemetry.addLine("📊 CURRENT STATUS:");
            telemetry.addData("  Shooter Ready", shooter.isAtTargetRPM() ? "YES" : "NO");
            telemetry.addData("  Indexing Ready", indexingSystem.isReadyToFire() ? "YES" : "NO");
            telemetry.addData("  System State", indexingSystem.getCurrentState());

            if (!shooter.isAtTargetRPM()) {
                telemetry.addLine("  ⏳ Waiting for shooter to reach target RPM");
            } else if (!indexingSystem.isReadyToFire()) {
                telemetry.addLine("  ⏳ Waiting for indexing system");
            } else {
                telemetry.addLine("  ✅ Ready to fire next shot");
            }

        } else {
            telemetry.addLine("🚀 FIRING SEQUENCE: IDLE");
            telemetry.addLine("");
            telemetry.addLine("Press [X] on Gamepad 1 to start");
            telemetry.addLine("firing sequence when artifacts ready");

            // Show what would happen
            if (indexingSystem.getArtifactCount() > 0) {
                telemetry.addLine("");
                telemetry.addLine("📋 SEQUENCE PREVIEW:");
                telemetry.addData("  Total Shots", indexingSystem.getArtifactCount());

                Artifact plannedFirst = indexingSystem.getPlannedFirstShot();
                Artifact plannedSecond = indexingSystem.getPlannedSecondShot();
                Artifact plannedThird = indexingSystem.getPlannedThirdShot();

                if (plannedFirst != null) {
                    telemetry.addData("  First", plannedFirst.getColor().toString());
                }
                if (plannedSecond != null) {
                    telemetry.addData("  Second", plannedSecond.getColor().toString());
                }
                if (plannedThird != null) {
                    telemetry.addData("  Third", plannedThird.getColor().toString());
                }
            }
        }
    }

    /**
     * Display system diagnostics page
     */
    private void displayDiagnosticsPage() {
        telemetry.addLine("");
        telemetry.addLine("🔧 SYSTEM DIAGNOSTICS:");

        // Performance metrics
        telemetry.addData("  Loop Time", String.format("%.1fms (Max: %.1fms)",
            (double)(System.currentTimeMillis() - loopStartTime), (double)maxLoopTime));
        telemetry.addData("  Total Loops", totalLoops);

        // Hardware status
        telemetry.addLine("");
        telemetry.addLine("🔌 HARDWARE STATUS:");
        telemetry.addData("  Indexing System", hardware.isIndexingSystemInitialized() ? "OK" : "ERROR");
        telemetry.addData("  Shooter System", hardware.isShooterSystemInitialized() ? "OK" : "ERROR");

        // Shooter motor power levels (if available)
        telemetry.addLine("");
        telemetry.addLine("⚡ SHOOTER MOTOR POWER:");
        if (hardware.getLeftShooterMotor() != null) {
            telemetry.addData("  Left Motor Power",
                String.format("%.2f", hardware.getLeftShooterMotor().getPower()));
        }
        if (hardware.getRightShooterMotor() != null) {
            telemetry.addData("  Right Motor Power",
                String.format("%.2f", hardware.getRightShooterMotor().getPower()));
        }

        // Error counts
        telemetry.addLine("");
        telemetry.addLine("⚠️ ERROR TRACKING:");
        telemetry.addData("  Indexing Errors", indexingSystem.getErrorCount());

        String indexingError = indexingSystem.getLastError();
        if (indexingError != null && !indexingError.isEmpty()) {
            telemetry.addData("  Last Indexing Error", indexingError);
        }
    }

    /**
     * Display debug messages page for uptake servo tracking
     */
    private void displayDebugPage() {
        telemetry.addLine("");
        telemetry.addLine("🔍 UPTAKE SERVO DEBUG:");

        // Current servo status
        boolean hasArtifactInCenter = indexingSystem.getArtifactInCenter() != null;
        boolean isPrePositioned = indexingSystem.isUptakeServoPrePositioned();
        boolean isCompleted = indexingSystem.isUptakeServoCompletedForCurrentArtifact();
        boolean isFiring = indexingSystem.getCurrentState() == IndexingSystem.SystemState.FIRING;
        boolean indexingControlling = isPrePositioned || isFiring;

        telemetry.addData("  Artifact in Center", hasArtifactInCenter ? "YES" : "NO");
        telemetry.addData("  Pre-Positioned", isPrePositioned ? "YES" : "NO");
        telemetry.addData("  Completed for Artifact", isCompleted ? "YES" : "NO");
        telemetry.addData("  Operation in Progress", indexingSystem.isOperationInProgress() ? "YES" : "NO");
        telemetry.addData("  🎛️ IndexingSystem Control", indexingControlling ? "ACTIVE (OpMode blocked)" : "Inactive (Manual OK)");

        // Show timing information if active
        if (isPrePositioned && indexingSystem.getUptakeServoActionTime() > 0) {
            long elapsed = indexingSystem.getUptakeServoElapsedTime();
            long remaining = indexingSystem.getUptakeServoRemainingTime();
            telemetry.addData("  ⏱️ Timer Status", String.format("%.0f/500ms (%.0fms left)",
                (double)elapsed, (double)remaining));
        } else if (indexingSystem.getUptakeServoActionTime() > 0) {
            telemetry.addData("  ⏱️ Timer Status", "Action time set but not pre-positioned");
        } else {
            telemetry.addData("  ⏱️ Timer Status", "Inactive");
        }

        // Show servo power levels
        if (hardware.getUptakeServoL() != null) {
            telemetry.addData("  Left Servo Power", String.format("%.2f", hardware.getUptakeServoL().getPower()));
        }
        if (hardware.getUptakeServoR() != null) {
            telemetry.addData("  Right Servo Power", String.format("%.2f", hardware.getUptakeServoR().getPower()));
        }

        // Show recent debug messages from IndexingSystem (ALL of them)
        java.util.List<String> debugMessages = indexingSystem.getRecentDebugMessages();
        if (!debugMessages.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine("📋 DEBUG MESSAGES:");
            // Show ALL debug messages (up to 10)
            for (int i = Math.max(0, debugMessages.size() - 10); i < debugMessages.size(); i++) {
                telemetry.addLine("  " + debugMessages.get(i));
            }
        }

        // System state context
        telemetry.addLine("");
        telemetry.addLine("🤖 SYSTEM STATE:");
        telemetry.addData("  Current State", indexingSystem.getCurrentState());
        telemetry.addData("  Artifact Count", String.format("%d/3", indexingSystem.getArtifactCount()));
    }

    /**
     * Display controls help page
     */
    private void displayControlsPage() {
        telemetry.addLine("");
        telemetry.addLine("🎮 GAMEPAD 1 - INDEXING:");
        telemetry.addLine("  [A] Simulate front intake");
        telemetry.addLine("  [B] Simulate back intake");
        telemetry.addLine("  [X] Start firing sequence");
        telemetry.addLine("  [Y] Emergency stop");
        telemetry.addLine("  [START] Reset systems");
        telemetry.addLine("  [BACK] Toggle debug");
        telemetry.addLine("  [L-STICK] Toggle manual push");
        telemetry.addLine("  [R-STICK] Execute manual push");
        telemetry.addLine("  [DPAD] Navigate pages");

        telemetry.addLine("");
        telemetry.addLine("🎮 GAMEPAD 2 - SHOOTER:");
        telemetry.addLine("  [A] Short Range (2000 RPM) + AUTO-FIRE");
        telemetry.addLine("  [B] Mid Range (2300 RPM) + AUTO-FIRE");
        telemetry.addLine("  [Y] Long Range (2800 RPM) + AUTO-FIRE");
        telemetry.addLine("  [X] Cycle motif pattern");
        telemetry.addLine("  [L-TRIGGER] Warmup mode (65%, no auto-fire)");
        telemetry.addLine("  [R-BUMPER] Spin up to target (manual)");
        telemetry.addLine("  [R-TRIGGER] Manual fire");
        telemetry.addLine("  [START] Stop shooter & cancel firing");
        telemetry.addLine("  [BACK] Clear shooter error");
        telemetry.addLine("");
        telemetry.addLine("  AUTO-FIRE: Hold A/B/Y buttons");
        telemetry.addLine("  • Spins up to preset RPM");
        telemetry.addLine("  • Fires all artifacts when ready");
        telemetry.addLine("  • Uses shot planning for optimal order");
        telemetry.addLine("");
        telemetry.addLine("  AUTO-CANCEL: Release A/B/Y buttons");
        telemetry.addLine("  • Cancels active firing sequence");
        telemetry.addLine("  • Stops shooter (unless manual mode)");
        telemetry.addLine("  • Manual controls override auto-cancel");
        telemetry.addLine("");
        telemetry.addLine("  DPAD Controls:");
        telemetry.addLine("  [DPAD-UP] Feed into shooter");
        telemetry.addLine("  [DPAD-DOWN] Retract from shooter");
        telemetry.addLine("  [LB+DPAD] RPM adjustment:");
        telemetry.addLine("    LB+UP/DOWN: ±100 RPM");
        telemetry.addLine("    LB+LEFT/RIGHT: ±500 RPM");

        telemetry.addLine("");
        telemetry.addLine("📊 TELEMETRY PAGES:");
        telemetry.addLine("  OVERVIEW - Main status");
        telemetry.addLine("  INDEXING - Indexing details");
        telemetry.addLine("  SHOOTER - Shooter details");
        telemetry.addLine("  SHOT_PLANNING - Planning analysis");
        telemetry.addLine("  FIRING_SEQUENCE - Sequence status");
        telemetry.addLine("  DIAGNOSTICS - System health");
        telemetry.addLine("  FIRING_TREE - Condition hierarchy");
        telemetry.addLine("  CONTROLS - This help");
    }

    /**
     * Display hierarchical firing conditions tree
     * Only shows failing branches to keep it uncluttered
     */
    private void displayFiringTreePage() {
        telemetry.addLine("");
        telemetry.addLine("🌳 COMPLETE FIRING CONDITIONS HIERARCHY");
        telemetry.addLine("");

        // Top level: Will Auto-Fire?
        boolean canAutoFire = canAutoFire();
        displayCondition("🚀 AUTO-FIRE", canAutoFire, 0);

        if (!canAutoFire) {
            // Level 1: Check ALL main auto-fire conditions
            boolean hasArtifacts = indexingSystem.getArtifactCount() > 0;
            boolean notFiring = !firingSequenceActive;
            boolean shooterReady = isShooterReadyForAutoFiring();
            boolean indexingReady = indexingSystem.isReadyToFire();
            boolean noOperation = !indexingSystem.isOperationInProgress();

            displayCondition("├─ Has Artifacts", hasArtifacts, 1);
            if (!hasArtifacts) {
                displayCondition("│  └─ Count: " + indexingSystem.getArtifactCount(), false, 2);
            }

            displayCondition("├─ Not Currently Firing", notFiring, 1);
            if (!notFiring) {
                displayCondition("│  └─ firingSequenceActive = true", false, 2);
            }

            displayCondition("├─ Shooter Ready", shooterReady, 1);
            if (!shooterReady) {
                expandShooterReadiness(2);
            }

            displayCondition("├─ Indexing Ready", indexingReady, 1);
            if (!indexingReady) {
                expandIndexingReadiness(2);
            }

            displayCondition("└─ No Operation In Progress", noOperation, 1);
            if (!noOperation) {
                expandOperationInProgress(2);
            }
        }

        // Show current button states for context
        telemetry.addLine("");
        telemetry.addLine("🎮 ACTIVE BUTTONS:");
        String activeButtons = "";
        if (gamepad2.a) activeButtons += "GP2.A ";
        if (gamepad2.b) activeButtons += "GP2.B ";
        if (gamepad2.y) activeButtons += "GP2.Y ";
        if (gamepad1.x) activeButtons += "GP1.X ";
        telemetry.addData("  Currently Held", activeButtons.isEmpty() ? "None" : activeButtons.trim());

        if (!activeButtons.isEmpty()) {
            telemetry.addData("  Auto-Fire Status", canAutoFire ? "🟢 FIRING!" : "🔴 BLOCKED");
        }
    }

    /**
     * Helper method to check if auto-firing is possible
     */
    private boolean canAutoFire() {
        return indexingSystem.getArtifactCount() > 0 &&
               !firingSequenceActive &&
               isShooterReadyForAutoFiring() &&
               indexingSystem.isReadyToFire() &&
               !indexingSystem.isOperationInProgress();
    }

    /**
     * Expand shooter readiness conditions
     */
    private void expandShooterReadiness(int indent) {
        boolean enabled = shooter.isEnabled();
        boolean atTargetRPM = shooter.isAtTargetRPM();

        displayCondition("├─ Shooter Enabled", enabled, indent);
        if (!enabled) {
            displayCondition("│  └─ Call shooter.enable() needed", false, indent + 1);
        }

        displayCondition("└─ At Target RPM", atTargetRPM, indent);
        if (!atTargetRPM) {
            expandRPMConditions(indent + 1);
        }
    }

    /**
     * Expand RPM conditions
     */
    private void expandRPMConditions(int indent) {
        boolean running = shooter.isRunning();
        boolean stateReady = shooter.getState().toString().equals("READY");
        double currentRPM = shooter.getCurrentRPM();
        double targetRPM = shooter.getTargetRPM();
        double rpmDiff = Math.abs(currentRPM - targetRPM);
        boolean withinTolerance = rpmDiff <= 75.0; // Our corrected tolerance

        displayCondition("├─ Shooter Running", running, indent);
        if (!running) {
            displayCondition("│  └─ Call shooter.spinUp() needed", false, indent + 1);
        }

        displayCondition("├─ State = READY", stateReady, indent);
        if (!stateReady) {
            String currentState = shooter.getState().toString();
            displayCondition("│  └─ Current: " + currentState, false, indent + 1);
            if (currentState.equals("SPINNING_UP")) {
                expandSpinningUpConditions(indent + 1);
            }
        }

        displayCondition("└─ Within RPM Tolerance", withinTolerance, indent);
        if (!withinTolerance) {
            displayCondition("   ├─ Current: " + String.format("%.0f RPM", currentRPM), false, indent + 1);
            displayCondition("   ├─ Target: " + String.format("%.0f RPM", targetRPM), false, indent + 1);
            displayCondition("   └─ Difference: " + String.format("%.0f RPM (>75)", rpmDiff), false, indent + 1);
        }
    }

    /**
     * Expand spinning up conditions
     */
    private void expandSpinningUpConditions(int indent) {
        double syncError = shooter.getSyncError();
        boolean syncOK = syncError <= 150.0;

        displayCondition("└─ Motor Sync", syncOK, indent);
        if (!syncOK) {
            displayCondition("   └─ Sync Error: " + String.format("%.0f RPM (>150)", syncError), false, indent + 1);
        }
    }

    /**
     * Expand IndexingSystem readiness conditions
     * IndexingSystem.isReadyToFire() = (currentState == READY_TO_FIRE && artifactInCenter != null)
     */
    private void expandIndexingReadiness(int indent) {
        // Get IndexingSystem state - we need to check what it actually requires
        String currentState = indexingSystem.getCurrentState().toString();
        boolean stateReady = currentState.equals("READY_TO_FIRE");
        boolean hasArtifactInCenter = indexingSystem.getArtifactInCenter() != null;

        displayCondition("├─ State = READY_TO_FIRE", stateReady, indent);
        if (!stateReady) {
            displayCondition("│  ├─ Current State: " + currentState, false, indent + 1);
            // Could expand further based on current state
            if (currentState.equals("WAITING_FOR_ARTIFACT")) {
                displayCondition("│     └─ Need artifact in center position", false, indent + 2);
            } else if (currentState.equals("POSITIONING")) {
                displayCondition("│     └─ Still positioning artifact", false, indent + 2);
            }
        }

        displayCondition("└─ Artifact In Center", hasArtifactInCenter, indent);
        if (!hasArtifactInCenter) {
            displayCondition("   └─ No artifact in center position", false, indent + 1);
            displayCondition("   └─ Need to load/position artifact", false, indent + 1);
        }
    }

    /**
     * Expand operation in progress conditions
     */
    private void expandOperationInProgress(int indent) {
        // IndexingSystem.isOperationInProgress() checks operationInProgress field
        // This could be true during various operations like pushing, positioning, etc.
        displayCondition("└─ Operation Details:", false, indent);
        displayCondition("   ├─ Uptake servo moving: " + (indexingSystem.isUptakeServoPrePositioned() ? "YES" : "NO"),
                         indexingSystem.isUptakeServoPrePositioned(), indent + 1);

        // Check current indexing state for more details
        String currentState = indexingSystem.getCurrentState().toString();
        displayCondition("   └─ Current State: " + currentState, false, indent + 1);

        if (currentState.contains("PUSHING")) {
            displayCondition("      └─ Push operation active", false, indent + 2);
        } else if (currentState.contains("POSITIONING")) {
            displayCondition("      └─ Positioning operation active", false, indent + 2);
        } else if (currentState.contains("FEEDING")) {
            displayCondition("      └─ Feeding operation active", false, indent + 2);
        }
    }

    /**
     * Display firing activation analysis page - why firingSequenceActive never becomes true
     */
    private void displayFiringActivationPage() {
        telemetry.addLine("");
        telemetry.addLine("🔥 FIRING SEQUENCE ACTIVATION ANALYSIS");
        telemetry.addLine("");

        // Current state
        telemetry.addData("🎯 firingSequenceActive", firingSequenceActive ? "TRUE ✅" : "FALSE ❌");

        if (firingSequenceActive) {
            telemetry.addData("  Set at time", firingSequenceStartTime);
            telemetry.addData("  Current shot #", currentShotNumber);
            long elapsed = System.currentTimeMillis() - firingSequenceStartTime;
            telemetry.addData("  Running for", elapsed + "ms");
        } else {
            telemetry.addLine("");
            telemetry.addLine("🔍 WHY IS IT FALSE?");

            // Check if any auto-fire buttons are pressed
            boolean anyButtonPressed = gamepad2.a || gamepad2.b || gamepad2.y || gamepad1.x;
            displayCondition("🎮 Auto-fire button pressed", anyButtonPressed, 0);

            if (anyButtonPressed) {
                // Show which buttons are pressed
                telemetry.addLine("  Active buttons:");
                if (gamepad2.a) telemetry.addLine("    • GP2.A (Short Range)");
                if (gamepad2.b) telemetry.addLine("    • GP2.B (Mid Range)");
                if (gamepad2.y) telemetry.addLine("    • GP2.Y (Long Range)");
                if (gamepad1.x) telemetry.addLine("    • GP1.X (Manual Fire)");

                // Check if handleAutoFiring() is being called
                displayCondition("├─ handleAutoFiring() called", anyButtonPressed, 1);

                if (anyButtonPressed) {
                    // Check all conditions that prevent startFiringSequence()
                    // These variable names MUST match exactly those in handleAutoFiring()
                    boolean hasArtifacts = indexingSystem.getArtifactCount() > 0;
                    boolean notFiring = !firingSequenceActive;  // EXACT match with handleAutoFiring()
                    boolean shooterReady = isShooterReadyForAutoFiring();
                    boolean indexingReady = indexingSystem.isReadyToFire();
                    boolean noOperation = !indexingSystem.isOperationInProgress();

                    displayCondition("├─ Has artifacts", hasArtifacts, 1);
                    if (!hasArtifacts) {
                        displayCondition("│  └─ Count: " + indexingSystem.getArtifactCount(), false, 2);
                        displayCondition("│  └─ SOLUTION: Load artifacts with GP1.A/B", false, 2);
                    }

                    displayCondition("├─ Not firing", notFiring, 1); // EXACT match
                    // This should always be true since firingSequenceActive is false

                    displayCondition("├─ Shooter ready", shooterReady, 1);
                    if (!shooterReady) {
                        expandShooterReadinessForActivation(2);
                    }

                    displayCondition("├─ Indexing ready", indexingReady, 1);
                    if (!indexingReady) {
                        expandIndexingReadinessForActivation(2);
                    }

                    displayCondition("└─ No operation in progress", noOperation, 1);
                    if (!noOperation) {
                        expandOperationInProgressForActivation(2);
                    }

                    // Final check: EXACT same condition as handleAutoFiring()
                    // if (hasArtifacts && notFiring && shooterReady && indexingReady && noOperation)
                    boolean allConditionsMet = hasArtifacts && notFiring && shooterReady && indexingReady && noOperation;
                    telemetry.addLine("");
                    displayCondition("🚀 WOULD CALL startFiringSequence()", allConditionsMet, 0);
                    telemetry.addData("  This matches line ~650", "handleAutoFiring() if-condition");

                    if (allConditionsMet) {
                        telemetry.addData("  ⚠️ PROBLEM", "All conditions met but firingSequenceActive still false!");
                        telemetry.addData("  🔍 CHECK", "Is startFiringSequence() actually being called?");
                        telemetry.addData("  🔍 CHECK", "Look for 'startFiringSequence() CALLED!' in OVERVIEW page");
                        telemetry.addData("  🔍 CHECK", "Look for '🚀 CALLING startFiringSequence()!' in OVERVIEW page");
                    }
                }

            } else {
                displayCondition("  └─ SOLUTION: Hold GP2.A/B/Y or GP1.X", false, 1);
            }
        }

        // Show places where firingSequenceActive gets set
        telemetry.addLine("");
        telemetry.addLine("📍 PLACES WHERE firingSequenceActive = true:");
        telemetry.addLine("  • startFiringSequence() - line ~400");
        telemetry.addLine("  • Called from handleAutoFiring() when conditions met");
        telemetry.addLine("  • Called from GP1.X manual fire (different path)");

        telemetry.addLine("");
        telemetry.addLine("📍 PLACES WHERE firingSequenceActive = false:");
        telemetry.addLine("  • completeFiringSequence() - when done");
        telemetry.addLine("  • emergencyStop() - GP1.Y pressed");
        telemetry.addLine("  • resetSystems() - GP1.START pressed");
        telemetry.addLine("  • Initial state (false by default)");

        // Real-time call tracking
        telemetry.addLine("");
        telemetry.addLine("🔍 REAL-TIME CALL TRACKING:");
        telemetry.addData("  Look for this in OVERVIEW", "'startFiringSequence() CALLED!'");
        telemetry.addData("  If missing", "startFiringSequence() never called");
        telemetry.addData("  If present", "Something in startFiringSequence() fails");
    }

    /**
     * Expand shooter readiness for firing activation analysis
     */
    private void expandShooterReadinessForActivation(int indent) {
        boolean enabled = shooter.isEnabled();
        boolean atTargetRPM = shooter.isAtTargetRPM();
        String state = shooter.getState().toString();

        displayCondition("├─ Enabled", enabled, indent);
        if (!enabled) {
            displayCondition("│  └─ SOLUTION: Should auto-enable when button pressed", false, indent + 1);
        }

        displayCondition("├─ At Target RPM", atTargetRPM, indent);
        if (!atTargetRPM) {
            displayCondition("│  ├─ Current RPM: " + String.format("%.0f", shooter.getCurrentRPM()), false, indent + 1);
            displayCondition("│  ├─ Target RPM: " + String.format("%.0f", shooter.getTargetRPM()), false, indent + 1);
            displayCondition("│  └─ State: " + state, false, indent + 1);
            if (state.equals("SPINNING_UP")) {
                displayCondition("│     └─ WAIT: Still spinning up", false, indent + 2);
            } else if (state.equals("IDLE")) {
                displayCondition("│     └─ PROBLEM: Should spin up when button pressed", false, indent + 2);
            }
        }

        displayCondition("└─ State Ready", state.equals("READY"), indent);
        if (!state.equals("READY")) {
            displayCondition("   └─ Current: " + state, false, indent + 1);
        }
    }

    /**
     * Expand indexing readiness for firing activation analysis
     */
    private void expandIndexingReadinessForActivation(int indent) {
        String currentState = indexingSystem.getCurrentState().toString();
        boolean stateReady = currentState.equals("READY_TO_FIRE");
        boolean hasArtifactInCenter = indexingSystem.getArtifactInCenter() != null;

        displayCondition("├─ State = READY_TO_FIRE", stateReady, indent);
        if (!stateReady) {
            displayCondition("│  ├─ Current State: " + currentState, false, indent + 1);
            if (currentState.equals("IDLE")) {
                displayCondition("│  └─ PROBLEM: Should advance when artifacts loaded", false, indent + 1);
            } else if (currentState.equals("WAITING_FOR_ARTIFACT")) {
                displayCondition("│  └─ SOLUTION: Load artifact in center position", false, indent + 1);
            } else if (currentState.equals("POSITIONING")) {
                displayCondition("│  └─ WAIT: Still positioning artifact", false, indent + 1);
            }
        }

        displayCondition("└─ Artifact in center", hasArtifactInCenter, indent);
        if (!hasArtifactInCenter) {
            displayCondition("   ├─ Center position empty", false, indent + 1);
            displayCondition("   ├─ Front: " + (indexingSystem.getArtifactInFrontIntake() != null ? "Has artifact" : "Empty"),
                           indexingSystem.getArtifactInFrontIntake() != null, indent + 1);
            displayCondition("   ├─ Back: " + (indexingSystem.getArtifactInBackIntake() != null ? "Has artifact" : "Empty"),
                           indexingSystem.getArtifactInBackIntake() != null, indent + 1);
            displayCondition("   └─ SOLUTION: Load artifacts and wait for positioning", false, indent + 1);
        }
    }

    /**
     * Expand operation in progress for firing activation analysis
     */
    private void expandOperationInProgressForActivation(int indent) {
        boolean operationInProgress = indexingSystem.isOperationInProgress();
        String currentState = indexingSystem.getCurrentState().toString();
        boolean servoPrePositioned = indexingSystem.isUptakeServoPrePositioned();

        displayCondition("└─ Operation type analysis:", false, indent);
        displayCondition("   ├─ Uptake servo moving: " + (servoPrePositioned ? "YES" : "NO"), servoPrePositioned, indent + 1);
        displayCondition("   ├─ Current State: " + currentState, false, indent + 1);

        if (currentState.contains("PUSHING")) {
            displayCondition("   └─ WAIT: Push operation active", false, indent + 1);
        } else if (currentState.contains("POSITIONING")) {
            displayCondition("   └─ WAIT: Positioning operation active", false, indent + 1);
        } else if (currentState.contains("FEEDING")) {
            displayCondition("   └─ WAIT: Feeding operation active", false, indent + 1);
        } else if (operationInProgress) {
            displayCondition("   └─ UNKNOWN: Operation type not identified", false, indent + 1);
        } else {
            displayCondition("   └─ READY: No operations running", true, indent + 1);
        }
    }

    /**
     * Helper method to display a condition with proper indentation
     */
    private void displayCondition(String condition, boolean status, int indent) {
        String prefix = "";
        for (int i = 0; i < indent; i++) {
            prefix += "  ";
        }

        String statusIcon = status ? "✅" : "❌";
        telemetry.addLine(prefix + statusIcon + " " + condition);
    }

    /**
     * Get next telemetry page
     */
    private TelemetryPage getNextPage(TelemetryPage current) {
        TelemetryPage[] pages = TelemetryPage.values();
        int currentIndex = current.ordinal();
        return pages[(currentIndex + 1) % pages.length];
    }

    /**
     * Get previous telemetry page
     */
    private TelemetryPage getPreviousPage(TelemetryPage current) {
        TelemetryPage[] pages = TelemetryPage.values();
        int currentIndex = current.ordinal();
        return pages[(currentIndex - 1 + pages.length) % pages.length];
    }

    /**
     * Update performance metrics
     */
    private void updatePerformanceMetrics() {
        long loopTime = System.currentTimeMillis() - loopStartTime;
        maxLoopTime = Math.max(maxLoopTime, loopTime);
        totalLoops++;
    }
}

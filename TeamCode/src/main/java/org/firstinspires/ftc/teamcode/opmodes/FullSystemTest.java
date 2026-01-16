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
        CONTROLS                // Control help and mappings
    }
    private TelemetryPage currentPage = TelemetryPage.OVERVIEW;

    // Shot Planning and Firing State
    private boolean firingSequenceActive = false;
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

    // RPM Stability Tracking for Auto-Firing
    private long rpmStableStartTime = 0;
    private boolean rpmWasStable = false;

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

        // [X] - Start firing sequence
        if (gamepad1.x && !lastX1) {
            if (indexingSystem.getArtifactCount() > 0 && !firingSequenceActive) {
                // Set default preset if none is set
                if (shooter.getTargetRPM() <= 0) {
                    shooter.setPreset(ShooterConfig.ShooterPreset.MID_RANGE);  // Default to mid-range
                    telemetry.addLine("🎯 Using default MID_RANGE preset for firing");
                }
                startFiringSequence();
            } else if (firingSequenceActive) {
                telemetry.addLine("⚠️ Firing sequence already active");
            } else {
                telemetry.addLine("⚠️ No artifacts to fire");
            }
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

        handleShooterPresetControls();
        handleShooterDpadControls();
        handleShooterTestingControls();
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
        if (indexingSystem.getArtifactCount() == 0) {
            telemetry.addLine("⚠️ No artifacts to fire");
            return;
        }

        firingSequenceActive = true;
        firingSequenceStartTime = System.currentTimeMillis();
        currentShotNumber = 1;

        // Ensure shooter is spinning up
        if (!shooter.isAtTargetRPM()) {
            shooter.spinUp();
        }

        telemetry.addLine("🚀 FIRING SEQUENCE STARTED");
        telemetry.addLine(String.format("   Artifacts: %d", indexingSystem.getArtifactCount()));
        telemetry.addLine(String.format("   Motif: %s", indexingSystem.getMotifPattern()));
    }

    /**
     * Handle the active firing sequence
     */
    private void handleFiringSequence() {
        if (!firingSequenceActive) return;

        // Check if we still have artifacts to fire
        if (indexingSystem.getArtifactCount() == 0) {
            completeFiringSequence();
            return;
        }

        // Wait for shooter to be ready
        if (!shooter.isAtTargetRPM()) {
            return; // Wait for shooter to spin up
        }

        // Check if indexing system is ready to fire
        if (indexingSystem.isReadyToFire() && !indexingSystem.isOperationInProgress()) {
            // Fire the current shot
            boolean fired = indexingSystem.onFireSignal();
            if (fired) {
                telemetry.addLine(String.format("🔥 Shot %d fired!", currentShotNumber));
                currentShotNumber++;
            }
        }

        // Safety timeout (30 seconds)
        if (System.currentTimeMillis() - firingSequenceStartTime > 30000) {
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
        boolean anyPresetActive = false;

        // A Button - Short Range
        if (gamepad2.a) {
            shooter.setPreset(ShooterConfig.ShooterPreset.SHORT_RANGE);
            shooter.spinUp();
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
            if (!lastA2) {
                telemetry.addLine("🎯 Short Range (2000 RPM) - Auto-firing when ready");
            }

            // Auto-start firing sequence if shooter ready and artifacts available
            handleAutoFiring();
        }
        lastA2 = gamepad2.a;

        // B Button - Mid Range
        if (gamepad2.b) {
            shooter.setPreset(ShooterConfig.ShooterPreset.MID_RANGE);
            shooter.spinUp();
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
            if (!lastB2) {
                telemetry.addLine("🎯 Mid Range (2300 RPM) - Auto-firing when ready");
            }

            // Auto-start firing sequence if shooter ready and artifacts available
            handleAutoFiring();
        }
        lastB2 = gamepad2.b;

        // Y Button - Long Range
        if (gamepad2.y) {
            shooter.setPreset(ShooterConfig.ShooterPreset.LONG_RANGE);
            shooter.spinUp();
            controlMode = ControlMode.PRESET;
            anyPresetActive = true;
            if (!lastY2) {
                telemetry.addLine("🎯 Long Range (2800 RPM) - Auto-firing when ready");
            }

            // Auto-start firing sequence if shooter ready and artifacts available
            handleAutoFiring();
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

            // Reset RPM stability tracking
            rpmWasStable = false;
            rpmStableStartTime = 0;
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
        // Only auto-fire if we have artifacts and not already firing
        if (indexingSystem.getArtifactCount() > 0 &&
            !firingSequenceActive &&
            isShooterReadyForAutoFiring() &&
            indexingSystem.isReadyToFire() &&
            !indexingSystem.isOperationInProgress()) {

            // Start the automated firing sequence
            startFiringSequence();
            telemetry.addLine("🚀 AUTO-FIRING: Preset button held, starting sequence");
        }
    }

    /**
     * Check if shooter is ready for auto-firing with more tolerant RPM requirements
     * Uses configurable tolerance and requires stability for minimum time
     */
    private boolean isShooterReadyForAutoFiring() {
        if (!shooter.isEnabled() || !shooter.isRunning()) {
            rpmWasStable = false;
            rpmStableStartTime = 0;
            return false;
        }

        // Check if shooter RPM is within acceptable tolerance of target
        double currentRPM = shooter.getCurrentRPM();
        double targetRPM = shooter.getTargetRPM();
        double tolerance = shooterConfig.getRpmTolerance(); // Use the configurable tolerance (100 RPM)

        boolean rpmInRange = Math.abs(currentRPM - targetRPM) <= tolerance;

        // Track stability time
        long currentTime = System.currentTimeMillis();
        if (rpmInRange) {
            if (!rpmWasStable) {
                // RPM just entered stable range
                rpmStableStartTime = currentTime;
                rpmWasStable = true;
            }
        } else {
            // RPM out of range, reset stability tracking
            rpmWasStable = false;
            rpmStableStartTime = 0;
        }

        // Check if RPM has been stable for minimum required time
        double requiredStabilityTime = shooterConfig.getRpmStabilityTime() * 1000; // Convert to milliseconds
        boolean stabilityTimeReached = rpmWasStable &&
            (currentTime - rpmStableStartTime) >= requiredStabilityTime;

        // For debugging - show RPM status in telemetry when auto-fire buttons are held
        if (gamepad2.a || gamepad2.b || gamepad2.y) {
            double stabilityElapsed = rpmWasStable ? (currentTime - rpmStableStartTime) / 1000.0 : 0.0;
            telemetry.addLine(String.format("🎯 AUTO-FIRE RPM: %.0f/%.0f (±%.0f) %s",
                currentRPM, targetRPM, tolerance, rpmInRange ? "✓" : "✗"));
            telemetry.addLine(String.format("   Stability: %.2fs/%.2fs %s",
                stabilityElapsed, requiredStabilityTime / 1000.0,
                stabilityTimeReached ? "READY ✓" : "waiting..."));
        }

        return rpmInRange && stabilityTimeReached;
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
     * Handle testing controls (spin up, fire, stop, etc.)
     */
    private void handleShooterTestingControls() {
        // Right Bumper - Spin up to target
        if (gamepad2.right_bumper && !lastRightBumper2) {
            if (controlMode == ControlMode.MANUAL) {
                shooter.setTargetRPM(manualTargetRPM);
                telemetry.addLine(String.format("🎯 Spinning up to manual RPM: %.0f", manualTargetRPM));
            }
            shooter.spinUp();
            controlMode = ControlMode.MANUAL; // Mark as manually activated
        }
        lastRightBumper2 = gamepad2.right_bumper;

        // Right Trigger - Manual fire
        if (gamepad2.right_trigger > 0.5 && !lastRightTrigger2) {
            if (shooter.isReadyToFire()) {
                shooter.fire();
                telemetry.addLine("🔥 Manual fire triggered");
            } else {
                telemetry.addLine("⚠️ Shooter not ready to fire");
            }
        }
        lastRightTrigger2 = gamepad2.right_trigger > 0.5;

        // START - Stop shooter (always stops regardless of activation method)
        if (gamepad2.start && !lastStart2) {
            shooter.stop();
            controlMode = ControlMode.PRESET; // Reset to preset mode after manual stop

            // Also cancel any active firing sequence
            if (firingSequenceActive) {
                completeFiringSequence();
                telemetry.addLine("🛑 Firing sequence cancelled");
            }

            // Reset RPM stability tracking
            rpmWasStable = false;
            rpmStableStartTime = 0;

            telemetry.addLine("⏹️ Shooter stopped manually");
        }
        lastStart2 = gamepad2.start;

        // BACK - Clear error
        if (gamepad2.back && !lastBack2) {
            shooter.clearError();
            telemetry.addLine("🔄 Shooter error cleared");
        }
        lastBack2 = gamepad2.back;
    }

    /**
     * Update telemetry display based on current page
     */
    private void updateTelemetryDisplay() {
        telemetry.clear();

        // Common header
        telemetry.addLine("🚀 FULL SYSTEM TEST");
        telemetry.addData("Page", currentPage.toString() + " (" + (currentPage.ordinal() + 1) + "/8)");
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
        telemetry.addLine("  CONTROLS - This help");
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

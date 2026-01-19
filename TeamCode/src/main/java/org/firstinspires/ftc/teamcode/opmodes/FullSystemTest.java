package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingSystem;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Artifact;
import org.firstinspires.ftc.teamcode.util.aurora.FiringSequenceCoordinator;
import org.firstinspires.ftc.teamcode.util.aurora.SystemMonitor;

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
 *   [BACK] - Cycle debug display modes (FULL/SUMMARY/BOOLEAN_TREE/RECENT/BY_CLASS/PRIORITY)
 *   [DPAD-RIGHT] - Cycle through class pages (when in BY_CLASS mode)
 *   [L-STICK] - Toggle manual push mode
 *   [R-STICK] - Execute manual push
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
    private FiringSequenceCoordinator firingCoordinator;

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

    // Shot Planning and Firing State
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
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        // Initialize configurations
        indexingConfig = new IndexingConfig();
        indexingConfig.setDebugTelemetry(false);

        shooterConfig = new ShooterConfig();

        // Initialize systems with shared debug logger
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        indexingSystem = new IndexingSystem(hardware, indexingConfig, shooter, telemetry);
        firingCoordinator = new FiringSequenceCoordinator(indexingSystem, shooter);

        waitForStart();

        if (isStopRequested()) return;

        // Enable systems
        shooter.enable();
        indexingSystem.enable();

        // Main control loop
        while (opModeIsActive()) {
            loopStartTime = System.currentTimeMillis();

            // Handle gamepad inputs
            handleGamepadInputs();

            // Update all systems
            indexingSystem.update();
            shooter.update();
            firingCoordinator.update();

            // Update system mode based on current state
            updateSystemMode();

            // Display SystemMonitor on telemetry
            SystemMonitor.displayOnTelemetry(telemetry);
            telemetry.update();

            // Performance tracking
            updatePerformanceMetrics();

            sleep(20); // 50Hz update rate
        }

        // Cleanup
        firingCoordinator.reset();
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
        }
        lastB1 = gamepad1.b;

        // [X] - Start firing sequence (hold to spin up and fire, release to cancel)
        if (gamepad1.x) {
            if (indexingSystem.getArtifactCount() > 0) {
                // Set default preset if none is set
                if (shooter.getTargetRPM() <= 0) {
                    shooter.setPreset(ShooterConfig.ShooterPreset.MID_RANGE);
                }

                // Spin up shooter
                shooter.spinUp();

                // Auto-start firing sequence if shooter ready and artifacts available
                if (!firingCoordinator.isFiringActive() &&
                    firingCoordinator.canStartFiring()) {
                    firingCoordinator.startFiring();
                }
            }
        } else if (lastX1) {
            // X button released after being held - cancel firing and stop shooter
            if (firingCoordinator.isFiringActive()) {
                firingCoordinator.completeFiring();
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

        // [BACK] - No longer used (was for cycling debug display modes)
        lastBack1 = gamepad1.back;
        
        // [DPAD RIGHT] - No longer used (was for cycling debug pages)
        lastDpadRight1 = gamepad1.dpad_right;

        // [L-STICK] - Toggle manual push mode
        if (gamepad1.left_stick_button && !lastLeftStick1) {
            boolean manual = !indexingSystem.isManualPushMode();
            indexingSystem.setManualPushMode(manual);
        }
        lastLeftStick1 = gamepad1.left_stick_button;

        // [R-STICK] - Execute manual push
        if (gamepad1.right_stick_button && !lastRightStick1) {
            indexingSystem.onManualPush();
        }
        lastRightStick1 = gamepad1.right_stick_button;

        // DPAD - Other directions unused
        lastDpadUp1 = gamepad1.dpad_up;
        lastDpadDown1 = gamepad1.dpad_down;
        lastDpadLeft1 = gamepad1.dpad_left;

        // ═══════════════════════════════════════════════════════════════
        // GAMEPAD 2 - Shooter & Advanced Controls
        // ═══════════════════════════════════════════════════════════════

        handleShooterPresetControls();
        handleShooterDpadControls();
    }

    /**
     * Update system mode based on current state of both systems
     */
    private void updateSystemMode() {
        if (firingCoordinator.isFiringActive()) {
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
     * Emergency stop all systems
     */
    private void emergencyStop() {
        firingCoordinator.emergencyStop();
        hardware.stopAllMotors();
    }

    /**
     * Reset both systems to initial state
     */
    private void resetSystems() {
        firingCoordinator.reset();
        indexingSystem.reset();
        shooter.stop();
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

            // Auto-start firing sequence if shooter ready and artifacts available
            handleAutoFiring();
        }
        lastY2 = gamepad2.y;

        // Check if preset buttons were released - cancel auto-firing and stop shooter
        if (!anyPresetActive && controlMode == ControlMode.PRESET && presetButtonsWereActive) {
            // Preset button released after being held - cancel auto-firing and stop shooter
            if (firingCoordinator.isFiringActive()) {
                firingCoordinator.completeFiring();
            }

            // Stop shooter unless it was activated by manual controls
            if (!wasActivatedManually()) {
                shooter.stopMotors();
            }
        }

        // Update preset button state tracking
        presetButtonsWereActive = anyPresetActive;

        // X Button - Cycle motif pattern
        if (gamepad2.x && !lastX2) {
            currentMotifIndex = (currentMotifIndex + 1) % motifPatterns.length;
            String newPattern = motifPatterns[currentMotifIndex];
            indexingSystem.setMotifPattern(newPattern);
        }
        lastX2 = gamepad2.x;

        // Left Trigger - Warmup Mode (no auto-firing in warmup)
        boolean warmupActive = gamepad2.left_trigger > 0.3;
        boolean lastWarmupActive = lastLeftTrigger2 > 0.3;

        if (warmupActive) {
            // Enable warmup mode
            if (!shooter.isRunning() || !shooter.isAtTargetRPM()) {
                double targetRPM = (controlMode == ControlMode.MANUAL) ? manualTargetRPM : shooter.getTargetRPM();
                double warmupRPM = ShooterConfig.getWarmupRPM(targetRPM);
                shooter.setTargetRPM(warmupRPM);
                shooter.spinUp();
            }
        } else if (lastWarmupActive && !warmupActive) {
            // Warmup trigger released - stop shooter unless other controls are active
            if (!wasActivatedManually() && !anyPresetActive) {
                shooter.stopMotors();
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
     * Delegates to FiringSequenceCoordinator for condition checking and execution
     */
    private void handleAutoFiring() {
        if (firingCoordinator.canStartFiring()) {
            firingCoordinator.startFiring();
        }
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
            }

            // DPAD DOWN - Decrease by 100
            if (gamepad2.dpad_down && !lastDpadDown2) {
                manualTargetRPM -= 100;
                manualTargetRPM = Math.max(manualTargetRPM, 0);
                controlMode = ControlMode.MANUAL;
            }

            // DPAD RIGHT - Increase by 500
            if (gamepad2.dpad_right && !lastDpadRight2) {
                manualTargetRPM += 500;
                manualTargetRPM = Math.min(manualTargetRPM, 6000);
                controlMode = ControlMode.MANUAL;
            }

            // DPAD LEFT - Decrease by 500
            if (gamepad2.dpad_left && !lastDpadLeft2) {
                manualTargetRPM -= 500;
                manualTargetRPM = Math.max(manualTargetRPM, 0);
                controlMode = ControlMode.MANUAL;
            }

        } else {
            // DEFAULT MODE - Uptake Servo Control
            // IMPORTANT: Only allow manual control when IndexingSystem is not managing servos

            boolean indexingSystemControlling = indexingSystem.isUptakeServoPrePositioned() ||
                                              indexingSystem.isOperationInProgress() ||
                                              indexingSystem.getCurrentState() == IndexingSystem.SystemState.FIRING ||
                                              indexingSystem.getCurrentState() == IndexingSystem.SystemState.PUSHING;

            if (indexingSystemControlling) {
                // IndexingSystem is controlling servos - do not interfere
                // This prevents manual control from overriding pre-positioning, pushing, or firing
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
     * Update performance metrics
     */
    private void updatePerformanceMetrics() {
        long loopTime = System.currentTimeMillis() - loopStartTime;
        maxLoopTime = Math.max(maxLoopTime, loopTime);
        totalLoops++;
    }
}

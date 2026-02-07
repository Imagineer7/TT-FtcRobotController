package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.AutoGyroTurret;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IntelMechanumDrive;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.localization.Localization;
import org.firstinspires.ftc.teamcode.util.aurora.v3.ArtifactIdentity;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IndexingSystemV3;
import org.firstinspires.ftc.teamcode.util.aurora.v3.SlotLedger;
import org.firstinspires.ftc.teamcode.util.debug.Dbg;
import org.firstinspires.ftc.teamcode.util.debug.LogGroup;
import org.firstinspires.ftc.teamcode.util.debug.LogLevel;

/**
 * IndexingSystemV3BasicTest - Basic functionality test for IndexingSystemV3
 * 
 * This OpMode tests core functionality of the v3 indexing system:
 * - Collection from FRONT and BACK intakes (with real sensors)
 * - Manual artifact injection (for testing without sensors)
 * - Transfer operations (FRONT/BACK → CENTER)
 * - Firing operations (CENTER → shooter)
 * - Ejection operations (remove artifacts)
 * - State machine transitions
 * - Hunt mode (auto-collection)
 * 
 * Controls:
 * 
 * GAMEPAD 1:
 *   DRIVE:
 *   RIGHT STICK Y - Forward/backward movement
 *   RIGHT STICK X - Strafe left/right
 *   LEFT STICK X  - Rotation
 *
 *   DPAD UP    - Fine drive forward (slow, precise)
 *   DPAD DOWN  - Fine drive backward (slow, precise)
 *   DPAD LEFT  - Fine strafe left (slow, precise)
 *   DPAD RIGHT - Fine strafe right (slow, precise)
 *
 *   TURRET (Manual Auto-Gyro Only):
 *   A - Set turret to robot heading (point forward)
 *   B - Enable/disable auto-gyro mode (toggle)
 *
 *   X - Collect FRONT (with sensors)
 *   Y - Collect BACK (with sensors)
 *
 *   RIGHT BUMPER   - Toggle hunt mode
 *   LEFT BUMPER    - Toggle fast collect mode (skip color detection)
 *   RIGHT TRIGGER  - Transfer FRONT → CENTER
 *   LEFT TRIGGER   - Transfer BACK → CENTER
 *   GUIDE          - Hold to eject ALL (full system eject operation)
 *
 * GAMEPAD 2:
 *   DPAD UP   - Hold to eject FRONT intake (runs backward, clears ledger)
 *   DPAD DOWN - Hold to eject BACK intake (runs backward, clears ledger)
 *   DPAD LEFT - Manually add UNKNOWN artifact to CENTER (if empty)
 *   DPAD RIGHT - Manually remove artifact from CENTER (if occupied)
 *
 *   A - Hold to fire SHORT range (2000 RPM)
 *   B - Hold to fire MID range (2300 RPM)
 *   Y - Hold to fire LONG range (2800 RPM)
 *
 * Test Sequence:
 * 1. Manual injection test
 *    - Press DPAD UP → should add Purple to FRONT
 *    - Press DPAD RIGHT → should add Green to BACK
 *    - Verify count = 2
 * 
 * 2. Transfer test
 *    - Press X → should transfer FRONT (Purple) to CENTER
 *    - Verify FRONT empty, CENTER has Purple
 * 
 * 3. Fire test
 *    - Hold GAMEPAD2 A (SHORT) → should spin up shooter to 2000 RPM
 *    - When ready → should automatically fire artifact from CENTER
 *    - Keep holding → should transfer next artifact and fire again (keep-alive)
 *    - Release GAMEPAD2 A → should stop shooter and cancel
 *    - Try GAMEPAD2 B (MID) and Y (LONG) for different RPM targets
 *    - Verify shooter stops, CENTER empty after firing
 * 
 * 4. Hunt mode test
 *    - Press RIGHT BUMPER → enable hunt mode
 *    - Place artifact in FRONT intake
 *    - Should auto-collect without button press
 * 
 * 5. Ejection test
 *    - Add artifact to BACK
 *    - Press START → should eject from BACK
 *    - Verify BACK empty
 * 
 * @author Copilot (AI Assistant)
 * @version 1.0
 * @since 2026-01-20
 */
@TeleOp(name="TeleOp: Auto Collect V3", group="Competition")
public class IndexingSystemV3BasicTest extends LinearOpMode {
    
    // Hardware & config
    private AuroraHardwareConfig hardware;
    private IndexingConfig indexingConfig;
    private ShooterConfig shooterConfig;
    
    // Subsystems
    private IndexingSystemV3 indexing;
    private IntelMechanumDrive drive;
    private AutoGyroTurret autoGyroTurret;
    private Localization localization;
    // Note: Shooter is managed internally by IndexingSystemV3 - OpModes should NOT access it directly
    
    // Button state tracking (gamepad1)
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastBack, lastStart;
    private boolean lastGuide;  // For telemetry page navigation
    
    // Button state tracking (gamepad2) - for edge detection
    private boolean lastGP2DpadLeft, lastGP2DpadRight;

    // Firing state tracking
    private boolean isFiring = false;  // True when firing sequence active
    private boolean lastReadyToFire = false;  // Track when ready-to-fire state changes (edge detection)
    private ShooterConfig.ShooterPreset currentFiringMode = null;  // Track which firing mode is active

    @Override
    public void runOpMode() {
        // Configure debug logging
        Dbg.setGlobalPrefix("V3Test");
        Dbg.setContext("V3BasicTest", "INIT");
        Dbg.setGlobalLevel(LogLevel.DEBUG);
        Dbg.setIncludeLoopCount(true);
        Dbg.resetLoop();

        // Enable critical groups
        Dbg.setGroupLevel(LogGroup.TEST, LogLevel.DEBUG);
        Dbg.setGroupLevel(LogGroup.INDEXING, LogLevel.DEBUG);
        Dbg.setGroupLevel(LogGroup.FIRING, LogLevel.DEBUG);
        Dbg.setGroupLevel(LogGroup.SHOOTER, LogLevel.DEBUG);

        Dbg.i(LogGroup.TEST, "========================================");
        Dbg.i(LogGroup.TEST, "  INDEXING SYSTEM V3 - BASIC TEST");
        Dbg.i(LogGroup.TEST, "========================================");

        telemetry.addLine("======================================");
        telemetry.addLine("   INDEXING SYSTEM V3 - BASIC TEST    ");
        telemetry.addLine("======================================");
        telemetry.addLine();
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        Dbg.i(LogGroup.TEST, "Initializing hardware...");

        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        Dbg.d(LogGroup.TEST, "Hardware initialized: %s", hardware.isIndexingSystemInitialized());

        // Check hardware status
        if (!hardware.isIndexingSystemInitialized()) {
            Dbg.e(LogGroup.TEST, "Indexing hardware not initialized!");
            Dbg.e(LogGroup.TEST, hardware.getInitializationSummary());
            telemetry.addLine("❌ ERROR: Indexing hardware not initialized");
            telemetry.addLine(hardware.getInitializationSummary());
            telemetry.update();
            while (!isStopRequested()) {
                sleep(100);
            }
            return;
        }
        
        Dbg.i(LogGroup.TEST, "Hardware initialization successful");

        // Initialize configs
        indexingConfig = new IndexingConfig();
        shooterConfig = new ShooterConfig();
        
        Dbg.d(LogGroup.TEST, "Creating subsystems...");

        // Initialize subsystems
        // Note: Shooter is created and managed internally by IndexingSystemV3
        Shooter shooter = new Shooter(hardware, shooterConfig, telemetry);
        indexing = new IndexingSystemV3(hardware, indexingConfig, shooter, telemetry);
        
        // Initialize drive system
        drive = new IntelMechanumDrive(hardware, gamepad1);

        // Initialize localization for robot heading
        telemetry.addLine("Initializing localization...");
        telemetry.update();

        localization = new Localization(hardwareMap);

        // Check localization status
        if (!localization.isOdometryInitialized()) {
            telemetry.addData("⚠ Odometry", "Not available - turret heading may be inaccurate");
        } else {
            telemetry.addData("✅ Odometry", "Initialized successfully");
        }
        telemetry.update();

        localization.resetPosition();

        // Initialize auto-gyro turret
        telemetry.addLine("Initializing turret...");
        telemetry.update();

        autoGyroTurret = new AutoGyroTurret(hardwareMap, telemetry);

        // Check if turret servo is available
        try {
            if (hardware.getTurretServo() != null) {
                telemetry.addData("✅ Turret Servo", "Found in hardware map");
                Dbg.i(LogGroup.TEST, "Turret servo initialized successfully");
            } else {
                telemetry.addData("❌ Turret Servo", "Not found in hardware map");
                Dbg.e(LogGroup.TEST, "Turret servo not found!");
            }
        } catch (Exception e) {
            telemetry.addData("❌ Turret Servo", "Error: " + e.getMessage());
            Dbg.e(LogGroup.TEST, "Turret servo error: %s", e.getMessage());
        }
        telemetry.update();

        // Enable auto-gyro by default and reset to forward
        double initialHeading = 0.0;
        if (localization.isOdometryInitialized()) {
            initialHeading = localization.getHeading(AngleUnit.DEGREES);
        }

        telemetry.addData("Initial Heading", String.format("%.1f°", initialHeading));
        telemetry.update();

        autoGyroTurret.resetToForward(initialHeading);
        autoGyroTurret.enable();

        telemetry.addData("✅ Turret", "Initialized and enabled");
        telemetry.update();

        Dbg.i(LogGroup.TEST, "Subsystems created successfully");

        // Enable systems
        shooter.enable();  // Enable shooter (managed internally by indexing system)
        indexing.enable();
        
        Dbg.i(LogGroup.TEST, "Systems enabled");

        telemetry.addLine("✓ Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Ready to test basic operations");
        telemetry.addLine();
        telemetry.addLine("TURRET CONTROLS (Gamepad 1):");
        telemetry.addLine("  A: Set turret to robot heading");
        telemetry.addLine("  B: Toggle auto-gyro on/off");
        telemetry.addLine();
        telemetry.addLine("COLLECTION (Gamepad 1):");
        telemetry.addLine("  X: Collect FRONT");
        telemetry.addLine("  Y: Collect BACK");
        telemetry.addLine("  Right Trigger: Transfer FRONT → CENTER");
        telemetry.addLine("  Left Trigger: Transfer BACK → CENTER");
        telemetry.addLine();
        telemetry.addLine("(Press START on Driver Station to begin)");
        telemetry.update();
        
        Dbg.i(LogGroup.TEST, "Waiting for START...");

        waitForStart();
        
        // Initialize ALL button states to false after waitForStart
        // This ensures first button press will be detected as an edge
        // (if we used current state, buttons held during init wouldn't trigger)
        lastDpadUp = false;
        lastDpadDown = false;
        lastDpadLeft = false;
        lastDpadRight = false;
        lastA = false;
        lastB = false;
        lastX = false;
        lastY = false;
        lastLeftBumper = false;
        lastRightBumper = false;
        lastBack = false;
        lastStart = false;
        lastGuide = false;

        // Gamepad 2 button states (for center artifact management)
        lastGP2DpadLeft = false;
        lastGP2DpadRight = false;

        Dbg.setContext("V3BasicTest", "RUN");
        Dbg.i(LogGroup.TEST, "OpMode started - entering main loop");
        Dbg.i(LogGroup.TEST, "Button states initialized to false for edge detection");

        // Main loop
        while (opModeIsActive()) {
            Dbg.incrementLoop();

            // Update localization for robot heading
            if (localization != null) {
                localization.update();
            }

            // Get current robot heading for turret control
            double robotHeading = 0.0;
            if (localization != null && localization.isOdometryInitialized()) {
                robotHeading = localization.getHeading(AngleUnit.DEGREES);
            }

            // ═══════════════════════════════════════════════════════════════
            // TURRET CONTROLS (GAMEPAD 1 - A and B buttons)
            // ═══════════════════════════════════════════════════════════════

            // A Button - Set turret to robot heading (point forward)
            // NO EDGE DETECTION - runs continuously when held
            if (gamepad1.a) {
                double targetHeading = robotHeading + 180;
                autoGyroTurret.setFieldRelativeHeading(targetHeading, robotHeading);
                autoGyroTurret.enable();

                Dbg.everyMs(LogGroup.TEST, LogLevel.DEBUG, "turret_set_a", 500,
                            "Button A held - turret target: %.1f° (robot: %.1f°)", targetHeading, robotHeading);
            }

            // B Button - Toggle auto-gyro mode on/off
            // EDGE DETECTION - only toggle on press, not hold
            boolean bPressed = gamepad1.b && !lastB;

            // Debug logging for button B
            Dbg.d(LogGroup.TEST, "Button B check - current: %b, last: %b, edge: %b",
                  gamepad1.b, lastB, bPressed);

            if (bPressed) {
                boolean oldState = autoGyroTurret.isEnabled();
                Dbg.i(LogGroup.TEST, "Button B EDGE DETECTED - Toggling turret auto-gyro");
                Dbg.d(LogGroup.TEST, "  Old state: %s", oldState ? "ENABLED" : "DISABLED");

                boolean newState = autoGyroTurret.toggle();

                Dbg.i(LogGroup.TEST, "Turret toggled - new state: %s", newState ? "ENABLED" : "DISABLED");
                if (newState) {
                    telemetry.addLine("✅ Turret: Auto-gyro ENABLED");
                } else {
                    telemetry.addLine("❌ Turret: Auto-gyro DISABLED");
                }
            }

            // Update turret to maintain field-relative heading (if enabled)
            if (autoGyroTurret != null) {
                autoGyroTurret.update(robotHeading);
            }

            // ═══════════════════════════════════════════════════════════════
            // SUBSYSTEM UPDATES
            // ═══════════════════════════════════════════════════════════════

            // Update subsystems
            // Update manual mode based on gamepad2 inputs
            // Note: dpad_up and dpad_down are used for intake ejection
            // Note: dpad_left and dpad_right are used for center artifact management
            // Manual mode is no longer tied to dpad - disabled by default
            boolean manualActive = false;
            indexing.setManualModeActive(manualActive);

            // Update watchdog trigger state with fire buttons (gamepad2 A, B, or Y)
            boolean fireButtonHeld = gamepad2.a || gamepad2.b || gamepad2.y;
            indexing.setWatchdogTriggerState(fireButtonHeld);

            // Update system
            // ⚠️ CRITICAL: indexing.update() calls firingHelper.update() internally,
            // which then calls shooter.update(). DO NOT call shooter.update() here!
            indexing.update();
            // shooter.update();  // ❌ REMOVED - would cause duplicate call and pulsing
            
            // ═══════════════════════════════════════════════════════════════
            // DRIVE CONTROLS (GAMEPAD 1)
            // ═══════════════════════════════════════════════════════════════

            // Robot-centric drive (relative to robot's orientation)
            double forward = -gamepad1.right_stick_y;  // Note: Y-axis is inverted
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;

            // Fine drive movements with DPAD (slower, precise control)
            final double FINE_DRIVE_POWER = 0.35;  // 25% power for fine movements
            if (gamepad1.dpad_up) {
                forward = FINE_DRIVE_POWER;
            } else if (gamepad1.dpad_down) {
                forward = -FINE_DRIVE_POWER;
            }
            if (gamepad1.dpad_left) {
                strafe = -FINE_DRIVE_POWER;
            } else if (gamepad1.dpad_right) {
                strafe = FINE_DRIVE_POWER;
            }

            drive.setMechanumPowers(forward, strafe, rotate);

            // Handle collection
            handleCollection();
            
            // Handle transfer
            handleTransfer();
            
            // Handle fire
            handleFire();
            
            // Handle ejection
            handleEjection();
            
            // Handle manual center artifact management
            handleCenterArtifactManagement();

            // Handle hunt mode toggle
            handleHuntMode();
            
            // Handle fast collect mode toggle
            handleFastCollectMode();

            // Handle telemetry page navigation
            handleTelemetryNavigation();
            
            // Add prominent mode status at top
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("🔍 HUNT MODE", indexing.isHuntEnabled() ? "✅ ON" : "❌ OFF");
            telemetry.addData("⚡ SKIP COLOR", indexing.isSkipColorDetection() ? "✅ ON (Fast)" : "❌ OFF (Full)");
            if (autoGyroTurret != null && autoGyroTurret.isEnabled()) {
                telemetry.addData("🎯 TURRET", String.format("✅ ON (%.0f°)", autoGyroTurret.getFieldRelativeHeading()));
            } else {
                telemetry.addData("🎯 TURRET", "❌ OFF");
            }
            telemetry.addLine("═══════════════════════════════");

            // ═══════════════════════════════════════════════════════════════
            // TURRET DEBUG TELEMETRY
            // ═══════════════════════════════════════════════════════════════
            telemetry.addLine();
            telemetry.addLine("─── TURRET DEBUG ───");

            // Localization status
            if (localization != null) {
                telemetry.addData("Localization", "Initialized");
                telemetry.addData("  Odometry", localization.isOdometryInitialized() ? "✅ Available" : "❌ Not Available");
                if (localization.isOdometryInitialized()) {
                    telemetry.addData("  Robot Heading", String.format("%.1f°", robotHeading));
                }
            } else {
                telemetry.addData("Localization", "❌ NULL");
            }

            // Turret status
            if (autoGyroTurret != null) {
                telemetry.addData("Turret Object", "✅ Initialized");
                telemetry.addData("  Enabled", autoGyroTurret.isEnabled() ? "✅ YES" : "❌ NO");
                telemetry.addData("  Target Heading", String.format("%.1f°", autoGyroTurret.getFieldRelativeHeading()));
                if (localization != null && localization.isOdometryInitialized()) {
                    telemetry.addData("  Current Heading", String.format("%.1f°", autoGyroTurret.getCurrentFieldHeading(robotHeading)));
                    telemetry.addData("  Error", String.format("%.1f°", autoGyroTurret.getHeadingError(robotHeading)));
                    telemetry.addData("  At Target", autoGyroTurret.isAtTarget(robotHeading) ? "✅ YES" : "❌ NO");
                }
                telemetry.addData("  Busy", autoGyroTurret.isBusy() ? "⚠️ YES" : "✅ NO");
            } else {
                telemetry.addData("Turret Object", "❌ NULL");
            }

            // Button states with edge detection
            boolean bCurrentlyPressed = gamepad1.b;
            boolean bEdgeDetected = bCurrentlyPressed && !lastB;

            telemetry.addData("GP1 A (Set Heading)", gamepad1.a ? "PRESSED (continuous)" : "released");

            telemetry.addData("GP1 B (Toggle)", bCurrentlyPressed ? "PRESSED" : "released");
            telemetry.addData("  Last B", lastB ? "true" : "false");
            telemetry.addData("  Edge Detected", bEdgeDetected ? "✅ YES" : "no");
            telemetry.addData("  gamepad1.b value", gamepad1.b);
            telemetry.addData("  lastB value", lastB);

            telemetry.addLine("───────────────────────────────");
            telemetry.addLine();

            // Display telemetry (from IndexingSystemV3)
            indexing.addTelemetry();
            
            // Update button states
            updateButtonStates();
            
            telemetry.update();
        }
    }
    
    // Manual injection removed - DPAD now used for fine drive movements
    // Use sensors for artifact collection instead

    private void handleCollection() {
        // X Button - Collect FRONT
        if (gamepad1.x && !lastX) {
            Dbg.i(LogGroup.TEST, "Collection request: FRONT");
            boolean success = indexing.requestCollect(SlotLedger.Slot.FRONT);
            if (success) {
                Dbg.i(LogGroup.TEST, "Collection started: FRONT");
            } else {
                Dbg.w(LogGroup.TEST, "Collection rejected: FRONT (slot occupied or busy)");
            }
            telemetry.addLine(success ? "→ Collecting FRONT" : "❌ Cannot collect FRONT");
        }

        // Y Button - Collect BACK
        if (gamepad1.y && !lastY) {
            Dbg.i(LogGroup.TEST, "Collection request: BACK");
            boolean success = indexing.requestCollect(SlotLedger.Slot.BACK);
            if (success) {
                Dbg.i(LogGroup.TEST, "Collection started: BACK");
            } else {
                Dbg.w(LogGroup.TEST, "Collection rejected: BACK (slot occupied or busy)");
            }
            telemetry.addLine(success ? "→ Collecting BACK" : "❌ Cannot collect BACK");
        }
    }
    
    private void handleTransfer() {
        // Right Trigger - Transfer FRONT → CENTER
        if (gamepad1.right_trigger > 0.5) {
            Dbg.i(LogGroup.TEST, "Transfer request: FRONT → CENTER");
            boolean success = indexing.requestTransfer(SlotLedger.Slot.FRONT);
            if (success) {
                Dbg.i(LogGroup.TEST, "Transfer started: FRONT → CENTER");
            } else {
                Dbg.w(LogGroup.TEST, "Transfer rejected: FRONT (slot empty, center full, or busy)");
            }
            telemetry.addLine(success ? "→ Transfer FRONT → CENTER" : "❌ Cannot transfer FRONT");
        }

        // Left Trigger - Transfer BACK → CENTER
        if (gamepad1.left_trigger > 0.5) {
            Dbg.i(LogGroup.TEST, "Transfer request: BACK → CENTER");
            boolean success = indexing.requestTransfer(SlotLedger.Slot.BACK);
            if (success) {
                Dbg.i(LogGroup.TEST, "Transfer started: BACK → CENTER");
            } else {
                Dbg.w(LogGroup.TEST, "Transfer rejected: BACK (slot empty, center full, or busy)");
            }
            telemetry.addLine(success ? "→ Transfer BACK → CENTER" : "❌ Cannot transfer BACK");
        }
    }
    
    private void handleFire() {
        // Hold-to-fire behavior using FiringHelper's built-in keep-alive mode
        // FiringHelper automatically handles spinup, firing, and keeping shooter alive

        // Detect which fire button is held and determine preset
        ShooterConfig.ShooterPreset selectedPreset = null;
        boolean fireButtonHeld = false;

        if (gamepad2.a) {
            selectedPreset = ShooterConfig.ShooterPreset.SHORT_RANGE;
            fireButtonHeld = true;
        } else if (gamepad2.b) {
            selectedPreset = ShooterConfig.ShooterPreset.MID_RANGE;
            fireButtonHeld = true;
        } else if (gamepad2.y) {
            selectedPreset = ShooterConfig.ShooterPreset.LONG_RANGE;
            fireButtonHeld = true;
        }

        // Note: Watchdog trigger state is set at top of loop with all fire buttons

        if (fireButtonHeld) {
            // Button is being held
            if (!isFiring) {
                // Just pressed - start firing sequence
                currentFiringMode = selectedPreset;  // Store the mode
                Dbg.i(LogGroup.TEST, "Fire button pressed - initiating firing sequence");
                // FiringHelper will handle spinup automatically when we call startFiring()
                if (!indexing.getLedger().isCenterOccupied()) {
                    Dbg.w(LogGroup.TEST, "Cannot fire: CENTER empty");
                    telemetry.addLine("❌ Cannot fire: CENTER empty");
                } else {
                    Dbg.i(LogGroup.TEST, "Starting firing with %s preset (%.0f RPM)",
                          currentFiringMode.getName(), currentFiringMode.getTargetRPM());
                    telemetry.addLine("🔥 Starting firing sequence (" + currentFiringMode.getName() + ")...");
                    // startFiring() handles spinup automatically and fires the first shot
                    // Keep-alive mode enabled - shooter stays spinning for rapid follow-up shots
                    boolean started = indexing.requestFire(currentFiringMode.getTargetRPM());
                    Dbg.d(LogGroup.TEST, "requestFire returned: %s", started);
                    if (started) {
                        isFiring = true;
                        Dbg.i(LogGroup.TEST, "Firing sequence started successfully");
                    } else {
                        Dbg.e(LogGroup.TEST, "Failed to start firing sequence");
                        telemetry.addLine("❌ Could not start firing");
                    }
                }
            } else {
                // Button still held - check if mode changed
                if (selectedPreset != currentFiringMode) {
                    // Mode changed while firing - cancel and restart with new mode
                    Dbg.i(LogGroup.TEST, "Fire mode changed from %s to %s - restarting",
                          currentFiringMode.getName(), selectedPreset.getName());
                    telemetry.addLine("🔄 Switching to " + selectedPreset.getName() + "...");
                    indexing.cancelBurstFiring();
                    isFiring = false;
                    lastReadyToFire = false;
                    currentFiringMode = null;
                    return;  // Will restart on next loop
                }

                // Button still held - check if ready for next shot and fire it
                // CRITICAL: Must check BOTH shooter ready AND no operations running
                // Otherwise we may fire before transfer completes and artifact physically loads
                
                // Check current state
                boolean readyToFire = indexing.isReadyForNextShot() && 
                                     !indexing.isOperationRunning() && 
                                     indexing.getLedger().isCenterOccupied();
                
                // EDGE DETECTION: Only fire when state changes from false → true
                // This prevents repeated firing of the same artifact
                if (readyToFire && !lastReadyToFire) {
                    // State just changed to ready - fire now
                    Dbg.i(LogGroup.TEST, "Ready for next shot - firing now");
                    Dbg.d(LogGroup.TEST, "Shooter RPM: %.0f / %.0f",
                          indexing.getShooterCurrentRPM(), indexing.getShooterTargetRPM());
                    Dbg.d(LogGroup.TEST, "Center occupied: %b, Operation running: %b",
                          indexing.getLedger().isCenterOccupied(), indexing.isOperationRunning());
                    telemetry.addLine("🔥 Firing next shot...");
                    // Fire the next shot (FiringHelper keeps shooter spinning)
                    boolean fired = indexing.fireNextShot();
                    Dbg.d(LogGroup.TEST, "fireNextShot returned: %b", fired);
                } else if (readyToFire) {
                    // Already fired this artifact, waiting for next
                    Dbg.everyMs(LogGroup.TEST, LogLevel.DEBUG, "wait_artifact", 1000,
                                "Shot fired, waiting for next artifact...");
                    telemetry.addLine("⏳ Shot fired, waiting for next artifact...");
                } else if (indexing.isOperationRunning()) {
                    // Transfer or other operation in progress
                    Dbg.everyMs(LogGroup.TEST, LogLevel.DEBUG, "transfer_progress", 500,
                                "Transfer in progress...");
                    telemetry.addLine("⏳ Transfer in progress...");
                } else if (!indexing.getLedger().isCenterOccupied()) {
                    // No artifact in center
                    Dbg.everyMs(LogGroup.TEST, LogLevel.DEBUG, "wait_transfer", 1000,
                                "Waiting for next artifact transfer...");
                    telemetry.addLine("⏳ Waiting for next artifact transfer...");
                } else {
                    // Still processing previous shot or spinning up
                    Dbg.everyMs(LogGroup.TEST, LogLevel.DEBUG, "spinup", 500,
                                "Spinning up: %.0f / %.0f RPM",
                                indexing.getShooterCurrentRPM(), indexing.getShooterTargetRPM());
                    telemetry.addLine("⏳ Processing... " +
                        String.format("%.0f", indexing.getShooterCurrentRPM()) + " / " + 
                        String.format("%.0f", indexing.getShooterTargetRPM()) + " RPM");
                }
                
                // Update edge detection state
                lastReadyToFire = readyToFire;
            }
        } else {
            // Button released
            if (isFiring) {
                // Was firing, now stop
                Dbg.i(LogGroup.TEST, "Fire button released - canceling firing sequence");
                telemetry.addLine("🛑 Button released - stopping shooter");
                // Cancel firing through FiringHelper (stops shooter, completes any transfers)
                indexing.cancelBurstFiring();
                isFiring = false;
                lastReadyToFire = false;  // Reset edge detection
                currentFiringMode = null;  // Clear firing mode
                Dbg.i(LogGroup.TEST, "Firing sequence canceled");
            }
        }
    }
    
    private void handleEjection() {
        // Hold GAMEPAD2 DPAD UP to eject FRONT intake (runs backward while held)
        // Hold GAMEPAD1 GUIDE to eject BOTH intakes (full system eject)
        boolean ejectFront = gamepad2.dpad_up || gamepad1.guide;
        indexing.setIntakeEjecting(SlotLedger.Slot.FRONT, ejectFront);
        if (gamepad2.dpad_up) {
            telemetry.addData("⏏️ Eject", "FRONT (hold to continue)");
        }

        // Hold GAMEPAD2 DPAD DOWN to eject BACK intake (runs backward while held)
        // Hold GAMEPAD1 GUIDE to eject BOTH intakes (full system eject)
        boolean ejectBack = gamepad2.dpad_down || gamepad1.guide;
        indexing.setIntakeEjecting(SlotLedger.Slot.BACK, ejectBack);
        if (gamepad2.dpad_down) {
            telemetry.addData("⏏️ Eject", "BACK (hold to continue)");
        }

        // Show combined eject status for GUIDE button
        if (gamepad1.guide) {
            telemetry.addData("⏏️ Eject", "ALL intakes (hold to continue)");
        }
    }
    
    private void handleCenterArtifactManagement() {
        // GAMEPAD2 DPAD LEFT - Manually add UNKNOWN artifact to CENTER (if empty)
        // Uses edge detection to prevent repeated additions while held
        if (gamepad2.dpad_left && !lastGP2DpadLeft) {
            Dbg.i(LogGroup.TEST, "Manual add artifact to CENTER requested");
            boolean success = indexing.addManualArtifactToCenter();
            if (success) {
                Dbg.i(LogGroup.TEST, "Manual artifact added to CENTER successfully");
                telemetry.addLine("✅ Manual: Added UNKNOWN artifact to CENTER");
            } else {
                Dbg.w(LogGroup.TEST, "Manual add to CENTER rejected (occupied or busy)");
                telemetry.addLine("❌ Manual: Cannot add to CENTER (occupied or busy)");
            }
        }

        // GAMEPAD2 DPAD RIGHT - Manually remove artifact from CENTER (if occupied)
        // Uses edge detection to prevent repeated removals while held
        if (gamepad2.dpad_right && !lastGP2DpadRight) {
            Dbg.i(LogGroup.TEST, "Manual remove artifact from CENTER requested");
            boolean success = indexing.removeManualArtifactFromCenter();
            if (success) {
                Dbg.i(LogGroup.TEST, "Manual artifact removed from CENTER successfully");
                telemetry.addLine("✅ Manual: Removed artifact from CENTER");
            } else {
                Dbg.w(LogGroup.TEST, "Manual remove from CENTER rejected (empty or busy)");
                telemetry.addLine("❌ Manual: Cannot remove from CENTER (empty or busy)");
            }
        }
    }

    private void handleHuntMode() {
        if (gamepad1.right_bumper && !lastRightBumper) {
            boolean newState = indexing.toggleHuntEnabled();
            Dbg.i(LogGroup.TEST, "Hunt mode toggled: %s", newState ? "ENABLED" : "DISABLED");
            if (newState) {
                telemetry.addLine("🔍 Hunt mode ENABLED");
            } else {
                telemetry.addLine("💤 Hunt mode DISABLED");
            }
        }
    }
    
    private void handleFastCollectMode() {
        if (gamepad1.left_bumper && !lastLeftBumper) {
            boolean newState = indexing.toggleSkipColorDetection();
            Dbg.i(LogGroup.TEST, "Fast collect mode toggled: %s", newState ? "ENABLED" : "DISABLED");
            if (newState) {
                telemetry.addLine("⚡ Fast collect mode ENABLED (skip color detection)");
            } else {
                telemetry.addLine("🎨 Fast collect mode DISABLED (detect color)");
            }
        }
    }


    private void handleTelemetryNavigation() {
        // Use GAMEPAD2 Guide button (Xbox logo / PS button) to cycle telemetry pages
        // (GAMEPAD1 Guide is used for full system eject)
        if (gamepad2.guide && !lastGuide) {
            indexing.nextTelemetryPage();
            Dbg.d(LogGroup.TEST, "Telemetry page switched to: %d", indexing.getTelemetryPage());
            telemetry.addLine("→ Switched to page " + indexing.getTelemetryPage());
        }
    }
    
    private void updateButtonStates() {
        // Gamepad 1 button states
        lastGuide = gamepad2.guide;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
        lastBack = gamepad1.back;
        lastStart = gamepad1.start;

        // Gamepad 2 button states (for center artifact management)
        lastGP2DpadLeft = gamepad2.dpad_left;
        lastGP2DpadRight = gamepad2.dpad_right;
    }
}

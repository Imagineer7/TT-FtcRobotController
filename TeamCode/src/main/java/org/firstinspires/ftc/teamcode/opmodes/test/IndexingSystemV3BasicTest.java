package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
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
 *   LEFT STICK Y  - Control intake rollers manually (testing)
 *   RIGHT STICK Y - Control uptake manually (testing)
 *   
 *   DPAD UP    - Add PURPLE to FRONT (manual injection)
 *   DPAD DOWN  - Add GREEN to FRONT (manual injection)
 *   DPAD LEFT  - Add PURPLE to BACK (manual injection)
 *   DPAD RIGHT - Add GREEN to BACK (manual injection)
 *   
 *   A - Collect FRONT (with sensors)
 *   B - Collect BACK (with sensors)
 *   X - Transfer FRONT → CENTER
 *   Y - Transfer BACK → CENTER
 *   
 *   LEFT BUMPER  - Hold to fire (spins up shooter, fires when ready, keep-alive mode)
 *   RIGHT BUMPER - Toggle hunt mode
 *   
 *   BACK - Eject FRONT
 *   START - Eject BACK
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
 *    - Hold LEFT BUMPER → should spin up shooter
 *    - When ready → should automatically fire Purple from CENTER
 *    - Keep holding → should transfer next artifact and fire again (keep-alive)
 *    - Release LEFT BUMPER → should stop shooter and cancel
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
@TeleOp(name="V3 Basic Test", group="V3 Testing")
public class IndexingSystemV3BasicTest extends LinearOpMode {
    
    // Hardware & config
    private AuroraHardwareConfig hardware;
    private IndexingConfig indexingConfig;
    private ShooterConfig shooterConfig;
    
    // Subsystems
    private IndexingSystemV3 indexing;
    // Note: Shooter is managed internally by IndexingSystemV3 - OpModes should NOT access it directly
    
    // Button state tracking
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastBack, lastStart;
    private boolean lastGuide;  // For telemetry page navigation
    
    // Firing state tracking
    private boolean isFiring = false;  // True when firing sequence active
    private boolean lastReadyToFire = false;  // Track when ready-to-fire state changes (edge detection)
    
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
        
        Dbg.i(LogGroup.TEST, "Subsystems created successfully");

        // Enable systems
        shooter.enable();  // Enable shooter (managed internally by indexing system)
        indexing.enable();
        
        Dbg.i(LogGroup.TEST, "Systems enabled");

        telemetry.addLine("✓ Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Ready to test basic operations");
        telemetry.addLine("See controls on driver station");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();
        
        Dbg.i(LogGroup.TEST, "Waiting for START...");

        waitForStart();
        
        Dbg.setContext("V3BasicTest", "RUN");
        Dbg.i(LogGroup.TEST, "OpMode started - entering main loop");

        // Main loop
        while (opModeIsActive()) {
            Dbg.incrementLoop();
            // Update subsystems
            // Update manual mode based on gamepad2 inputs
            boolean manualActive = gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down;
            indexing.setManualModeActive(manualActive);

            // Update watchdog trigger state
            indexing.setWatchdogTriggerState(gamepad1.right_trigger > 0.1);

            // Update system
            // ⚠️ CRITICAL: indexing.update() calls firingHelper.update() internally,
            // which then calls shooter.update(). DO NOT call shooter.update() here!
            indexing.update();
            // shooter.update();  // ❌ REMOVED - would cause duplicate call and pulsing
            
            // Handle manual injection
            handleManualInjection();
            
            // Handle collection
            handleCollection();
            
            // Handle transfer
            handleTransfer();
            
            // Handle fire
            handleFire();
            
            // Handle ejection
            handleEjection();
            
            // Handle hunt mode toggle
            handleHuntMode();
            
            // Handle telemetry page navigation
            handleTelemetryNavigation();
            
            // Display telemetry (from IndexingSystemV3)
            indexing.addTelemetry();
            
            // Update button states
            updateButtonStates();
            
            telemetry.update();
        }
    }
    
    private void handleManualInjection() {
        if (gamepad1.dpad_up && !lastDpadUp) {
            Dbg.i(LogGroup.TEST, "Manual injection: PURPLE → FRONT");
            indexing.addManualArtifact(SlotLedger.Slot.FRONT, ArtifactIdentity.ColorClass.PURPLE);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            Dbg.i(LogGroup.TEST, "Manual injection: GREEN → FRONT");
            indexing.addManualArtifact(SlotLedger.Slot.FRONT, ArtifactIdentity.ColorClass.GREEN);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            Dbg.i(LogGroup.TEST, "Manual injection: PURPLE → BACK");
            indexing.addManualArtifact(SlotLedger.Slot.BACK, ArtifactIdentity.ColorClass.PURPLE);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            Dbg.i(LogGroup.TEST, "Manual injection: GREEN → BACK");
            indexing.addManualArtifact(SlotLedger.Slot.BACK, ArtifactIdentity.ColorClass.GREEN);
        }
    }
    
    private void handleCollection() {
        if (gamepad1.a && !lastA) {
            Dbg.i(LogGroup.TEST, "Collection request: FRONT");
            boolean success = indexing.requestCollect(SlotLedger.Slot.FRONT);
            if (success) {
                Dbg.i(LogGroup.TEST, "Collection started: FRONT");
            } else {
                Dbg.w(LogGroup.TEST, "Collection rejected: FRONT (slot occupied or busy)");
            }
            telemetry.addLine(success ? "→ Collecting FRONT" : "❌ Cannot collect FRONT");
        }
        if (gamepad1.b && !lastB) {
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
        if (gamepad1.x && !lastX) {
            Dbg.i(LogGroup.TEST, "Transfer request: FRONT → CENTER");
            boolean success = indexing.requestTransfer(SlotLedger.Slot.FRONT);
            if (success) {
                Dbg.i(LogGroup.TEST, "Transfer started: FRONT → CENTER");
            } else {
                Dbg.w(LogGroup.TEST, "Transfer rejected: FRONT (slot empty, center full, or busy)");
            }
            telemetry.addLine(success ? "→ Transfer FRONT → CENTER" : "❌ Cannot transfer FRONT");
        }
        if (gamepad1.y && !lastY) {
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
        boolean fireButtonHeld = gamepad1.left_bumper;
        
        // Update button state for FiringHelper to track
        indexing.setFiringButtonHeld(fireButtonHeld);
        
        if (fireButtonHeld) {
            // Button is being held
            if (!isFiring) {
                // Just pressed - start firing sequence
                Dbg.i(LogGroup.TEST, "Fire button pressed - initiating firing sequence");
                // FiringHelper will handle spinup automatically when we call startFiring()
                if (!indexing.getLedger().isCenterOccupied()) {
                    Dbg.w(LogGroup.TEST, "Cannot fire: CENTER empty");
                    telemetry.addLine("❌ Cannot fire: CENTER empty");
                } else {
                    Dbg.i(LogGroup.TEST, "Starting firing with SHORT_RANGE preset (%.0f RPM)",
                          ShooterConfig.ShooterPreset.SHORT_RANGE.getTargetRPM());
                    telemetry.addLine("🔥 Starting firing sequence...");
                    // startFiring() handles spinup automatically and fires the first shot
                    // Keep-alive mode enabled - shooter stays spinning for rapid follow-up shots
                    boolean started = indexing.requestFire(ShooterConfig.ShooterPreset.SHORT_RANGE.getTargetRPM());
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
                // Button still held - check if ready for next shot and fire it
                // CRITICAL: Must check BOTH shooter ready AND no operations running AND center occupied
                // FIX: Also check that we're NOT already feeding (prevents double-fire)
                
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
                Dbg.i(LogGroup.TEST, "Firing sequence canceled");
            }
        }
    }
    
    private void handleEjection() {
        if (gamepad1.back && !lastBack) {
            Dbg.i(LogGroup.TEST, "Ejection request: ALL slots");
            // Note: V3 uses EjectMode.ALL to eject all artifacts
            indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.ALL);
            telemetry.addLine("⏏️ Clearing all slots");
        }
        if (gamepad1.start && !lastStart) {
            Dbg.i(LogGroup.TEST, "Ejection request: CENTER only");
            // Note: V3 uses EjectMode.CENTER to eject center only
            boolean success = indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.CENTER);
            if (success) {
                Dbg.i(LogGroup.TEST, "Ejection started: CENTER");
            } else {
                Dbg.w(LogGroup.TEST, "Ejection rejected: CENTER (empty or busy)");
            }
            telemetry.addLine(success ? "⏏️ Ejecting CENTER" : "❌ Cannot eject CENTER");
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
    
    private void handleTelemetryNavigation() {
        // Use Guide button (Xbox logo / PS button) to cycle telemetry pages
        if (gamepad1.guide && !lastGuide) {
            indexing.nextTelemetryPage();
            Dbg.d(LogGroup.TEST, "Telemetry page switched to: %d", indexing.getTelemetryPage());
            telemetry.addLine("→ Switched to page " + indexing.getTelemetryPage());
        }
    }
    
    private void updateButtonStates() {
        lastGuide = gamepad1.guide;
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
    }
}

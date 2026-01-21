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

/**
 * ManualArtifactInjectionTest - Comprehensive test for manual artifact injection
 * 
 * This OpMode tests the manual artifact injection feature of IndexingSystemV3.
 * It allows operators to inject artifacts with specified colors into FRONT or BACK
 * slots without requiring physical artifacts or sensors.
 * 
 * Controls:
 * 
 * GAMEPAD 1:
 *   DPAD UP    - Add PURPLE artifact to FRONT slot
 *   DPAD DOWN  - Add GREEN artifact to FRONT slot
 *   DPAD LEFT  - Add PURPLE artifact to BACK slot
 *   DPAD RIGHT - Add GREEN artifact to BACK slot
 *   
 *   A - Request transfer FRONT → CENTER
 *   B - Request transfer BACK → CENTER
 *   X - Request swap (rearrange artifacts)
 *   Y - Request fire (if CENTER occupied)
 *   
 *   LEFT BUMPER  - Request eject FRONT
 *   RIGHT BUMPER - Request eject BACK
 *   
 *   BACK - Clear all slots (reset system)
 *   START - Enable/disable system
 * 
 * GAMEPAD 2:
 *   DPAD UP - Spin up shooter to HIGH_BASKET (3200 RPM)
 *   DPAD DOWN - Stop shooter
 * 
 * Test Scenarios:
 * 1. Basic injection - Add single artifacts
 * 2. Fill system - Add 3 artifacts (1 in CENTER, 2 in intakes)
 * 3. Test rejection - Try to add to occupied slot, full system
 * 4. Shot planning - Add PPG pattern, test rearrangement
 * 5. Transfer/swap - Verify operations work with injected artifacts
 * 6. Burst firing - Add 3 artifacts, set motif, fire sequence
 * 
 * Expected behavior:
 * - Manual injection should bypass sensor detection
 * - Artifacts should behave identically to sensor-detected ones
 * - All operations (transfer, swap, fire, eject) should work normally
 * - Shot planning should optimize injected artifacts
 * 
 * @author Copilot (AI Assistant)
 * @version 1.0
 * @since 2026-01-20
 */
@TeleOp(name="Manual Artifact Injection Test", group="V3 Testing")
public class ManualArtifactInjectionTest extends LinearOpMode {
    
    // Hardware & config
    private AuroraHardwareConfig hardware;
    private IndexingConfig indexingConfig;
    private ShooterConfig shooterConfig;
    
    // Subsystems
    private IndexingSystemV3 indexing;
    private Shooter shooter;
    
    // Button state tracking (edge detection)
    private boolean lastDpadUp1, lastDpadDown1, lastDpadLeft1, lastDpadRight1;
    private boolean lastA1, lastB1, lastX1, lastY1;
    private boolean lastLeftBumper1, lastRightBumper1;
    private boolean lastBack1, lastStart1;
    private boolean lastDpadUp2, lastDpadDown2;
    private boolean lastGuide1;  // For telemetry page navigation
    
    @Override
    public void runOpMode() {
        telemetry.addLine("========================================");
        telemetry.addLine("   MANUAL ARTIFACT INJECTION TEST");
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        // Initialize configs
        indexingConfig = new IndexingConfig();
        shooterConfig = new ShooterConfig();
        
        // Initialize subsystems
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        indexing = new IndexingSystemV3(hardware, indexingConfig, shooter, telemetry);
        
        // Enable systems
        shooter.enable();
        indexing.enable();
        
        // Set default motif pattern for testing
        indexing.setMotifPattern("PPG");
        
        telemetry.addLine("✓ Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();
        
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // Update subsystems
            // Update manual mode based on gamepad2 inputs
            boolean manualActive = gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down;
            indexing.setManualModeActive(manualActive);

            // Update watchdog trigger state
            indexing.setWatchdogTriggerState(gamepad1.right_trigger > 0.1);

            // Update system
            indexing.update();
            shooter.update();
            
            // Handle manual artifact injection (GAMEPAD 1 DPAD)
            handleManualInjection();
            
            // Handle operations (GAMEPAD 1 buttons)
            handleOperations();
            
            // Handle system control (GAMEPAD 1 BACK/START)
            handleSystemControl();
            
            // Handle shooter control (GAMEPAD 2 DPAD)
            handleShooterControl();
            
            // Handle telemetry page navigation
            handleTelemetryNavigation();
            
            // Display telemetry (from IndexingSystemV3)
            indexing.addTelemetry();
            
            // Update button states
            updateButtonStates();
            
            telemetry.update();
        }
    }
    
    /**
     * Handle manual artifact injection (DPAD controls)
     */
    private void handleManualInjection() {
        // FRONT PURPLE - DPAD UP
        if (gamepad1.dpad_up && !lastDpadUp1) {
            boolean success = indexing.addManualArtifact(
                SlotLedger.Slot.FRONT, 
                ArtifactIdentity.ColorClass.PURPLE
            );
            
            if (success) {
                telemetry.addLine("🟣 Injecting PURPLE → FRONT");
            }
        }
        
        // FRONT GREEN - DPAD DOWN
        if (gamepad1.dpad_down && !lastDpadDown1) {
            boolean success = indexing.addManualArtifact(
                SlotLedger.Slot.FRONT, 
                ArtifactIdentity.ColorClass.GREEN
            );
            
            if (success) {
                telemetry.addLine("🟢 Injecting GREEN → FRONT");
            }
        }
        
        // BACK PURPLE - DPAD LEFT
        if (gamepad1.dpad_left && !lastDpadLeft1) {
            boolean success = indexing.addManualArtifact(
                SlotLedger.Slot.BACK, 
                ArtifactIdentity.ColorClass.PURPLE
            );
            
            if (success) {
                telemetry.addLine("🟣 Injecting PURPLE → BACK");
            }
        }
        
        // BACK GREEN - DPAD RIGHT
        if (gamepad1.dpad_right && !lastDpadRight1) {
            boolean success = indexing.addManualArtifact(
                SlotLedger.Slot.BACK, 
                ArtifactIdentity.ColorClass.GREEN
            );
            
            if (success) {
                telemetry.addLine("🟢 Injecting GREEN → BACK");
            }
        }
    }
    
    /**
     * Handle operations (A/B/X/Y/BUMPER controls)
     */
    private void handleOperations() {
        // A - Transfer FRONT → CENTER
        if (gamepad1.a && !lastA1) {
            boolean success = indexing.requestTransfer(SlotLedger.Slot.FRONT);
            if (success) {
                telemetry.addLine("→ Transfer FRONT → CENTER");
            } else {
                telemetry.addLine("❌ Cannot transfer FRONT");
            }
        }
        
        // B - Transfer BACK → CENTER
        if (gamepad1.b && !lastB1) {
            boolean success = indexing.requestTransfer(SlotLedger.Slot.BACK);
            if (success) {
                telemetry.addLine("→ Transfer BACK → CENTER");
            } else {
                telemetry.addLine("❌ Cannot transfer BACK");
            }
        }
        
        // X - Request swap (rearrange)
        if (gamepad1.x && !lastX1) {
            boolean success = indexing.requestSwap(SlotLedger.Slot.FRONT);
            if (success) {
                telemetry.addLine("↔️ Swapping FRONT ↔ BACK");
            } else {
                telemetry.addLine("❌ Cannot swap");
            }
        }
        
        // Y - Fire
        if (gamepad1.y && !lastY1) {
            boolean success = indexing.requestFire();
            if (success) {
                telemetry.addLine("🔥 Firing!");
            } else {
                telemetry.addLine("❌ Cannot fire (CENTER empty or shooter not ready)");
            }
        }
        
        // LEFT BUMPER - Eject CENTER
        if (gamepad1.left_bumper && !lastLeftBumper1) {
            boolean success = indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.CENTER);
            if (success) {
                telemetry.addLine("⏏️ Ejecting CENTER");
            } else {
                telemetry.addLine("❌ Cannot eject CENTER");
            }
        }
        
        // RIGHT BUMPER - Eject ALL
        if (gamepad1.right_bumper && !lastRightBumper1) {
            boolean success = indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.ALL);
            if (success) {
                telemetry.addLine("⏏️ Ejecting ALL");
            } else {
                telemetry.addLine("❌ Cannot eject ALL");
            }
        }
    }
    
    /**
     * Handle system control (BACK/START)
     */
    private void handleSystemControl() {
        // BACK - Clear all slots (reset)
        if (gamepad1.back && !lastBack1) {
            indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.ALL);
            telemetry.addLine("🗑️ Cleared all slots");
        }
        
        // START - Enable/disable
        if (gamepad1.start && !lastStart1) {
            if (indexing.isEnabled()) {
                indexing.disable();
                telemetry.addLine("⏸️ System DISABLED");
            } else {
                indexing.enable();
                telemetry.addLine("▶️ System ENABLED");
            }
        }
    }
    
    /**
     * Handle shooter control (GAMEPAD 2)
     */
    private void handleShooterControl() {
        // DPAD UP - Spin up to HIGH_BASKET
        if (gamepad2.dpad_up && !lastDpadUp2) {
            shooter.spinUp(ShooterConfig.ShooterPreset.LONG_RANGE);
            telemetry.addLine("🎯 Shooter spinning up to 3200 RPM");
        }
        
        // DPAD DOWN - Stop shooter
        if (gamepad2.dpad_down && !lastDpadDown2) {
            shooter.stop();
            telemetry.addLine("⏹️ Shooter stopped");
        }
    }
    
    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        // Removed - now using IndexingSystemV3.addTelemetry() for comprehensive 3-page display
    }
    
    private void handleTelemetryNavigation() {
        // Use Guide button (Xbox logo / PS button) on gamepad1 to cycle telemetry pages
        if (gamepad1.guide && !lastGuide1) {
            indexing.nextTelemetryPage();
            telemetry.addLine("→ Switched to page " + indexing.getTelemetryPage());
        }
    }
    
    /**
     * Get slot status string with color emoji
     */
    private String getSlotStatus(SlotLedger.Slot slot) {
        if (!indexing.getLedger().isOccupied(slot)) {
            return "⬜ Empty";
        }
        
        ArtifactIdentity artifact = indexing.getLedger().get(slot);
        if (artifact == null) return "⬜ Empty";
        
        switch (artifact.getColorClass()) {
            case PURPLE: return "🟣 Purple";
            case GREEN: return "🟢 Green";
            default: return "⚪ Unknown";
        }
    }
    
    /**
     * Update button states for edge detection
     */
    private void updateButtonStates() {
        lastGuide1 = gamepad1.guide;
        lastDpadUp1 = gamepad1.dpad_up;
        lastDpadDown1 = gamepad1.dpad_down;
        lastDpadLeft1 = gamepad1.dpad_left;
        lastDpadRight1 = gamepad1.dpad_right;
        lastA1 = gamepad1.a;
        lastB1 = gamepad1.b;
        lastX1 = gamepad1.x;
        lastY1 = gamepad1.y;
        lastLeftBumper1 = gamepad1.left_bumper;
        lastRightBumper1 = gamepad1.right_bumper;
        lastBack1 = gamepad1.back;
        lastStart1 = gamepad1.start;
        lastDpadUp2 = gamepad2.dpad_up;
        lastDpadDown2 = gamepad2.dpad_down;
    }
}

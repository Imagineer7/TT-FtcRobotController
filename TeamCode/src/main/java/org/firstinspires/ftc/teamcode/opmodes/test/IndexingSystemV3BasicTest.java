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
 *   LEFT BUMPER  - Fire (if ready)
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
 *    - Spin up shooter (check telemetry)
 *    - Press LEFT BUMPER → should fire Purple from CENTER
 *    - Verify CENTER empty
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
    private Shooter shooter;
    
    // Button state tracking
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastBack, lastStart;
    
    @Override
    public void runOpMode() {
        telemetry.addLine("========================================");
        telemetry.addLine("   INDEXING SYSTEM V3 - BASIC TEST");
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();
        
        // Check hardware status
        if (!hardware.isIndexingSystemInitialized()) {
            telemetry.addLine("❌ ERROR: Indexing hardware not initialized");
            telemetry.addLine(hardware.getInitializationSummary());
            telemetry.update();
            while (!isStopRequested()) {
                sleep(100);
            }
            return;
        }
        
        // Initialize configs
        indexingConfig = new IndexingConfig();
        shooterConfig = new ShooterConfig();
        
        // Initialize subsystems
        shooter = new Shooter(hardware, shooterConfig, telemetry);
        indexing = new IndexingSystemV3(hardware, indexingConfig, shooter, telemetry);
        
        // Enable systems
        shooter.enable();
        indexing.enable();
        
        telemetry.addLine("✓ Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Ready to test basic operations");
        telemetry.addLine("See controls on driver station");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();
        
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // Update subsystems
            indexing.update(gamepad1, gamepad2);
            shooter.update();
            
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
            
            // Display telemetry
            displayTelemetry();
            
            // Update button states
            updateButtonStates();
            
            telemetry.update();
        }
    }
    
    private void handleManualInjection() {
        if (gamepad1.dpad_up && !lastDpadUp) {
            indexing.addManualArtifact(SlotLedger.Slot.FRONT, ArtifactIdentity.ColorClass.PURPLE);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            indexing.addManualArtifact(SlotLedger.Slot.FRONT, ArtifactIdentity.ColorClass.GREEN);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            indexing.addManualArtifact(SlotLedger.Slot.BACK, ArtifactIdentity.ColorClass.PURPLE);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            indexing.addManualArtifact(SlotLedger.Slot.BACK, ArtifactIdentity.ColorClass.GREEN);
        }
    }
    
    private void handleCollection() {
        if (gamepad1.a && !lastA) {
            boolean success = indexing.requestCollect(SlotLedger.Slot.FRONT);
            telemetry.addLine(success ? "→ Collecting FRONT" : "❌ Cannot collect FRONT");
        }
        if (gamepad1.b && !lastB) {
            boolean success = indexing.requestCollect(SlotLedger.Slot.BACK);
            telemetry.addLine(success ? "→ Collecting BACK" : "❌ Cannot collect BACK");
        }
    }
    
    private void handleTransfer() {
        if (gamepad1.x && !lastX) {
            boolean success = indexing.requestTransfer(SlotLedger.Slot.FRONT);
            telemetry.addLine(success ? "→ Transfer FRONT → CENTER" : "❌ Cannot transfer FRONT");
        }
        if (gamepad1.y && !lastY) {
            boolean success = indexing.requestTransfer(SlotLedger.Slot.BACK);
            telemetry.addLine(success ? "→ Transfer BACK → CENTER" : "❌ Cannot transfer BACK");
        }
    }
    
    private void handleFire() {
        if (gamepad1.left_bumper && !lastLeftBumper) {
            boolean success = indexing.requestFire();
            telemetry.addLine(success ? "🔥 Firing!" : "❌ Cannot fire");
        }
    }
    
    private void handleEjection() {
        if (gamepad1.back && !lastBack) {
            // Note: V3 uses EjectMode.ALL to eject all artifacts
            indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.ALL);
            telemetry.addLine("⏏️ Clearing all slots");
        }
        if (gamepad1.start && !lastStart) {
            // Note: V3 uses EjectMode.CENTER to eject center only
            boolean success = indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.CENTER);
            telemetry.addLine(success ? "⏏️ Ejecting CENTER" : "❌ Cannot eject CENTER");
        }
    }
    
    private void handleHuntMode() {
        if (gamepad1.right_bumper && !lastRightBumper) {
            boolean newState = indexing.toggleHuntEnabled();
            if (newState) {
                telemetry.addLine("🔍 Hunt mode ENABLED");
            } else {
                telemetry.addLine("💤 Hunt mode DISABLED");
            }
        }
    }
    
    private void displayTelemetry() {
        telemetry.addLine("========== V3 BASIC TEST ==========");
        telemetry.addLine();
        
        // System state
        telemetry.addData("State", indexing.getCurrentState());
        telemetry.addData("Enabled", indexing.isEnabled() ? "✓" : "✗");
        telemetry.addData("Busy", indexing.isBusy() ? "YES" : "NO");
        telemetry.addData("Hunt Mode", indexing.isHuntEnabled() ? "🔍 ON" : "💤 OFF");
        telemetry.addLine();
        
        // Slots
        telemetry.addLine("--- SLOTS ---");
        telemetry.addData("FRONT", getSlotString(SlotLedger.Slot.FRONT));
        telemetry.addData("CENTER", getSlotString(SlotLedger.Slot.CENTER));
        telemetry.addData("BACK", getSlotString(SlotLedger.Slot.BACK));
        telemetry.addData("Count", indexing.getArtifactCount() + "/3");
        telemetry.addLine();
        
        // Shooter
        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("RPM", String.format("%.0f / %.0f", 
                         shooter.getCurrentRPM(), shooter.getTargetRPM()));
        telemetry.addData("Ready", shooter.isReadyToFire() ? "✓" : "✗");
        telemetry.addLine();
        
        // Statistics
        telemetry.addData("Collections", indexing.getTotalCollections());
        telemetry.addData("Transfers", indexing.getTotalTransfers());
        telemetry.addData("Shots", indexing.getTotalShots());
        telemetry.addData("Ejections", indexing.getTotalEjections());
        telemetry.addLine();
        
        // Controls reminder
        telemetry.addLine("DPAD: Manual inject | A/B: Collect");
        telemetry.addLine("X/Y: Transfer | L-Bumper: Fire");
        telemetry.addLine("R-Bumper: Hunt | BACK/START: Eject");
    }
    
    private String getSlotString(SlotLedger.Slot slot) {
        if (!indexing.getLedger().isOccupied(slot)) return "⬜ Empty";
        ArtifactIdentity artifact = indexing.getLedger().get(slot);
        if (artifact == null) return "⬜ Empty";
        
        String emoji = artifact.getColorClass() == ArtifactIdentity.ColorClass.PURPLE ? "🟣" : 
                      artifact.getColorClass() == ArtifactIdentity.ColorClass.GREEN ? "🟢" : "⚪";
        return emoji + " " + artifact.getColorClass();
    }
    
    private void updateButtonStates() {
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

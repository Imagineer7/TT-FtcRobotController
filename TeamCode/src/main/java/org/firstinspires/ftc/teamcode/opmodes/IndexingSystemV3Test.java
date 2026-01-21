package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.IndexingSystemV3;
import org.firstinspires.ftc.teamcode.util.aurora.v3.SlotLedger;

/**
 * IndexingSystemV3Test - Comprehensive integration test for v3 system.
 * 
 * Tests all major features:
 * - Manual collection (A=front, B=back)
 * - Manual transfer (X=auto-select, Y=front, B+Y=back)
 * - Manual swap (left bumper)
 * - Single fire (right trigger)
 * - Burst fire (hold right trigger)
 * - Ejection (back button)
 * - Motif pattern selection (dpad)
 * - Manual control override (gamepad2 dpad)
 * 
 * Controls (Gamepad 1):
 * - A: Collect from FRONT intake
 * - B: Collect from BACK intake
 * - X: Transfer (auto-select best slot)
 * - Y: Transfer from FRONT
 * - B+Y: Transfer from BACK
 * - Right Trigger: Fire (tap=single, hold=burst)
 * - Right Bumper: Preposition
 * - Left Bumper: Swap (for rearrangement)
 * - Back: Emergency eject ALL
 * - Dpad Up: Set motif PPG
 * - Dpad Right: Set motif PGP
 * - Dpad Down: Set motif GPP
 * 
 * Controls (Gamepad 2):
 * - Dpad Left: Manual FRONT intake
 * - Dpad Right: Manual BACK intake
 * - Dpad Up/Down: Manual uptake
 * 
 * Telemetry:
 * - System state and statistics
 * - Slot ledger contents
 * - Current operation progress
 * - Shot planning recommendations
 * - Watchdog status
 * 
 * @author Copilot (AI Assistant)
 * @version 3.0
 * @since 2026-01-20
 */
@TeleOp(name="IndexingSystemV3 Test", group="V3 Testing")
public class IndexingSystemV3Test extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    private IndexingSystemV3 indexing;
    private Shooter shooter;
    
    // Button edge detection
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastRightBumper, lastLeftBumper;
    private boolean lastBack;
    private boolean lastDpadUp, lastDpadRight, lastDpadDown;
    
    @Override
    public void runOpMode() {
        telemetry.addLine("========== INDEXING V3 TEST ==========");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        try {
            // Initialize hardware
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initializeWithOdometry();
            
            // Initialize shooter
            shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
            shooter.enable();
            
            // Initialize indexing system v3
            IndexingConfig config = new IndexingConfig();
            indexing = new IndexingSystemV3(hardware, config, shooter, telemetry);
            indexing.enable();
            
            telemetry.addLine("✓ Initialization complete");
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  A: Collect FRONT");
            telemetry.addLine("  B: Collect BACK");
            telemetry.addLine("  X: Transfer (auto)");
            telemetry.addLine("  Y: Transfer FRONT");
            telemetry.addLine("  RT: Fire (hold=burst)");
            telemetry.addLine("  RB: Preposition");
            telemetry.addLine("  LB: Swap");
            telemetry.addLine("  Back: Eject ALL");
            telemetry.addLine();
            telemetry.addLine("Ready to start!");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("❌ INIT ERROR", e.getMessage());
            telemetry.update();
            return;
        }
        
        waitForStart();
        
        if (!opModeIsActive()) return;
        
        // Set default motif pattern
        indexing.setMotifPattern("PPG");
        
        // Spin up shooter
        shooter.spinUpToRPM(ShooterConfig.RPM_HIGH_BASKET);
        
        // Main loop
        while (opModeIsActive()) {
            try {
                // Update systems
                indexing.update(gamepad1, gamepad2);
                shooter.update();
                
                // Handle button inputs
                handleControls();
                
                // Display telemetry
                displayTelemetry();
                telemetry.update();
                
            } catch (Exception e) {
                telemetry.addData("❌ LOOP ERROR", e.getMessage());
                telemetry.update();
            }
        }
        
        // Cleanup
        indexing.disable();
        shooter.disable();
    }
    
    /**
     * Handle gamepad controls with edge detection.
     */
    private void handleControls() {
        // Collection
        if (gamepad1.a && !lastA) {
            if (indexing.requestCollect(SlotLedger.Slot.FRONT)) {
                telemetry.addData("✓ Requested", "Collect FRONT");
            } else {
                telemetry.addData("✗ Rejected", "Collect FRONT");
            }
        }
        if (gamepad1.b && !lastB) {
            if (indexing.requestCollect(SlotLedger.Slot.BACK)) {
                telemetry.addData("✓ Requested", "Collect BACK");
            } else {
                telemetry.addData("✗ Rejected", "Collect BACK");
            }
        }
        
        // Transfer
        if (gamepad1.x && !lastX) {
            // Auto-select best slot to transfer
            SlotLedger.Slot slot = null;
            if (indexing.getLedger().isFrontOccupied()) {
                slot = SlotLedger.Slot.FRONT;
            } else if (indexing.getLedger().isBackOccupied()) {
                slot = SlotLedger.Slot.BACK;
            }
            
            if (slot != null) {
                if (indexing.requestTransfer(slot)) {
                    telemetry.addData("✓ Requested", "Transfer " + slot);
                } else {
                    telemetry.addData("✗ Rejected", "Transfer " + slot);
                }
            }
        }
        
        if (gamepad1.y && !lastY) {
            SlotLedger.Slot slot = gamepad1.b ? SlotLedger.Slot.BACK : SlotLedger.Slot.FRONT;
            if (indexing.requestTransfer(slot)) {
                telemetry.addData("✓ Requested", "Transfer " + slot);
            } else {
                telemetry.addData("✗ Rejected", "Transfer " + slot);
            }
        }
        
        // Swap
        if (gamepad1.left_bumper && !lastLeftBumper) {
            // Auto-select swap slot (prefer front)
            SlotLedger.Slot swapSlot = SlotLedger.Slot.FRONT;
            if (!indexing.getLedger().isFrontOccupied() && indexing.getLedger().isBackOccupied()) {
                swapSlot = SlotLedger.Slot.BACK;
            }
            
            if (indexing.requestSwap(swapSlot)) {
                telemetry.addData("✓ Requested", "Swap CENTER ↔ " + swapSlot);
            } else {
                telemetry.addData("✗ Rejected", "Swap");
            }
        }
        
        // Preposition
        if (gamepad1.right_bumper && !lastRightBumper) {
            if (indexing.requestPreposition()) {
                telemetry.addData("✓ Requested", "Preposition");
            } else {
                telemetry.addData("✗ Rejected", "Preposition");
            }
        }
        
        // Fire (single shot on release, burst on hold)
        if (gamepad1.right_trigger > 0.1) {
            // Holding trigger - burst fire mode
            if (!indexing.isBurstFiring() && !indexing.isBusy()) {
                // Start burst
                if (indexing.requestBurstFire(() -> gamepad1.right_trigger > 0.1)) {
                    telemetry.addData("✓ Started", "🔥 BURST FIRE");
                }
            }
        }
        
        // Ejection
        if (gamepad1.back && !lastBack) {
            if (indexing.requestEject(org.firstinspires.ftc.teamcode.util.aurora.v3.EjectOperation.EjectMode.ALL)) {
                telemetry.addData("✓ Requested", "Eject ALL");
            } else {
                telemetry.addData("✗ Rejected", "Eject");
            }
        }
        
        // Motif pattern selection
        if (gamepad1.dpad_up && !lastDpadUp) {
            indexing.setMotifPattern("PPG");
            telemetry.addData("Motif", "Set to PPG");
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            indexing.setMotifPattern("PGP");
            telemetry.addData("Motif", "Set to PGP");
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            indexing.setMotifPattern("GPP");
            telemetry.addData("Motif", "Set to GPP");
        }
        
        // Update edge detection
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
        lastBack = gamepad1.back;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadDown = gamepad1.dpad_down;
    }
    
    /**
     * Display comprehensive telemetry.
     */
    private void displayTelemetry() {
        // System telemetry
        indexing.addTelemetry();
        
        // Shooter status
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shooter.getState());
        telemetry.addData("RPM", String.format("%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM()));
        telemetry.addData("Ready", shooter.isReadyToFire() ? "✓" : "✗");
        telemetry.addLine();
        
        // Controls reminder
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A=Collect Front, B=Collect Back");
        telemetry.addLine("X=Transfer Auto, Y=Transfer Front");
        telemetry.addLine("RT=Fire (hold=burst), RB=Preposition");
        telemetry.addLine("LB=Swap, Back=Eject");
    }
}

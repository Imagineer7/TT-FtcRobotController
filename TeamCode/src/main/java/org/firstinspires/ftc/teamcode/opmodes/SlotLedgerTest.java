package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.aurora.v3.SlotLedger;
import org.firstinspires.ftc.teamcode.util.aurora.v3.ArtifactIdentity;

/**
 * SlotLedgerTest - Unit test for SlotLedger component.
 * 
 * Tests:
 * - Slot operations (set, clear, swap)
 * - Occupancy queries
 * - Count tracking
 * - Artifact identity management
 * - Snapshot generation
 * 
 * Controls:
 * - A: Add artifact to FRONT
 * - B: Add artifact to BACK
 * - X: Add artifact to CENTER
 * - Y: Clear CENTER
 * - Left Bumper: Swap FRONT ↔ CENTER
 * - Right Bumper: Swap BACK ↔ CENTER
 * - Back: Clear all slots
 * 
 * @author Copilot (AI Assistant)
 * @version 3.0
 * @since 2026-01-20
 */
@TeleOp(name="SlotLedger Test", group="V3 Testing")
public class SlotLedgerTest extends LinearOpMode {
    
    private SlotLedger ledger;
    private int artifactSequence = 1;
    
    // Button edge detection
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastBack;
    
    @Override
    public void runOpMode() {
        telemetry.addLine("========== SLOT LEDGER TEST ==========");
        telemetry.addLine("Testing slot-based artifact tracking");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: Add FRONT");
        telemetry.addLine("  B: Add BACK");
        telemetry.addLine("  X: Add CENTER");
        telemetry.addLine("  Y: Clear CENTER");
        telemetry.addLine("  LB: Swap FRONT ↔ CENTER");
        telemetry.addLine("  RB: Swap BACK ↔ CENTER");
        telemetry.addLine("  Back: Clear ALL");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();
        
        // Initialize ledger
        ledger = new SlotLedger();
        
        waitForStart();
        
        while (opModeIsActive()) {
            handleControls();
            displayTelemetry();
            telemetry.update();
        }
    }
    
    private void handleControls() {
        // Add artifacts
        if (gamepad1.a && !lastA) {
            if (!ledger.isFrontOccupied()) {
                ArtifactIdentity artifact = ArtifactIdentity.createFromSensor(
                    ArtifactIdentity.ColorClass.PURPLE,
                    0.85,
                    artifactSequence++,
                    ArtifactIdentity.ClassificationSource.COLOR_SENSOR,
                    System.currentTimeMillis()
                );
                ledger.setFront(artifact);
                telemetry.addData("✓ Added", "FRONT slot");
            } else {
                telemetry.addData("✗ Occupied", "FRONT slot");
            }
        }
        
        if (gamepad1.b && !lastB) {
            if (!ledger.isBackOccupied()) {
                ArtifactIdentity artifact = ArtifactIdentity.createFromSensor(
                    ArtifactIdentity.ColorClass.GREEN,
                    0.90,
                    artifactSequence++,
                    ArtifactIdentity.ClassificationSource.COLOR_SENSOR,
                    System.currentTimeMillis()
                );
                ledger.setBack(artifact);
                telemetry.addData("✓ Added", "BACK slot");
            } else {
                telemetry.addData("✗ Occupied", "BACK slot");
            }
        }
        
        if (gamepad1.x && !lastX) {
            if (!ledger.isCenterOccupied()) {
                ArtifactIdentity artifact = ArtifactIdentity.createFromSensor(
                    ArtifactIdentity.ColorClass.PURPLE,
                    0.75,
                    artifactSequence++,
                    ArtifactIdentity.ClassificationSource.COLOR_SENSOR,
                    System.currentTimeMillis()
                );
                ledger.setCenter(artifact);
                telemetry.addData("✓ Added", "CENTER slot");
            } else {
                telemetry.addData("✗ Occupied", "CENTER slot");
            }
        }
        
        // Clear center
        if (gamepad1.y && !lastY) {
            if (ledger.isCenterOccupied()) {
                ledger.setCenter(null);
                telemetry.addData("✓ Cleared", "CENTER slot");
            } else {
                telemetry.addData("✗ Empty", "CENTER slot");
            }
        }
        
        // Swap operations
        if (gamepad1.left_bumper && !lastLeftBumper) {
            if (ledger.isCenterOccupied() || ledger.isFrontOccupied()) {
                ledger.swap(SlotLedger.Slot.CENTER, SlotLedger.Slot.FRONT);
                telemetry.addData("✓ Swapped", "CENTER ↔ FRONT");
            } else {
                telemetry.addData("✗ Both Empty", "Cannot swap");
            }
        }
        
        if (gamepad1.right_bumper && !lastRightBumper) {
            if (ledger.isCenterOccupied() || ledger.isBackOccupied()) {
                ledger.swap(SlotLedger.Slot.CENTER, SlotLedger.Slot.BACK);
                telemetry.addData("✓ Swapped", "CENTER ↔ BACK");
            } else {
                telemetry.addData("✗ Both Empty", "Cannot swap");
            }
        }
        
        // Clear all
        if (gamepad1.back && !lastBack) {
            ledger.setCenter(null);
            ledger.setFront(null);
            ledger.setBack(null);
            telemetry.addData("✓ Cleared", "ALL slots");
        }
        
        // Update edge detection
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
        lastBack = gamepad1.back;
    }
    
    private void displayTelemetry() {
        telemetry.addLine("========== SLOT LEDGER ==========");
        telemetry.addData("Count", ledger.getCount() + "/3");
        telemetry.addData("Full", ledger.isFull() ? "YES" : "No");
        telemetry.addData("Empty", ledger.isEmpty() ? "YES" : "No");
        telemetry.addLine();
        
        // CENTER slot
        telemetry.addLine("--- CENTER ---");
        if (ledger.isCenterOccupied()) {
            ArtifactIdentity artifact = ledger.getCenter();
            telemetry.addData("Color", artifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.0f%%", artifact.getColorConfidence() * 100));
            telemetry.addData("Source", artifact.getSource());
            telemetry.addData("Sequence ID", artifact.getSequenceId());
        } else {
            telemetry.addData("Status", "EMPTY");
        }
        telemetry.addLine();
        
        // FRONT slot
        telemetry.addLine("--- FRONT ---");
        if (ledger.isFrontOccupied()) {
            ArtifactIdentity artifact = ledger.getFront();
            telemetry.addData("Color", artifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.0f%%", artifact.getColorConfidence() * 100));
            telemetry.addData("Source", artifact.getSource());
            telemetry.addData("Sequence ID", artifact.getSequenceId());
        } else {
            telemetry.addData("Status", "EMPTY");
        }
        telemetry.addLine();
        
        // BACK slot
        telemetry.addLine("--- BACK ---");
        if (ledger.isBackOccupied()) {
            ArtifactIdentity artifact = ledger.getBack();
            telemetry.addData("Color", artifact.getColorClass());
            telemetry.addData("Confidence", String.format("%.0f%%", artifact.getColorConfidence() * 100));
            telemetry.addData("Source", artifact.getSource());
            telemetry.addData("Sequence ID", artifact.getSequenceId());
        } else {
            telemetry.addData("Status", "EMPTY");
        }
        telemetry.addLine();
        
        // Queries
        telemetry.addLine("--- Queries ---");
        telemetry.addData("Find PURPLE", ledger.findArtifactByColor(ArtifactIdentity.ColorClass.PURPLE));
        telemetry.addData("Find GREEN", ledger.findArtifactByColor(ArtifactIdentity.ColorClass.GREEN));
        telemetry.addData("Empty Slots", ledger.getEmptySlots().size());
        telemetry.addLine();
        
        // Controls
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A=Add FRONT, B=Add BACK, X=Add CENTER");
        telemetry.addLine("Y=Clear CENTER, Back=Clear ALL");
        telemetry.addLine("LB=Swap FRONT↔CENTER, RB=Swap BACK↔CENTER");
    }
}

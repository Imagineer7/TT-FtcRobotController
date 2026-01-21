package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.v3.FireOperation;
import org.firstinspires.ftc.teamcode.util.aurora.v3.KeepAliveWatchdog;
import org.firstinspires.ftc.teamcode.util.aurora.v3.OperationRunner;
import org.firstinspires.ftc.teamcode.util.aurora.v3.SlotLedger;
import org.firstinspires.ftc.teamcode.util.aurora.v3.ArtifactIdentity;
import org.firstinspires.ftc.teamcode.util.aurora.v3.ShotPlanningCoordinator;
import org.firstinspires.ftc.teamcode.util.aurora.ShotPlanner;

/**
 * BurstFireTest - Test keep-alive burst firing sequences.
 * 
 * Tests:
 * - Keep-alive mode (shooter stays spun)
 * - Mid-operation cancellation (release trigger)
 * - Shot counter reliability
 * - KeepAliveWatchdog enforcement
 * - Burst sequence orchestration
 * 
 * Controls:
 * - Right Trigger: Burst fire (hold to fire multiple shots)
 * - A: Add artifact to CENTER (for testing)
 * - B: Reset shot counter
 * - X: Manual cancel (stop shooter)
 * - Y: Toggle watchdog enable
 * 
 * Statistics Tracked:
 * - Total shots fired (from helper counter)
 * - Operation count
 * - Burst duration
 * - Average shot rate
 * - Cancellation events
 * 
 * @author Copilot (AI Assistant)
 * @version 3.0
 * @since 2026-01-20
 */
@TeleOp(name="Burst Fire Test", group="V3 Testing")
public class BurstFireTest extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;
    private BasicFiringHelper firingHelper;
    private Shooter shooter;
    private SlotLedger ledger;
    private OperationRunner runner;
    private ShotPlanningCoordinator shotPlanner;
    private KeepAliveWatchdog watchdog;
    
    private long burstStartTime;
    private int shotsAtBurstStart;
    private int artifactSequence = 1;
    private boolean watchdogEnabled = true;
    
    // Button edge detection
    private boolean lastA, lastB, lastX, lastY;
    
    @Override
    public void runOpMode() {
        telemetry.addLine("========== BURST FIRE TEST ==========");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        try {
            // Initialize hardware
            hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
            hardware.initializeWithOdometry();
            
            // Initialize helpers
            indexingHelper = new BasicIndexingHelper(hardware, telemetry);
            shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
            shooter.enable();
            firingHelper = new BasicFiringHelper(hardware, telemetry, indexingHelper, shooter);
            
            // Initialize v3 components
            ledger = new SlotLedger();
            runner = new OperationRunner(telemetry);
            ShotPlanner planner = new ShotPlanner(telemetry);
            shotPlanner = new ShotPlanningCoordinator(planner, telemetry);
            watchdog = new KeepAliveWatchdog(firingHelper, telemetry);
            
            telemetry.addLine("✓ Initialization complete");
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  RT: Burst fire (hold)");
            telemetry.addLine("  A: Add artifact to CENTER");
            telemetry.addLine("  B: Reset shot counter");
            telemetry.addLine("  X: Manual cancel shooter");
            telemetry.addLine("  Y: Toggle watchdog");
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
        
        // Spin up shooter
        shooter.spinUpToRPM(ShooterConfig.RPM_HIGH_BASKET);
        
        // Main loop
        while (opModeIsActive()) {
            try {
                // Update systems
                shooter.update();
                runner.update();
                
                // Update watchdog if enabled
                if (watchdogEnabled) {
                    boolean triggerPressed = gamepad1.right_trigger > 0.1;
                    watchdog.update(triggerPressed, runner.isBusy(), false);
                }
                
                // Handle controls
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
        firingHelper.cancelFiring();
        shooter.disable();
    }
    
    private void handleControls() {
        // Burst fire (hold trigger)
        if (gamepad1.right_trigger > 0.1) {
            if (!runner.isBusy() && ledger.isCenterOccupied()) {
                // Start new fire operation
                if (shotsAtBurstStart == 0) {
                    burstStartTime = System.currentTimeMillis();
                    shotsAtBurstStart = firingHelper.getShotsFiredCount();
                }
                
                double rpm = shooter.getTargetRPM();
                FireOperation.ShouldContinueCallback shouldContinue = () -> gamepad1.right_trigger > 0.1;
                
                FireOperation op = new FireOperation(
                    ledger, firingHelper, shooter, rpm, 
                    true,  // Keep-alive mode
                    shotPlanner, shouldContinue, telemetry
                );
                
                runner.start(op);
                telemetry.addData("✓ Started", "Fire operation");
            }
        } else {
            // Trigger released - reset burst tracking
            if (shotsAtBurstStart > 0) {
                shotsAtBurstStart = 0;
            }
        }
        
        // Add artifact to center (for testing)
        if (gamepad1.a && !lastA) {
            if (!ledger.isCenterOccupied()) {
                ArtifactIdentity artifact = ArtifactIdentity.createFromSensor(
                    ArtifactIdentity.ColorClass.PURPLE,
                    0.85,
                    artifactSequence++,
                    ArtifactIdentity.ClassificationSource.COLOR_SENSOR,
                    System.currentTimeMillis()
                );
                ledger.setCenter(artifact);
                telemetry.addData("✓ Added", "Artifact to CENTER");
            }
        }
        
        // Reset shot counter
        if (gamepad1.b && !lastB) {
            firingHelper.resetShotDetection();
            telemetry.addData("✓ Reset", "Shot counter");
        }
        
        // Manual cancel
        if (gamepad1.x && !lastX) {
            if (runner.isBusy()) {
                runner.forceCancel();
                telemetry.addData("✓ Cancelled", "Operation");
            }
            if (firingHelper.isFiring()) {
                firingHelper.cancelFiring();
                telemetry.addData("✓ Stopped", "Shooter");
            }
        }
        
        // Toggle watchdog
        if (gamepad1.y && !lastY) {
            watchdogEnabled = !watchdogEnabled;
            telemetry.addData("Watchdog", watchdogEnabled ? "ENABLED" : "DISABLED");
        }
        
        // Update edge detection
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
    }
    
    private void displayTelemetry() {
        telemetry.addLine("========== BURST FIRE TEST ==========");
        telemetry.addLine();
        
        // Shot counter status
        telemetry.addLine("--- Shot Counter ---");
        telemetry.addData("Total Shots", firingHelper.getShotsFiredCount());
        telemetry.addData("Has New Shot", firingHelper.hasNewShotFired() ? "YES" : "No");
        
        if (shotsAtBurstStart > 0) {
            int shotsFired = firingHelper.getShotsFiredCount() - shotsAtBurstStart;
            long duration = System.currentTimeMillis() - burstStartTime;
            double shotRate = (shotsFired > 0) ? (shotsFired / (duration / 1000.0)) : 0.0;
            telemetry.addData("Burst Shots", shotsFired);
            telemetry.addData("Burst Duration", String.format("%.1fs", duration / 1000.0));
            telemetry.addData("Shot Rate", String.format("%.1f shots/sec", shotRate));
        }
        telemetry.addLine();
        
        // Slot ledger
        telemetry.addLine("--- Slot Ledger ---");
        telemetry.addData("CENTER", ledger.isCenterOccupied() ? "OCCUPIED" : "EMPTY");
        telemetry.addLine();
        
        // Operation runner
        telemetry.addLine("--- Operation Runner ---");
        telemetry.addData("Busy", runner.isBusy() ? "YES" : "No");
        if (runner.isBusy()) {
            telemetry.addData("Current", runner.getCurrentOperationName());
            telemetry.addData("Progress", String.format("%.0f%%", runner.getCurrentOperation().getProgressPercent()));
        }
        telemetry.addData("Total Ops", runner.getOperationCount());
        telemetry.addLine();
        
        // Shooter status
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shooter.getState());
        telemetry.addData("RPM", String.format("%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM()));
        telemetry.addData("Ready", shooter.isReadyToFire() ? "✓" : "✗");
        telemetry.addLine();
        
        // Firing helper status
        telemetry.addLine("--- Firing Helper ---");
        telemetry.addData("State", firingHelper.getFiringState());
        telemetry.addData("Is Firing", firingHelper.isFiring() ? "YES" : "No");
        telemetry.addData("Ready For Next", firingHelper.isReadyForNextShot() ? "YES" : "No");
        telemetry.addLine();
        
        // Watchdog status
        if (watchdogEnabled) {
            watchdog.addTelemetry();
        } else {
            telemetry.addData("⚠️ Watchdog", "DISABLED");
        }
        telemetry.addLine();
        
        // Controls
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("RT=Burst Fire (hold), A=Add Artifact");
        telemetry.addLine("B=Reset Counter, X=Cancel, Y=Toggle Watchdog");
    }
}

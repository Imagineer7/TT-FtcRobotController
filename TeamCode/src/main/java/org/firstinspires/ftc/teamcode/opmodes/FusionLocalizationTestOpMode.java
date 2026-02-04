package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.localization.FusionLocalizer;
import org.firstinspires.ftc.teamcode.util.aurora.localization.PredefinedPoses;
import org.firstinspires.ftc.teamcode.util.aurora.localization.RobotPose2D;

/**
 * Test OpMode for the Fusion Localization System
 * 
 * This OpMode demonstrates the dual-pose API:
 * - Relative pose: Smooth odometry (for drivetrain visualization)
 * - Absolute pose: Vision-corrected (for turret/field tasks)
 * 
 * Controls:
 * - D-Pad Up/Down: Select start position during init
 * - A: Reset to origin
 * - B: Reset to selected start position
 * - X: Toggle debug telemetry
 * - Y: Show statistics
 * 
 * The robot should be stationary during this test to observe
 * the fusion behavior and vision correction application.
 */
@TeleOp(name = "Test: Fusion Localization", group = "Test")
public class FusionLocalizationTestOpMode extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    private FusionLocalizer localizer;
    
    private int selectedStartIndex = 0;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    
    private boolean showDebug = false;
    private boolean showStats = false;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // ═══════════════════════════════════════════════════════════════════════
        // INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════════
        
        telemetry.addLine("🤖 Initializing Fusion Localization System...");
        telemetry.update();
        
        // Initialize hardware
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initializeWithOdometry();
        
        // Let user select start position
        telemetry.clear();
        telemetry.addLine("═══ START POSITION SELECTION ═══");
        telemetry.addLine("Use D-Pad Up/Down to select");
        telemetry.addLine("Press START when ready");
        telemetry.addLine();
        
        while (!isStarted() && !isStopRequested()) {
            // Handle D-Pad input
            if (gamepad1.dpad_up && !lastDpadUp) {
                selectedStartIndex = (selectedStartIndex - 1 + 6) % 6;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                selectedStartIndex = (selectedStartIndex + 1) % 6;
            }
            
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            
            // Show selected position
            PredefinedPoses.StartPosition selected = 
                PredefinedPoses.selectPosition(selectedStartIndex);
            RobotPose2D startPose = PredefinedPoses.getPose(selected);
            
            telemetry.addData("Selected Position", PredefinedPoses.getName(selected));
            telemetry.addData("Coordinates", "(%.1f, %.1f) mm",
                startPose.x, startPose.y);
            telemetry.addData("Heading", "%.1f°",
                Math.toDegrees(startPose.heading));
            telemetry.update();
            
            idle();
        }
        
        // Create localizer with selected start position
        PredefinedPoses.StartPosition startPosition = 
            PredefinedPoses.selectPosition(selectedStartIndex);
        
        telemetry.clear();
        telemetry.addLine("🤖 Creating Fusion Localizer...");
        telemetry.addData("Start Position", PredefinedPoses.getName(startPosition));
        telemetry.update();
        
        localizer = new FusionLocalizer(hardware, startPosition);
        
        // Verify initialization
        if (!localizer.isInitialized()) {
            telemetry.addLine("❌ ERROR: Localizer not initialized!");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }
        
        telemetry.addLine("✅ Fusion Localizer Ready");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();
        
        waitForStart();
        
        // ═══════════════════════════════════════════════════════════════════════
        // MAIN LOOP
        // ═══════════════════════════════════════════════════════════════════════
        
        while (opModeIsActive()) {
            // Update localizer (predict + optional correct)
            localizer.update();
            
            // Get poses
            RobotPose2D relativePose = localizer.getRelativePose();
            RobotPose2D absolutePose = localizer.getAbsolutePose();
            
            // Handle button inputs
            handleButtons();
            
            // Display telemetry
            telemetry.clear();
            
            if (showStats) {
                displayStatistics();
            } else if (showDebug) {
                localizer.addDebugTelemetry(telemetry);
            } else {
                displayStandardTelemetry(relativePose, absolutePose);
            }
            
            // Controls help
            telemetry.addLine();
            telemetry.addLine("═══ Controls ═══");
            telemetry.addData("A", "Reset to origin");
            telemetry.addData("B", "Reset to start position");
            telemetry.addData("X", "Toggle debug telemetry");
            telemetry.addData("Y", "Toggle statistics");
            
            telemetry.update();
        }
        
        // Cleanup
        // (No explicit cleanup needed for localizer)
    }
    
    /**
     * Handle button inputs
     */
    private void handleButtons() {
        // A: Reset to origin
        if (gamepad1.a && !lastA) {
            localizer.reset(new RobotPose2D(0, 0, 0));
            telemetry.speak("Position reset to origin");
        }
        
        // B: Reset to start position
        if (gamepad1.b && !lastB) {
            PredefinedPoses.StartPosition position = 
                PredefinedPoses.selectPosition(selectedStartIndex);
            localizer.reset(position);
            telemetry.speak("Position reset to start");
        }
        
        // X: Toggle debug
        if (gamepad1.x && !lastX) {
            showDebug = !showDebug;
            showStats = false;
        }
        
        // Y: Toggle stats
        if (gamepad1.y && !lastY) {
            showStats = !showStats;
            showDebug = false;
        }
        
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
    }
    
    /**
     * Display standard telemetry (default view)
     */
    private void displayStandardTelemetry(RobotPose2D relativePose, RobotPose2D absolutePose) {
        telemetry.addLine("═══ Fusion Localization System ═══");
        telemetry.addLine();
        
        // Relative pose (odometry-only, smooth)
        telemetry.addLine("--- RELATIVE POSE (Drivetrain) ---");
        telemetry.addData("Position", "(%.1f, %.1f) in",
            relativePose.getX(DistanceUnit.INCH),
            relativePose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", "%.1f°",
            relativePose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Uncertainty", "%.1f in",
            relativePose.getPositionUncertainty() / 25.4);
        telemetry.addLine();
        
        // Absolute pose (vision-corrected, stable)
        telemetry.addLine("--- ABSOLUTE POSE (Turret/Field) ---");
        telemetry.addData("Position", "(%.1f, %.1f) in",
            absolutePose.getX(DistanceUnit.INCH),
            absolutePose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", "%.1f°",
            absolutePose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Uncertainty", "%.1f in",
            absolutePose.getPositionUncertainty() / 25.4);
        telemetry.addLine();
        
        // Vision status
        telemetry.addLine("--- VISION STATUS ---");
        if (localizer.isVisionActive()) {
            long age = localizer.getTimeSinceLastVisionUpdate();
            telemetry.addData("Status", "✅ ACTIVE (%d ms ago)", age);
        } else {
            long age = localizer.getTimeSinceLastVisionUpdate();
            if (age < 0) {
                telemetry.addData("Status", "⏳ WAITING (no corrections yet)");
            } else {
                telemetry.addData("Status", "❌ INACTIVE (%d ms ago)", age);
            }
        }
        
        double acceptRate = localizer.getVisionAcceptanceRate() * 100;
        telemetry.addData("Acceptance Rate", "%.0f%%", acceptRate);
        telemetry.addData("Accepted", localizer.getVisionAcceptCount());
        telemetry.addData("Rejected", localizer.getVisionRejectCount());
        telemetry.addLine();
        
        // Velocity
        telemetry.addLine("--- VELOCITY ---");
        telemetry.addData("Linear", "%.1f in/s",
            localizer.getVelocityMagnitude(DistanceUnit.INCH));
        telemetry.addData("Angular", "%.1f °/s",
            localizer.getHeadingVelocity(AngleUnit.DEGREES));
    }
    
    /**
     * Display statistics view
     */
    private void displayStatistics() {
        telemetry.addLine("═══ STATISTICS ═══");
        telemetry.addLine();
        
        // Vision statistics
        telemetry.addLine("--- VISION MEASUREMENTS ---");
        telemetry.addData("Total Accepted", localizer.getVisionAcceptCount());
        telemetry.addData("Total Rejected", localizer.getVisionRejectCount());
        telemetry.addData("Acceptance Rate", "%.1f%%",
            localizer.getVisionAcceptanceRate() * 100);
        telemetry.addLine();
        
        // Pose difference (relative vs absolute)
        RobotPose2D rel = localizer.getRelativePose();
        RobotPose2D abs = localizer.getAbsolutePose();
        double dx = abs.x - rel.x;
        double dy = abs.y - rel.y;
        double distance = Math.sqrt(dx*dx + dy*dy);
        double dh = Math.toDegrees(RobotPose2D.angleWrap(abs.heading - rel.heading));
        
        telemetry.addLine("--- POSE DIVERGENCE ---");
        telemetry.addData("Position Diff", "%.1f mm (%.1f in)",
            distance, distance / 25.4);
        telemetry.addData("Heading Diff", "%.1f°", dh);
        telemetry.addLine();
        
        // Uncertainty
        telemetry.addLine("--- UNCERTAINTY ---");
        telemetry.addData("Relative Pose", "%.1f mm", rel.getPositionUncertainty());
        telemetry.addData("Absolute Pose", "%.1f mm", abs.getPositionUncertainty());
        telemetry.addData("Ratio", "%.2f", 
            rel.getPositionUncertainty() / abs.getPositionUncertainty());
    }
}

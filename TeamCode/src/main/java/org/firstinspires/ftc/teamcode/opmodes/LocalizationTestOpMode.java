package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.aurora.Localization;

/**
 * AURORA Localization System Test OpMode
 * 
 * This OpMode demonstrates how to use the Localization class for robot positioning.
 * It combines Pinpoint odometry with Limelight vision for accurate localization.
 * 
 * Usage:
 * - Robot will track position using odometry
 * - When AprilTags are visible, Limelight will correct position drift
 * - Position is displayed on telemetry
 * - Press A to reset position to origin
 */
@TeleOp(name = "Test: Localization System", group = "Test")
@Disabled  // Remove this line to enable the OpMode
public class LocalizationTestOpMode extends LinearOpMode {
    
    private Localization localization;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize localization system
        telemetry.addLine("Initializing Localization System...");
        telemetry.update();
        
        localization = new Localization(hardwareMap);
        
        // Report initialization status
        telemetry.addLine("Initialization Complete:");
        telemetry.addData("  Odometry", localization.isOdometryInitialized() ? "✅" : "❌");
        telemetry.addData("  Limelight", localization.isLimelightInitialized() ? "✅" : "❌");
        telemetry.addLine();
        telemetry.addLine("Press START to begin tracking");
        telemetry.addLine("Press A to reset position");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update localization (includes odometry and vision fusion)
            localization.update();
            
            // Get current position
            Pose2D pose = localization.getPosition();
            double x = localization.getX(DistanceUnit.INCH);
            double y = localization.getY(DistanceUnit.INCH);
            double heading = localization.getHeading(AngleUnit.DEGREES);
            
            // Get velocity
            double velX = localization.getVelocityX(DistanceUnit.INCH);
            double velY = localization.getVelocityY(DistanceUnit.INCH);
            
            // Reset position if A is pressed
            if (gamepad1.a) {
                localization.resetPosition();
                telemetry.addLine("Position Reset!");
            }
            
            // Display position information
            telemetry.addLine("═══ Robot Position ═══");
            telemetry.addData("X Position", "%.2f in", x);
            telemetry.addData("Y Position", "%.2f in", y);
            telemetry.addData("Heading", "%.1f°", heading);
            telemetry.addLine();
            
            telemetry.addLine("═══ Velocity ═══");
            telemetry.addData("X Velocity", "%.2f in/s", velX);
            telemetry.addData("Y Velocity", "%.2f in/s", velY);
            telemetry.addLine();
            
            // Display sensor status
            telemetry.addLine("═══ Sensor Status ═══");
            if (localization.isOdometryInitialized()) {
                telemetry.addData("Odometry Status", localization.getOdometryStatus());
                telemetry.addData("Odometry Freq", "%.1f Hz", localization.getOdometryFrequency());
            } else {
                telemetry.addData("Odometry", "Not initialized");
            }
            
            if (localization.isLimelightInitialized()) {
                boolean hasTarget = localization.getLimelight().hasTarget();
                telemetry.addData("Limelight Target", hasTarget ? "✅ Visible" : "❌ None");
                
                if (hasTarget) {
                    telemetry.addData("Vision Data", localization.isVisionDataFresh() ? "Fresh" : "Stale");
                    telemetry.addData("Data Age", "%d ms", localization.getVisionDataAge());
                    telemetry.addData("Data Quality", localization.isVisionDataQualityGood() ? "Good" : "Poor");
                }
            } else {
                telemetry.addData("Limelight", "Not initialized");
            }
            
            telemetry.addLine();
            telemetry.addLine("Press A to reset position");
            telemetry.update();
        }
        
        // Clean up
        localization.stop();
    }
}

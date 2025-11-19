/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Example demonstrating the unified AuroraHardwareConfig class
 */

package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;

/**
 * Example OpMode demonstrating how to use AuroraHardwareConfig
 * 
 * This example shows:
 * 1. How to initialize the hardware configuration
 * 2. How to access individual hardware components
 * 3. How to check initialization status
 * 4. How to handle errors gracefully
 * 
 * This is a simplified example - for real robot control, use AuroraManager instead.
 */
@TeleOp(name="Hardware Config Example", group="Examples")
@Disabled  // Remove this annotation to enable the OpMode
public class AuroraHardwareConfigExample extends LinearOpMode {
    
    private AuroraHardwareConfig hardware;
    
    @Override
    public void runOpMode() {
        // ============================================================================
        // STEP 1: Initialize the unified hardware configuration
        // ============================================================================
        
        telemetry.addLine("=== AURORA Hardware Configuration Example ===");
        telemetry.addLine("");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        // Create hardware config object
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        
        // Initialize with odometry (for TeleOp)
        // Use hardware.initialize() for Autonomous without odometry
        hardware.initializeWithOdometry();
        
        // ============================================================================
        // STEP 2: Check initialization status
        // ============================================================================
        
        telemetry.clear();
        telemetry.addLine("=== Initialization Complete ===");
        telemetry.addLine("");
        
        // Display initialization summary
        telemetry.addLine(hardware.getInitializationSummary());
        telemetry.addLine("");
        
        // Check if critical systems initialized
        if (hardware.isFullyInitialized()) {
            telemetry.addLine("✅ All critical systems initialized successfully!");
        } else {
            telemetry.addLine("⚠️ Some systems failed to initialize");
            telemetry.addLine("Robot may have limited functionality");
        }
        
        telemetry.addLine("");
        telemetry.addLine("Press START to begin");
        telemetry.update();
        
        // ============================================================================
        // STEP 3: Wait for driver to press START
        // ============================================================================
        
        waitForStart();
        
        // ============================================================================
        // STEP 4: Access hardware components
        // ============================================================================
        
        // Get drive motors (will be null if initialization failed)
        DcMotor frontLeft = hardware.getFrontLeftMotor();
        DcMotor frontRight = hardware.getFrontRightMotor();
        DcMotor backLeft = hardware.getBackLeftMotor();
        DcMotor backRight = hardware.getBackRightMotor();
        
        // Get shooter components
        DcMotor shooter = hardware.getShooterMotor();
        
        // Get sensors
        var voltage = hardware.getVoltageSensor();
        var odometry = hardware.getOdometry();
        
        // ============================================================================
        // STEP 5: Main control loop
        // ============================================================================
        
        while (opModeIsActive()) {
            // Example: Simple tank drive using hardware config
            if (hardware.isDriveSystemInitialized()) {
                double leftPower = -gamepad1.left_stick_y;
                double rightPower = -gamepad1.right_stick_y;
                
                // Null checks are important!
                if (frontLeft != null && backLeft != null) {
                    frontLeft.setPower(leftPower);
                    backLeft.setPower(leftPower);
                }
                
                if (frontRight != null && backRight != null) {
                    frontRight.setPower(rightPower);
                    backRight.setPower(rightPower);
                }
                
                telemetry.addData("Left Power", "%.2f", leftPower);
                telemetry.addData("Right Power", "%.2f", rightPower);
            } else {
                telemetry.addLine("⚠️ Drive system not available");
                telemetry.addData("Error", hardware.getDriveInitError());
            }
            
            telemetry.addLine("");
            
            // Example: Display shooter status
            if (hardware.isShooterSystemInitialized()) {
                telemetry.addLine("✅ Shooter system available");
                if (shooter != null) {
                    telemetry.addData("Shooter Encoder", shooter.getCurrentPosition());
                }
            } else {
                telemetry.addLine("⚠️ Shooter system not available");
                telemetry.addData("Error", hardware.getShooterInitError());
            }
            
            telemetry.addLine("");
            
            // Example: Display sensor data
            if (voltage != null) {
                telemetry.addData("Battery Voltage", "%.1f V", voltage.getVoltage());
            } else {
                telemetry.addLine("⚠️ Voltage sensor not available");
            }
            
            if (odometry != null) {
                telemetry.addData("Odometry Status", odometry.getDeviceStatus());
            } else {
                telemetry.addLine("⚠️ Odometry not available");
            }
            
            telemetry.addLine("");
            telemetry.addLine("=== Controls ===");
            telemetry.addLine("Left Stick: Left side drive");
            telemetry.addLine("Right Stick: Right side drive");
            
            telemetry.update();
        }
        
        // ============================================================================
        // STEP 6: Cleanup (optional)
        // ============================================================================
        
        // Stop all motors
        if (hardware.isDriveSystemInitialized()) {
            if (frontLeft != null) frontLeft.setPower(0);
            if (frontRight != null) frontRight.setPower(0);
            if (backLeft != null) backLeft.setPower(0);
            if (backRight != null) backRight.setPower(0);
        }
        
        if (hardware.isShooterSystemInitialized() && shooter != null) {
            shooter.setPower(0);
        }
        
        telemetry.addLine("OpMode stopped - all motors stopped");
        telemetry.update();
    }
}

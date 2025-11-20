/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * StarterDrive - Unified drive system supporting tank and mecanum configurations
 * 
 * This class handles robot movement for both tank (2-wheel) and mecanum (4-wheel)
 * drive systems with a single, easy-to-use interface.
 */

package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * StarterDrive - Flexible drive system for FTC robots
 * 
 * Purpose:
 * - Support both tank and mecanum drive configurations
 * - Provide smooth, controlled movement
 * - Include safety features like power limiting
 * - Easy to understand and modify
 * 
 * Drive Types:
 * - TANK: Traditional left/right side drive (2 or 4 motors)
 * - MECANUM: Holonomic drive with strafing (4 motors with mecanum wheels)
 * 
 * Motor Configuration:
 * Your robot hardware config must have these motor names:
 * - frontLeft
 * - frontRight
 * - backLeft
 * - backRight
 * 
 * Important: Motor directions are set in configureMotors() - adjust if needed!
 */
public class StarterDrive {
    
    // Drive type enum
    public enum DriveType {
        TANK,      // Tank drive - left/right side control
        MECANUM    // Mecanum drive - omnidirectional movement
    }
    
    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DriveType driveType;
    
    // Power limiting (prevents damage and improves control)
    private double maxPower = 0.8;  // 80% max power for safer driving
    
    /**
     * Constructor - initializes drive system
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param driveType Type of drive (TANK or MECANUM)
     * @throws IllegalArgumentException if motors not found in hardware config
     */
    public StarterDrive(HardwareMap hardwareMap, DriveType driveType) {
        this.driveType = driveType;
        
        // Get motors from hardware map
        // These names MUST match your robot configuration file!
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        configureMotors();
    }
    
    /**
     * Configure motors with optimal settings
     * 
     * Why these settings?
     * - RUN_USING_ENCODER: Better speed control and consistency
     * - BRAKE: Robot stops quickly when joysticks released (safer)
     * - REVERSE left motors: Standardizes "forward" direction for all motors
     */
    private void configureMotors() {
        // Use encoders for better control
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Brake mode - robot stops quickly (safer than coast)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set motor directions
        // Standard configuration: left motors reversed, right motors forward
        // If your robot drives backward when pushing forward, swap these!
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    
    /**
     * Main drive method - works for both tank and mecanum
     * 
     * @param axial Forward/backward movement (-1.0 to 1.0)
     * @param lateral Left/right strafe (-1.0 to 1.0) - ignored in tank mode
     * @param yaw Rotation (-1.0 to 1.0)
     */
    public void drive(double axial, double lateral, double yaw) {
        if (driveType == DriveType.TANK) {
            driveTank(axial, yaw);
        } else {
            driveMecanum(axial, lateral, yaw);
        }
    }
    
    /**
     * Tank drive implementation
     * 
     * Tank drive uses "arcade" style control:
     * - axial (forward/back) + yaw (turn) = differential steering
     * - All wheels on same side move together
     * 
     * @param axial Forward/backward power
     * @param yaw Rotation power
     */
    private void driveTank(double axial, double yaw) {
        // Calculate left and right side powers
        double leftPower = axial + yaw;   // Forward + turn right = right side faster
        double rightPower = axial - yaw;  // Forward + turn right = left side slower
        
        // Normalize powers if any exceed 1.0 (prevents unintended speed changes)
        double maxInput = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxInput > 1.0) {
            leftPower /= maxInput;
            rightPower /= maxInput;
        }
        
        // Apply max power limit
        leftPower *= maxPower;
        rightPower *= maxPower;
        
        // Set motor powers
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }
    
    /**
     * Mecanum drive implementation
     * 
     * Mecanum wheels allow omnidirectional movement:
     * - axial: Forward/backward
     * - lateral: Strafe left/right
     * - yaw: Rotate in place
     * 
     * Each wheel contributes to all three movement types based on its roller angle.
     * 
     * @param axial Forward/backward power
     * @param lateral Strafe left/right power
     * @param yaw Rotation power
     */
    private void driveMecanum(double axial, double lateral, double yaw) {
        // Calculate power for each wheel
        // This math comes from vector analysis of mecanum wheel geometry
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;
        
        // Normalize powers (keep movement direction correct even at max speed)
        double maxInput = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        
        if (maxInput > 1.0) {
            frontLeftPower /= maxInput;
            frontRightPower /= maxInput;
            backLeftPower /= maxInput;
            backRightPower /= maxInput;
        }
        
        // Apply max power limit
        frontLeftPower *= maxPower;
        frontRightPower *= maxPower;
        backLeftPower *= maxPower;
        backRightPower *= maxPower;
        
        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    
    /**
     * Stop all motors immediately
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
    /**
     * Set maximum power limit (0.0 to 1.0)
     * 
     * Lower values = slower but more precise control
     * Higher values = faster but harder to control
     * 
     * @param maxPower Maximum power (0.0 to 1.0)
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.0, Math.min(1.0, maxPower));
    }
    
    /**
     * Get current max power setting
     */
    public double getMaxPower() {
        return maxPower;
    }
    
    /**
     * Get drive type
     */
    public DriveType getDriveType() {
        return driveType;
    }
    
    /**
     * Get motor encoder positions (useful for autonomous)
     */
    public int[] getEncoderPositions() {
        return new int[] {
            frontLeft.getCurrentPosition(),
            frontRight.getCurrentPosition(),
            backLeft.getCurrentPosition(),
            backRight.getCurrentPosition()
        };
    }
    
    /**
     * Reset encoder positions to zero
     */
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Return to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

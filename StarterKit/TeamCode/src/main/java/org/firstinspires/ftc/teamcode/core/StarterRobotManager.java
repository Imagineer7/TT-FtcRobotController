/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * StarterRobotManager - Central coordination hub for robot subsystems
 * 
 * This class manages all robot subsystems (drive, shooter, etc.) and provides
 * a unified interface for TeleOp and Autonomous OpModes.
 */

package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * StarterRobotManager - Simplified robot coordinator inspired by AURORA architecture
 * 
 * Purpose:
 * - Initialize all robot subsystems with error handling
 * - Coordinate between drive and shooter systems
 * - Handle driver mode switching (single vs dual driver)
 * - Provide unified telemetry display
 * 
 * Key Features:
 * - Graceful degradation: Robot continues working even if some hardware fails
 * - Clear error messages: Helps teams debug hardware configuration issues
 * - Extensible design: Easy to add new subsystems later
 * 
 * How to Use:
 * 1. Create instance in your OpMode's runOpMode() method
 * 2. Call update(gamepad1, gamepad2) in your main loop
 * 3. Access subsystems via getDrive() and getShooter()
 */
public class StarterRobotManager {
    
    // Driver mode configuration
    public enum DriverMode {
        SINGLE_DRIVER,    // Gamepad1 controls everything (recommended for beginners)
        DUAL_DRIVER       // Gamepad1: drive, Gamepad2: shooter (recommended for competition)
    }
    
    // Subsystems
    private StarterDrive driveSystem;
    private StarterShooter shooterSystem;
    
    // Hardware references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    
    // System state
    private boolean driveInitialized = false;
    private boolean shooterInitialized = false;
    private DriverMode currentDriverMode = DriverMode.SINGLE_DRIVER;
    
    /**
     * Initialize the robot manager and all subsystems
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     */
    public StarterRobotManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        
        initializeSystems();
    }
    
    /**
     * Initialize all subsystems with graceful error handling
     * 
     * Why graceful error handling?
     * - Helps teams identify which specific hardware is misconfigured
     * - Allows partial functionality for testing (e.g., drive without shooter)
     * - Prevents complete OpMode crashes during development
     */
    private void initializeSystems() {
        telemetry.addLine("Initializing Robot Systems...");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Initialize drive system
        try {
            driveSystem = new StarterDrive(
                hardwareMap, 
                StarterDrive.DriveType.MECANUM  // Change to TANK if using tank drive
            );
            driveInitialized = true;
            telemetry.addLine("✅ Drive system initialized");
        } catch (Exception e) {
            driveSystem = null;
            driveInitialized = false;
            telemetry.addLine("❌ Drive system FAILED: " + e.getMessage());
            telemetry.addLine("   Check hardware config:");
            telemetry.addLine("   - frontLeft, frontRight, backLeft, backRight");
        }
        
        // Initialize shooter system
        try {
            shooterSystem = new StarterShooter(hardwareMap);
            shooterInitialized = true;
            telemetry.addLine("✅ Shooter system initialized");
        } catch (Exception e) {
            shooterSystem = null;
            shooterInitialized = false;
            telemetry.addLine("❌ Shooter system FAILED: " + e.getMessage());
            telemetry.addLine("   Check hardware config:");
            telemetry.addLine("   - shooter (motor), feedServo (servo)");
        }
        
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Summary
        if (driveInitialized && shooterInitialized) {
            telemetry.addLine("🎉 All systems operational!");
        } else if (!driveInitialized && !shooterInitialized) {
            telemetry.addLine("⚠️  No systems initialized - check hardware config");
        } else {
            telemetry.addLine("⚠️  Partial functionality - some systems failed");
        }
        
        telemetry.update();
    }
    
    /**
     * Main update loop - call this every cycle in your OpMode
     * 
     * @param gamepad1 Driver gamepad
     * @param gamepad2 Operator gamepad (only used in dual driver mode)
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        // Update drive system
        if (driveInitialized && driveSystem != null) {
            // In single driver mode, gamepad1 controls drive
            // In dual driver mode, gamepad1 still controls drive
            double axial = -gamepad1.left_stick_y;   // Forward/backward (reversed because Y is inverted)
            double lateral = gamepad1.left_stick_x;  // Strafe left/right
            double yaw = gamepad1.right_stick_x;     // Rotation
            
            driveSystem.drive(axial, lateral, yaw);
        }
        
        // Update shooter system
        if (shooterInitialized && shooterSystem != null) {
            // Choose which gamepad controls shooter based on driver mode
            Gamepad shooterGamepad = (currentDriverMode == DriverMode.DUAL_DRIVER) ? gamepad2 : gamepad1;
            
            // Shooter controls (these are handled inside StarterShooter.update())
            shooterSystem.update(shooterGamepad);
        }
    }
    
    /**
     * Stop all subsystems - call this when OpMode ends
     */
    public void stop() {
        if (driveSystem != null) {
            driveSystem.stop();
        }
        if (shooterSystem != null) {
            shooterSystem.stop();
        }
    }
    
    /**
     * Get the drive subsystem (for advanced control or telemetry)
     */
    public StarterDrive getDrive() {
        return driveSystem;
    }
    
    /**
     * Get the shooter subsystem (for advanced control or telemetry)
     */
    public StarterShooter getShooter() {
        return shooterSystem;
    }
    
    /**
     * Set driver mode (single vs dual driver)
     */
    public void setDriverMode(DriverMode mode) {
        this.currentDriverMode = mode;
    }
    
    /**
     * Get current driver mode
     */
    public DriverMode getDriverMode() {
        return currentDriverMode;
    }
    
    /**
     * Check if all systems are healthy
     */
    public boolean isHealthy() {
        return driveInitialized && shooterInitialized;
    }
    
    /**
     * Check if drive system is initialized
     */
    public boolean isDriveInitialized() {
        return driveInitialized;
    }
    
    /**
     * Check if shooter system is initialized
     */
    public boolean isShooterInitialized() {
        return shooterInitialized;
    }
}

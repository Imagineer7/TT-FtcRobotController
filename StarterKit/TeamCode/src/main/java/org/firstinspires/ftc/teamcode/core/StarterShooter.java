/* Copyright (c) 2025 FTC DECODE Starter Kit. All rights reserved.
 *
 * StarterShooter - Simple flywheel shooter for launching ARTIFACTS
 * 
 * This class controls a flywheel motor and feed servo to shoot 5-inch
 * ARTIFACTS into the GOAL structure during DECODE season.
 */

package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * StarterShooter - Simplified shooter system for DECODE game
 * 
 * Purpose:
 * - Launch preloaded ARTIFACTS into the GOAL
 * - Control flywheel speed with presets
 * - Simple one-button firing
 * 
 * Hardware Requirements:
 * Your robot hardware config must have:
 * - "shooter" - DC motor with encoder (flywheel)
 * - "feedServo" - Continuous rotation servo (feeds artifacts)
 * 
 * Shooting Presets:
 * - SHORT_RANGE: Close shots, fast cycling (100% power, ~2780 RPM)
 * - SAFE_TEST: Testing mode, slower speed (50% power, ~1390 RPM)
 * 
 * How to Use:
 * 1. Call spinUp() to start the flywheel
 * 2. Wait until isAtSpeed() returns true
 * 3. Press fire button (handled in update())
 * 4. Robot feeds one artifact automatically
 */
public class StarterShooter {
    
    // Shooting presets (simplified from EnhancedDecodeHelper)
    public enum ShootingPreset {
        SHORT_RANGE(1.0, 2780, "Short Range - Fast shooting for close goals"),
        SAFE_TEST(0.5, 1390, "Safe Test - Slow speed for testing");
        
        private final double power;
        private final double targetRPM;
        private final String description;
        
        ShootingPreset(double power, double targetRPM, String description) {
            this.power = power;
            this.targetRPM = targetRPM;
            this.description = description;
        }
        
        public double getPower() { return power; }
        public double getTargetRPM() { return targetRPM; }
        public String getDescription() { return description; }
    }
    
    // Hardware
    private DcMotor shooter;
    private CRServo feedServo;
    
    // State management
    private ShootingPreset currentPreset = ShootingPreset.SHORT_RANGE;
    private boolean shooterRunning = false;
    private boolean isFiring = false;
    private ElapsedTime fireTimer = new ElapsedTime();
    private static final double FIRE_DURATION = 0.3;  // Feed servo runs for 300ms
    
    // RPM tracking
    private int lastEncoderPosition = 0;
    private double lastRPMCheckTime = 0;
    private double currentRPM = 0;
    private static final double COUNTS_PER_REV = 28.0;  // Typical for many FTC motors
    private static final double RPM_CHECK_INTERVAL = 0.1;  // Check RPM every 100ms
    
    // Button state tracking (for edge detection)
    private boolean prevSpinButton = false;
    private boolean prevFireButton = false;
    
    /**
     * Constructor - initializes shooter hardware
     * 
     * @param hardwareMap Hardware map from OpMode
     * @throws IllegalArgumentException if shooter or feedServo not found
     */
    public StarterShooter(HardwareMap hardwareMap) {
        // Get hardware from config
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        feedServo = hardwareMap.get(CRServo.class, "feedServo");
        
        configureShooter();
    }
    
    /**
     * Configure shooter motor with optimal settings
     * 
     * Why these settings?
     * - RUN_USING_ENCODER: Consistent speed even as battery drains
     * - FLOAT: Flywheel spins freely when power = 0 (no braking resistance)
     * - Direction: Adjust if your flywheel spins the wrong way
     */
    private void configureShooter() {
        // Use encoder for speed control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Float mode - let flywheel coast when stopped
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set direction (adjust if needed)
        shooter.setDirection(DcMotor.Direction.FORWARD);
        
        // Initialize timer
        fireTimer.reset();
        lastRPMCheckTime = fireTimer.seconds();
        lastEncoderPosition = shooter.getCurrentPosition();
    }
    
    /**
     * Main update loop - call this every cycle
     * 
     * Handles:
     * - RPM calculations
     * - Button press detection
     * - Automatic feed servo timing
     * 
     * @param gamepad Gamepad controlling shooter (gamepad1 or gamepad2)
     */
    public void update(Gamepad gamepad) {
        // Update RPM calculation
        updateRPM();
        
        // Button controls (edge detection - only trigger on press, not hold)
        boolean spinButton = gamepad.y;     // Y button: spin up/stop
        boolean fireButton = gamepad.a;     // A button: fire artifact
        
        // Spin up/stop (toggle on button press)
        if (spinButton && !prevSpinButton) {
            if (shooterRunning) {
                stop();
            } else {
                spinUp();
            }
        }
        prevSpinButton = spinButton;
        
        // Fire (only if spinning and button just pressed)
        if (fireButton && !prevFireButton && shooterRunning && !isFiring) {
            fireOnce();
        }
        prevFireButton = fireButton;
        
        // Handle feed servo timing
        if (isFiring) {
            if (fireTimer.seconds() < FIRE_DURATION) {
                // Still feeding
                feedServo.setPower(1.0);  // Full speed forward
            } else {
                // Feeding complete
                feedServo.setPower(0);
                isFiring = false;
            }
        }
    }
    
    /**
     * Calculate current RPM from encoder
     * 
     * RPM = (encoder ticks / ticks per revolution) / (time in minutes)
     */
    private void updateRPM() {
        double currentTime = fireTimer.seconds();
        double deltaTime = currentTime - lastRPMCheckTime;
        
        // Only update at intervals to avoid noise
        if (deltaTime >= RPM_CHECK_INTERVAL) {
            int currentPosition = shooter.getCurrentPosition();
            int deltaTicks = currentPosition - lastEncoderPosition;
            
            // Calculate RPM
            double revolutions = deltaTicks / COUNTS_PER_REV;
            double minutes = deltaTime / 60.0;
            currentRPM = revolutions / minutes;
            
            // Update for next calculation
            lastEncoderPosition = currentPosition;
            lastRPMCheckTime = currentTime;
        }
    }
    
    /**
     * Spin up the flywheel to current preset speed
     */
    public void spinUp() {
        shooter.setPower(currentPreset.getPower());
        shooterRunning = true;
    }
    
    /**
     * Stop the flywheel
     */
    public void stop() {
        shooter.setPower(0);
        feedServo.setPower(0);
        shooterRunning = false;
        isFiring = false;
    }
    
    /**
     * Fire one artifact
     * 
     * This starts the feed servo, which will automatically stop after FIRE_DURATION
     */
    public void fireOnce() {
        if (!shooterRunning) {
            return;  // Can't fire if not spinning
        }
        
        isFiring = true;
        fireTimer.reset();
        feedServo.setPower(1.0);
    }
    
    /**
     * Check if flywheel is at target speed
     * 
     * @return true if RPM is within 10% of target
     */
    public boolean isAtSpeed() {
        if (!shooterRunning) {
            return false;
        }
        
        double targetRPM = currentPreset.getTargetRPM();
        double tolerance = targetRPM * 0.1;  // 10% tolerance
        
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }
    
    /**
     * Change shooting preset
     * 
     * @param preset New preset to use
     */
    public void setPreset(ShootingPreset preset) {
        this.currentPreset = preset;
        
        // If already running, update power immediately
        if (shooterRunning) {
            shooter.setPower(preset.getPower());
        }
    }
    
    /**
     * Get current RPM
     */
    public double getCurrentRPM() {
        return currentRPM;
    }
    
    /**
     * Get target RPM for current preset
     */
    public double getTargetRPM() {
        return currentPreset.getTargetRPM();
    }
    
    /**
     * Get current preset
     */
    public ShootingPreset getCurrentPreset() {
        return currentPreset;
    }
    
    /**
     * Check if shooter is running
     */
    public boolean isRunning() {
        return shooterRunning;
    }
    
    /**
     * Check if currently firing
     */
    public boolean isFiring() {
        return isFiring;
    }
    
    /**
     * Get shooter state as string (for telemetry)
     */
    public String getStateString() {
        if (!shooterRunning) {
            return "Idle";
        } else if (isFiring) {
            return "Firing";
        } else if (isAtSpeed()) {
            return "Ready";
        } else {
            return "Spinning Up";
        }
    }
}

/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Enhanced DECODE Helper with advanced features and monitoring
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * EnhancedDecodeHelper - Next-generation shooter control system
 *
 * New Features:
 * - Configurable shooting presets
 * - Performance monitoring and analytics
 * - Battery voltage compensation
 * - Predictive maintenance alerts
 * - Advanced safety systems
 * - Machine learning shot optimization (future)
 */
public class EnhancedDecodeHelper {

    // Hardware components
    private DcMotor shooter;
    private CRServo feedServo1;
    private CRServo feedServo2;
    private VoltageSensor voltageSensor;

    // Enhanced systems
    private ShooterConfig config;
    private PerformanceMonitor monitor;
    private ElapsedTime clock;

    // State management
    private boolean shooterRunning = false;
    private boolean isShooting = false;
    private double lastShotTime = 0;
    private double shooterStartTime = Double.NEGATIVE_INFINITY;
    private double feedStartTime = Double.NEGATIVE_INFINITY;
    private boolean prevButtonState = false;

    // RPM tracking
    private int lastEncoderPosition = 0;
    private double lastRpmCheckTime = 0;
    private double currentRPM = 0;
    private boolean rpmIsStable = false;
    private double lastStableRpmTime = 0;
    private static final double COUNTS_PER_REV = 28.0;

    // Safety systems
    private boolean emergencyStop = false;
    private int consecutiveFailures = 0;
    private static final int MAX_FAILURES = 3;

    /**
     * Constructor with enhanced initialization
     */
    public EnhancedDecodeHelper(HardwareMap hardwareMap) {
        // Initialize hardware
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        feedServo1 = hardwareMap.get(CRServo.class, "servo1");
        feedServo2 = hardwareMap.get(CRServo.class, "servo2");

        // Try to get voltage sensor (may not exist on all robots)
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Configure shooter motor
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize enhanced systems
        config = new ShooterConfig();
        monitor = new PerformanceMonitor();
        clock = new ElapsedTime();

        reset();
    }

    /**
     * Enhanced shooting with performance tracking and safety checks
     */
    public boolean handleShootButton(boolean buttonPressed, ShooterConfig.ShooterPreset preset) {
        // Update performance monitoring
        monitor.updateLoopTiming();

        // Record battery voltage if available
        if (voltageSensor != null) {
            monitor.recordVoltage(voltageSensor.getVoltage());
        }

        // Safety check
        if (emergencyStop || monitor.isPerformanceDegraded()) {
            if (shooterRunning) {
                stopShooter();
            }
            return false;
        }

        // Apply preset configuration
        config.setPreset(preset);

        boolean shotFired = false;

        if (buttonPressed && !prevButtonState) {
            // Button press edge
            if (!shooterRunning) {
                startShooter();
            }
        } else if (!buttonPressed && prevButtonState) {
            // Button release edge
            stopShooter();
            if (isShooting) {
                stopFeedServos();
                isShooting = false;
            }
        }

        // Handle shooting logic
        if (isShooting) {
            shotFired = fireSingleShot();
        } else if (buttonPressed && isShooterReady()) {
            shotFired = fireSingleShot();
        }

        prevButtonState = buttonPressed;
        return shotFired;
    }

    /**
     * Start shooter with voltage compensation
     */
    public void startShooter() {
        double batteryVoltage = voltageSensor != null ? voltageSensor.getVoltage() : 12.0;
        double compensatedPower = config.getPower(batteryVoltage);

        shooter.setPower(compensatedPower);
        shooterRunning = true;
        shooterStartTime = clock.seconds();

        // Reset RPM tracking
        lastEncoderPosition = shooter.getCurrentPosition();
        lastRpmCheckTime = shooterStartTime;
        currentRPM = 0;
        rpmIsStable = false;
    }

    /**
     * Enhanced RPM calculation with performance tracking
     */
    private void updateRPM() {
        if (!shooterRunning) {
            currentRPM = 0;
            rpmIsStable = false;
            return;
        }

        double currentTime = clock.seconds();
        int currentPosition = shooter.getCurrentPosition();

        if (currentTime - lastRpmCheckTime >= 0.1) {
            double deltaTime = currentTime - lastRpmCheckTime;
            int deltaPosition = currentPosition - lastEncoderPosition;

            currentRPM = Math.abs((deltaPosition / deltaTime) * 60.0 / COUNTS_PER_REV);

            // Record RPM for performance monitoring
            monitor.recordRPM(currentRPM);

            lastRpmCheckTime = currentTime;
            lastEncoderPosition = currentPosition;

            // Check stability
            double targetRpm = config.getTargetRPM();
            boolean withinTolerance = Math.abs(currentRPM - targetRpm) <= config.getRpmTolerance();

            if (withinTolerance) {
                if (!rpmIsStable) {
                    lastStableRpmTime = currentTime;
                    rpmIsStable = true;
                }
            } else {
                rpmIsStable = false;
            }
        }
    }

    /**
     * Enhanced readiness check with failure detection
     */
    public boolean isShooterReady() {
        double currentTime = clock.seconds();
        double shotInterval = config.getShotInterval();

        boolean intervalReady = (currentTime - lastShotTime >= shotInterval);
        boolean spinupReady;

        if (config.isUseRpmSpinup()) {
            spinupReady = isRPMReady();

            // Detect spinup failures
            if (shooterRunning && (currentTime - shooterStartTime > 5.0) && !spinupReady) {
                consecutiveFailures++;
                if (consecutiveFailures >= MAX_FAILURES) {
                    emergencyStop = true;
                }
            }
        } else {
            spinupReady = (currentTime - shooterStartTime >= config.getSpinupTime());
        }

        return shooterRunning && spinupReady && intervalReady && !emergencyStop;
    }

    /**
     * Enhanced single shot with success tracking
     */
    public boolean fireSingleShot() {
        double currentTime = clock.seconds();

        if (!isShooting && isShooterReady()) {
            startFeedServos();
            isShooting = true;
            feedStartTime = currentTime;
            lastShotTime = currentTime;

            // Record shot attempt
            double spinupTime = currentTime - shooterStartTime;
            monitor.recordShotAttempt(true, spinupTime); // Assume success for now
            consecutiveFailures = 0; // Reset failure count on successful shot

            return true;
        }

        if (isShooting && (currentTime - feedStartTime >= config.getFeedTime())) {
            stopFeedServos();
            isShooting = false;
        }

        return false;
    }

    /**
     * Smart autonomous shooting with adaptive timing
     */
    public void autoShootSmart(int numShots, boolean keepShooterRunning, ShooterConfig.ShooterPreset preset) {
        if (numShots <= 0) return;

        config.setPreset(preset);
        if (!shooterRunning) startShooter();

        int shotsFired = 0;
        double adaptiveInterval = config.getShotInterval();

        // Wait for spinup
        while (!isShooterReady() && !emergencyStop) {
            updateRPM();

            if (monitor.isPerformanceDegraded()) {
                adaptiveInterval *= 1.2; // Increase interval if performance is poor
            }
        }

        // Fire shots with adaptive timing
        while (shotsFired < numShots && !emergencyStop) {
            if (fireSingleShot()) {
                shotsFired++;

                // Adaptive timing based on performance
                if (monitor.getAccuracy() < 80) {
                    adaptiveInterval *= 1.1; // Slow down if accuracy is poor
                } else if (monitor.getAccuracy() > 95) {
                    adaptiveInterval *= 0.95; // Speed up if accuracy is excellent
                }
            }

            // Wait for next shot with performance monitoring
            double waitStart = clock.seconds();
            while (clock.seconds() - waitStart < adaptiveInterval && shotsFired < numShots) {
                updateRPM();
                monitor.updateLoopTiming();
            }
        }

        if (!keepShooterRunning) {
            stopShooter();
        }

        // Note: Performance reporting is now handled by SmartTelemetryManager
        // through structured data access rather than direct telemetry calls
    }

    /**
     * Emergency stop system
     */
    public void emergencyStop() {
        emergencyStop = true;
        stopShooter();
        stopFeedServos();
        isShooting = false;
    }

    /**
     * Reset emergency stop (call after fixing issues)
     */
    public void resetEmergencyStop() {
        emergencyStop = false;
        consecutiveFailures = 0;
    }

    // Helper methods
    private boolean isRPMReady() {
        updateRPM();
        double currentTime = clock.seconds();

        if (currentTime - shooterStartTime > 5.0) { // Timeout fallback
            return true;
        }

        return rpmIsStable && (currentTime - lastStableRpmTime >= 0.25);
    }

    private void startFeedServos() {
        feedServo1.setPower(-config.getFeedPower());
        feedServo2.setPower(config.getFeedPower());
    }

    private void stopFeedServos() {
        feedServo1.setPower(0.0);
        feedServo2.setPower(0.0);
    }

    public void stopShooter() {
        shooter.setPower(0.0);
        shooterRunning = false;
    }

    public void reset() {
        stopShooter();
        stopFeedServos();
        isShooting = false;
        lastShotTime = 0;
        prevButtonState = false;
        emergencyStop = false;
        consecutiveFailures = 0;
        monitor.reset();
        clock.reset();
    }

    // Getters for monitoring and configuration
    public ShooterConfig getConfig() { return config; }
    public PerformanceMonitor getMonitor() { return monitor; }
    public boolean isEmergencyStop() { return emergencyStop; }
    public double getCurrentRPM() { updateRPM(); return currentRPM; }
    public boolean isShooterRunning() { return shooterRunning; }
    public boolean isShooting() { return isShooting; }
}

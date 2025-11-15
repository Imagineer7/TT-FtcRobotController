/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Enhanced DECODE Helper with advanced features and monitoring
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    private Servo light;
    private VoltageSensor voltageSensor;

    // Enhanced systems
    private ShooterConfig config;
    private PerformanceMonitor monitor;
    private ElapsedTime clock;

    // RGB Light color constants
    private static final double LIGHT_OFF = 0.0;
    private static final double LIGHT_RED = 0.277;
    private static final double LIGHT_ORANGE = 0.333;
    private static final double LIGHT_YELLOW = 0.388;
    private static final double LIGHT_SAGE = 0.444;
    private static final double LIGHT_GREEN = 0.5;
    private static final double LIGHT_BLUE = 0.611;
    private static final double LIGHT_INDIGO = 0.666;
    private static final double LIGHT_VIOLET = 0.722;
    private static final double LIGHT_WHITE = 1.0;

    // Light flash control
    private double lastFlashTime = 0;
    private boolean flashState = false;
    private static final double FLASH_INTERVAL = 0.3; // 300ms flash interval

    // State management
    private boolean shooterRunning = false;
    private boolean isShooting = false;
    private boolean warmupMode = false; // Track if shooter is in warmup mode
    private double warmupStartTime = Double.NEGATIVE_INFINITY;
    private double lastShotTime = 0;
    private double shooterStartTime = Double.NEGATIVE_INFINITY;
    private double feedStartTime = Double.NEGATIVE_INFINITY;
    private boolean prevButtonState = false;
    private boolean prevWarmupButtonState = false;
    private boolean firstShotFired = false; // Track if first shot has been fired

    // RPM tracking
    private int lastEncoderPosition = 0;
    private double lastRpmCheckTime = 0;
    private double currentRPM = 0;
    private boolean rpmIsStable = false;
    private double lastStableRpmTime = 0;
    private double lastRpmInRangeTime = 0; // Track when RPM returns to range after a shot
    private int consecutiveStableReadings = 0; // Count consecutive stable RPM readings
    private static final double COUNTS_PER_REV = 28.0;

    // Dynamic RPM control (Balanced for fast, stable convergence)
    private double currentPower = 0;
    private double rpmError = 0;
    private double lastRpmError = 0;
    private double rpmErrorSum = 0;
    private static final double RPM_KP = 0.00015; // Proportional gain (reduced for stability)
    private static final double RPM_KI = 0.00004; // Integral gain (reduced to prevent windup)
    private static final double RPM_KD = 0.00003; // Derivative gain (reduced for smoother response)
    private static final double MAX_POWER_ADJUSTMENT = 0.15; // Maximum power adjustment per cycle (prevents overcorrection)
    private static final double MAX_POWER_RATE = 0.08; // Maximum power change per update (smoothing)

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

        // Try to get light indicator (may not exist on all robots)
        try {
            light = hardwareMap.get(Servo.class, "light");
        } catch (Exception e) {
            light = null;
        }

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
            updateLightIndicator();
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

        // Update light indicator after all state changes
        updateLightIndicator();

        prevButtonState = buttonPressed;
        return shotFired;
    }

    /**
     * Handle warmup button - spins shooter at lower RPM to save power while staying ready
     * Call this method in your teleop loop with a dedicated warmup button
     *
     * @param warmupButtonPressed - true if warmup button is pressed
     * @param preset - the shooter preset to use (determines target RPM)
     */
    public void handleWarmupButton(boolean warmupButtonPressed, ShooterConfig.ShooterPreset preset) {
        // Update performance monitoring
        monitor.updateLoopTiming();

        // Apply preset configuration
        config.setPreset(preset);

        if (warmupButtonPressed) {
            // Button is being held - start warmup if not already running
            if (!warmupMode && !isShooting) {
                startWarmup();
            }
            // Keep warmup running while button is held
            if (warmupMode && !isShooting) {
                updateWarmup();
            }
        } else {
            // Button released - stop warmup (but only if not shooting)
            if (warmupMode && !isShooting) {
                stopWarmup();
            }
        }

        // Update light indicator after all state changes
        updateLightIndicator();

        prevWarmupButtonState = warmupButtonPressed;
    }

    /**
     * Start warmup mode - spins shooter at reduced RPM
     */
    private void startWarmup() {
        double batteryVoltage = voltageSensor != null ? voltageSensor.getVoltage() : 12.0;

        // Calculate warmup target RPM (typically 65% of full target)
        double warmupTargetRpm = config.getWarmupTargetRPM();
        double feedforwardPower = calculateFeedforwardPower(warmupTargetRpm, batteryVoltage);

        currentPower = feedforwardPower;
        currentPower = Math.min(0.7, Math.max(0.0, currentPower)); // Cap at 0.7 for warmup

        shooter.setPower(currentPower);
        shooterRunning = true;
        warmupMode = true;
        warmupStartTime = clock.seconds();
        shooterStartTime = warmupStartTime;

        // Immediately set light to red to indicate spinning up
        if (light != null) {
            light.setPosition(LIGHT_RED);
        }

        // Reset RPM tracking
        lastEncoderPosition = shooter.getCurrentPosition();
        lastRpmCheckTime = warmupStartTime;
        currentRPM = 0;
        rpmIsStable = false;
        rpmError = 0;
        lastRpmError = 0;
        rpmErrorSum = 0;
    }

    /**
     * Update warmup mode - maintains warmup RPM with gentle control
     */
    private void updateWarmup() {
        updateRPM();

        // Use gentler PID control for warmup (target is warmup RPM, not full RPM)
        double warmupTargetRpm = config.getWarmupTargetRPM();
        double currentTime = clock.seconds();
        double timeSinceWarmupStart = currentTime - warmupStartTime;

        // Only adjust after initial spinup
        if (timeSinceWarmupStart > config.getWarmupSpinupTime()) {
            double error = warmupTargetRpm - currentRPM;

            // Simple proportional control for warmup (no integral/derivative needed)
            double powerAdjustment = error * RPM_KP * 0.5; // Half gain for gentle control
            powerAdjustment = Math.max(-0.05, Math.min(0.05, powerAdjustment));

            currentPower += powerAdjustment;
            currentPower = Math.max(0.0, Math.min(0.7, currentPower));

            shooter.setPower(currentPower);
        }
    }

    /**
     * Stop warmup mode
     */
    private void stopWarmup() {
        stopShooter();
        warmupMode = false;
    }

    /**
     * Start shooter with voltage compensation and dynamic RPM control
     * Now intelligently handles transition from warmup mode
     */
    public void startShooter() {
        double batteryVoltage = voltageSensor != null ? voltageSensor.getVoltage() : 12.0;
        double targetRpm = config.getTargetRPM();

        // Check if we're transitioning from warmup mode
        boolean transitioningFromWarmup = warmupMode;
        double currentRpmAtStart = currentRPM;

        if (transitioningFromWarmup) {
            // We're already spinning - calculate how much more power we need
            warmupMode = false; // Exit warmup mode

            // Calculate feedforward for full target RPM
            double feedforwardPower = calculateFeedforwardPower(targetRpm, batteryVoltage);

            // Blend current power with target power based on RPM difference
            double rpmRatio = currentRpmAtStart / targetRpm;
            if (rpmRatio > 0.5) {
                // Already spinning significantly - smooth transition
                currentPower = currentPower * 0.3 + feedforwardPower * 0.7;
            } else {
                // Low RPM - use mostly feedforward
                currentPower = feedforwardPower;
            }
        } else {
            // Starting from zero - use feedforward
            double basePower = config.getPower(batteryVoltage);
            double feedforwardPower = calculateFeedforwardPower(targetRpm, batteryVoltage);

            // Blend base power with feedforward (70% feedforward, 30% base)
            currentPower = (feedforwardPower * 0.7) + (basePower * 0.3);
        }

        currentPower = Math.min(1.0, Math.max(0.0, currentPower));

        shooter.setPower(currentPower);
        shooterRunning = true;
        shooterStartTime = clock.seconds();
        firstShotFired = false; // Reset first shot flag when shooter starts

        // Immediately set light to red to indicate spinning up
        if (light != null) {
            light.setPosition(LIGHT_RED);
        }

        // Reset or maintain RPM tracking based on transition
        if (!transitioningFromWarmup) {
            lastEncoderPosition = shooter.getCurrentPosition();
            lastRpmCheckTime = shooterStartTime;
            currentRPM = 0;
            rpmIsStable = false;
        }
        // If transitioning, keep existing RPM values for continuity

        lastRpmInRangeTime = 0;
        consecutiveStableReadings = 0;
        rpmError = 0;
        lastRpmError = 0;
        rpmErrorSum = 0;
    }

    /**
     * Enhanced RPM calculation with dynamic power adjustment
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

            // Dynamic RPM control - adjust power to reach target RPM
            adjustPowerForTargetRPM(deltaTime);

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
     * Dynamic power adjustment to reach target RPM using PID-like control
     */
    private void adjustPowerForTargetRPM(double deltaTime) {
        // Skip if in warmup mode (warmup has its own control)
        if (warmupMode) {
            return;
        }

        double targetRpm = config.getTargetRPM();
        lastRpmError = rpmError;
        rpmError = targetRpm - currentRPM;

        // Let feedforward work first - delay PID intervention
        double timeSinceStart = clock.seconds() - shooterStartTime;
        if (timeSinceStart < 0.6) {
            return; // Give feedforward more time to stabilize
        }

        // PID-like control calculation
        double proportional = rpmError * RPM_KP;

        // Integral term with adaptive windup protection
        // Reduce integral accumulation when close to target to prevent overshoot
        double integralGain = RPM_KI;
        if (Math.abs(rpmError) < 100) {
            // Near target - reduce integral contribution
            integralGain *= 0.5;
        }

        rpmErrorSum += rpmError * deltaTime;
        // Dynamic windup limit based on error magnitude
        double windupLimit = Math.max(500, 2000 - Math.abs(rpmError));
        if (Math.abs(rpmErrorSum) > windupLimit) {
            rpmErrorSum = Math.signum(rpmErrorSum) * windupLimit;
        }
        double integral = rpmErrorSum * integralGain;

        // Derivative term for damping
        double derivative = ((rpmError - lastRpmError) / deltaTime) * RPM_KD;

        // Calculate power adjustment
        double powerAdjustment = proportional + integral + derivative;

        // Limit the adjustment magnitude to prevent large jumps
        powerAdjustment = Math.max(-MAX_POWER_ADJUSTMENT,
                         Math.min(MAX_POWER_ADJUSTMENT, powerAdjustment));

        // Apply rate limiting for smooth power changes
        double targetPower = currentPower + powerAdjustment;
        double powerDelta = targetPower - currentPower;

        // Limit how fast power can change
        if (Math.abs(powerDelta) > MAX_POWER_RATE) {
            powerDelta = Math.signum(powerDelta) * MAX_POWER_RATE;
        }

        currentPower += powerDelta;

        // Ensure power stays within valid range
        currentPower = Math.max(0.0, Math.min(1.0, currentPower));

        // Update motor power
        shooter.setPower(currentPower);
    }

    /**
     * Enhanced readiness check with failure detection and RPM validation
     */
    public boolean isShooterReady() {
        double currentTime = clock.seconds();
        double shotInterval = config.getShotInterval();

        boolean intervalReady = (currentTime - lastShotTime >= shotInterval);
        boolean spinupReady;
        boolean rpmInRange = true;

        // Only check spinup time for the first shot
        if (!firstShotFired) {
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
        } else {
            // After first shot, spinup is always ready but we check RPM stability
            spinupReady = true;

            // Always validate RPM is in range for subsequent shots
            if (config.isUseRpmSpinup()) {
                updateRPM();
                double targetRpm = config.getTargetRPM();
                double rpmDifference = Math.abs(currentRPM - targetRpm);

                // Tighter preferred zone for consistency: ±25 RPM
                double preferredTolerance = 25.0;
                boolean inPreferredZone = rpmDifference <= preferredTolerance;

                // Reasonable acceptable zone: ±50 RPM
                boolean inAcceptableZone = rpmDifference <= config.getShootingRpmTolerance();

                if (inPreferredZone) {
                    // RPM is in preferred zone - short but consistent wait
                    if (lastRpmInRangeTime == 0) {
                        lastRpmInRangeTime = currentTime;
                        consecutiveStableReadings = 1;
                    } else {
                        consecutiveStableReadings++;
                    }

                    // Require 0.12s AND 2 consecutive readings for consistency
                    if ((currentTime - lastRpmInRangeTime < 0.12) || (consecutiveStableReadings < 2)) {
                        rpmInRange = false;
                    }
                } else if (inAcceptableZone) {
                    // RPM is acceptable but not ideal - require more stability
                    if (lastRpmInRangeTime == 0) {
                        lastRpmInRangeTime = currentTime;
                        consecutiveStableReadings = 1;
                    } else {
                        consecutiveStableReadings++;
                    }

                    // Require 0.2s AND 3 consecutive readings in acceptable zone
                    if ((currentTime - lastRpmInRangeTime < 0.2) || (consecutiveStableReadings < 3)) {
                        rpmInRange = false;
                    }
                } else {
                    // RPM is out of acceptable range
                    rpmInRange = false;
                    lastRpmInRangeTime = 0; // Reset stability timer
                    consecutiveStableReadings = 0;
                }
            }
        }

        return shooterRunning && spinupReady && intervalReady && rpmInRange && !emergencyStop;
    }

    /**
     * Enhanced single shot with success tracking
     */
    public boolean fireSingleShot() {
        double currentTime = clock.seconds();

        // Check if we can start a new shot
        if (!isShooting && isShooterReady()) {
            startFeedServos();
            isShooting = true;
            feedStartTime = currentTime;
            lastShotTime = currentTime;
            firstShotFired = true; // Mark that first shot has been fired

            // Reset RPM stability tracking for next shot
            lastRpmInRangeTime = 0;
            consecutiveStableReadings = 0;

            // Record shot attempt
            double spinupTime = currentTime - shooterStartTime;
            monitor.recordShotAttempt(true, spinupTime); // Assume success for now
            consecutiveFailures = 0; // Reset failure count on successful shot

            return true;
        }

        // Handle ongoing shot - stop feed servos when feed time expires
        if (isShooting && (currentTime - feedStartTime >= config.getFeedTime())) {
            stopFeedServos();
            isShooting = false;
        }

        return false;
    }

    // Smart shooting state for async operation
    private int autoShootTargetShots = 0;
    private int autoShootShotsFired = 0;
    private boolean autoShootInProgress = false;
    private double autoShootLastShotTime = 0;

    /**
     * Result class for autoShootSmart operations
     */
    public static class AutoShootResult {
        public final boolean isComplete;
        public final int shotsFired;
        public final int targetShots;
        public final boolean isWaitingForRPM;

        public AutoShootResult(boolean isComplete, int shotsFired, int targetShots, boolean isWaitingForRPM) {
            this.isComplete = isComplete;
            this.shotsFired = shotsFired;
            this.targetShots = targetShots;
            this.isWaitingForRPM = isWaitingForRPM;
        }
    }

    /**
     * Start autonomous smart shooting sequence (non-blocking)
     * Call this once to begin, then call updateAutoShootSmart() in your loop
     */
    public void startAutoShootSmart(int numShots, ShooterConfig.ShooterPreset preset) {
        if (numShots <= 0) return;

        config.setPreset(preset);
        autoShootTargetShots = numShots;
        autoShootShotsFired = 0;
        autoShootInProgress = true;
        autoShootLastShotTime = 0;
        firstShotFired = false; // Reset first shot flag to ensure proper spinup

        if (!shooterRunning) {
            startShooter();
        }
    }

    /**
     * Update autonomous shooting sequence (non-blocking)
     * Call this in your loop after starting with startAutoShootSmart()
     *
     * @return AutoShootResult containing completion status and progress
     */
    public AutoShootResult updateAutoShootSmart() {
        if (!autoShootInProgress) {
            return new AutoShootResult(true, autoShootShotsFired, autoShootTargetShots, false);
        }

        double currentTime = clock.seconds();
        boolean waitingForRPM = false;

        // Handle active shot - check if feed servos should stop
        if (isShooting && (currentTime - feedStartTime >= config.getFeedTime())) {
            stopFeedServos();
            isShooting = false;
            lastShotTime = currentTime;
            // Reset RPM stability tracking for next shot
            lastRpmInRangeTime = 0;
            consecutiveStableReadings = 0;
        }

        // Update RPM control
        updateRPM();
        monitor.updateLoopTiming();
        updateLightIndicator();

        // Check if we can fire the next shot
        if (!isShooting && autoShootShotsFired < autoShootTargetShots) {
            // Check if shooter is ready for next shot
            if (isShooterReady()) {
                // Fire next shot
                startFeedServos();
                isShooting = true;
                feedStartTime = currentTime;
                autoShootShotsFired++;
                firstShotFired = true;

                // Record shot attempt
                double spinupTime = currentTime - shooterStartTime;
                monitor.recordShotAttempt(true, spinupTime);
                consecutiveFailures = 0;
            } else {
                waitingForRPM = true;
            }
        }

        // Check if complete - all shots fired AND feed servos have finished
        if (autoShootShotsFired >= autoShootTargetShots && !isShooting) {
            autoShootInProgress = false;
            return new AutoShootResult(true, autoShootShotsFired, autoShootTargetShots, false);
        }

        return new AutoShootResult(false, autoShootShotsFired, autoShootTargetShots, waitingForRPM);
    }

    /**
     * Smart autonomous shooting with RPM-based timing (blocking version)
     * Uses preset shot interval and maintains RPM between shots
     */
    public void autoShootSmart(int numShots, boolean keepShooterRunning, ShooterConfig.ShooterPreset preset) {
        if (numShots <= 0) return;

        config.setPreset(preset);
        if (!shooterRunning) startShooter();

        int shotsFired = 0;
        double shotInterval = config.getShotInterval();

        // Wait for initial spinup
        while (!isShooterReady() && !emergencyStop) {
            updateRPM();
            monitor.updateLoopTiming();
            updateLightIndicator();
        }

        // Fire shots with consistent timing
        while (shotsFired < numShots && !emergencyStop) {
            if (fireSingleShot()) {
                shotsFired++;
            }

            // Wait for next shot while maintaining RPM with PID control
            double waitStart = clock.seconds();
            while (clock.seconds() - waitStart < shotInterval && shotsFired < numShots) {
                updateRPM(); // Continuously maintain target RPM between shots
                monitor.updateLoopTiming();
                updateLightIndicator();
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

    /**
     * Calculate feedforward power based on target RPM
     * This provides a better starting point than the preset power
     */
    private double calculateFeedforwardPower(double targetRpm, double batteryVoltage) {
        // Empirical relationship: RPM ≈ power * voltage * constant
        // Solve for power: power = RPM / (voltage * constant)

        // Calibration constant (adjust based on your motor/wheel)
        // For typical shooter: ~5000 RPM at full power (1.0) and 12V
        double rpmPerVoltPerPower = 416.67; // 5000 / (12 * 1.0)

        // Calculate required power
        double voltage = Math.max(batteryVoltage, 10.5);
        double estimatedPower = targetRpm / (voltage * rpmPerVoltPerPower);

        // Clamp to valid range
        return Math.min(1.0, Math.max(0.3, estimatedPower));
    }

    private void startFeedServos() {
        feedServo1.setPower(-config.getFeedPower());
        feedServo2.setPower(config.getFeedPower());
    }

    public void stopFeedServos() {
        feedServo1.setPower(0.0);
        feedServo2.setPower(0.0);
    }

    public void stopShooter() {
        shooter.setPower(0.0);
        shooterRunning = false;
        warmupMode = false; // Exit warmup mode
        firstShotFired = false; // Reset first shot flag when shooter stops
    }

    public void reset() {
        stopShooter();
        stopFeedServos();
        isShooting = false;
        warmupMode = false;
        lastShotTime = 0;
        prevButtonState = false;
        prevWarmupButtonState = false;
        firstShotFired = false;
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
    public boolean isWarmupMode() { return warmupMode; }
    public double getTargetRPM() { return warmupMode ? config.getWarmupTargetRPM() : config.getTargetRPM(); }

    /**
     * Check if RPM is within shooting tolerance
     * Useful for telemetry and diagnostics
     */
    public boolean isRPMInShootingRange() {
        if (!shooterRunning || !config.isUseRpmSpinup()) {
            return true; // Not using RPM control
        }
        updateRPM();
        double targetRpm = config.getTargetRPM();
        double rpmDifference = Math.abs(currentRPM - targetRpm);
        return rpmDifference <= config.getShootingRpmTolerance();
    }

    /**
     * Update RGB light indicator based on system state
     * Should be called regularly (in telemetry update or main loop)
     */
    public void updateLightIndicator() {
        if (light == null) return;

        double currentTime = clock.seconds();

        // Check for failure conditions first (highest priority)
        if (emergencyStop || consecutiveFailures >= MAX_FAILURES) {
            // Flashing red for critical failure
            setFlashingLight(LIGHT_RED, currentTime);
            return;
        }

        // Check for low voltage warning (only if critically low)
        if (voltageSensor != null && voltageSensor.getVoltage() < 10.5) {
            // Flashing yellow/red for low voltage
            if (currentTime - lastFlashTime >= FLASH_INTERVAL) {
                flashState = !flashState;
                lastFlashTime = currentTime;
            }
            light.setPosition(flashState ? LIGHT_YELLOW : LIGHT_RED);
            return;
        }

        // Normal operation states - check isShooting first (highest priority)
        if (isShooting) {
            // Yellow when actively firing
            light.setPosition(LIGHT_YELLOW);
            return;
        }

        // Not shooting - check if shooter is running
        if (!shooterRunning) {
            // Blue for normal operation (idle)
            light.setPosition(LIGHT_BLUE);
            return;
        }

        // Shooter is running - determine if ready or spinning up
        // Show RED immediately when motor starts, don't wait for RPM measurement
        double timeSinceStart = currentTime - shooterStartTime;
        boolean lightIsReady = false;

        if (config.isUseRpmSpinup()) {
            // Need to wait a bit before we have valid RPM readings
            if (timeSinceStart < 0.15) {
                // Too early to have RPM data - definitely still spinning up (show red)
                lightIsReady = false;
            } else {
                // We have RPM data - check if in range
                double targetRpm = warmupMode ? config.getWarmupTargetRPM() : config.getTargetRPM();
                double rpmDifference = Math.abs(currentRPM - targetRpm);
                double tolerance = config.getRpmTolerance();

                // Ready when: RPM is close to target AND has been running long enough
                lightIsReady = (rpmDifference <= tolerance) && (timeSinceStart > 0.5);
            }
        } else {
            // Time-based spinup
            lightIsReady = (timeSinceStart >= config.getSpinupTime());
        }

        if (lightIsReady) {
            // Green when ready to fire
            light.setPosition(LIGHT_GREEN);
        } else {
            // Red when spinning up (motor powered but not at target RPM)
            light.setPosition(LIGHT_RED);
        }
    }

    /**
     * Helper method for flashing light patterns
     */
    private void setFlashingLight(double color, double currentTime) {
        if (currentTime - lastFlashTime >= FLASH_INTERVAL) {
            flashState = !flashState;
            lastFlashTime = currentTime;
        }
        light.setPosition(flashState ? color : LIGHT_OFF);
    }

    /**
     * Manually set light color (for testing or special modes)
     */
    public void setLightColor(double color) {
        if (light != null) {
            light.setPosition(color);
        }
    }

    /**
     * Get current light color
     */
    public double getLightColor() {
        if (light != null) {
            return light.getPosition();
        }
        return LIGHT_OFF;
    }

    /**
     * Turn light off
     */
    public void turnLightOff() {
        if (light != null) {
            light.setPosition(LIGHT_OFF);
        }
    }

    /**
     * Get diagnostic info for light indicator troubleshooting
     */
    public String getLightDiagnostics() {
        if (light == null) return "Light: NULL";

        double currentTime = clock.seconds();
        double timeSinceStart = currentTime - shooterStartTime;

        return String.format("Light: %.3f | Running: %b | Shooting: %b | TimeSince: %.2fs | RPM: %.0f | Ready: %b",
                light.getPosition(), shooterRunning, isShooting, timeSinceStart, currentRPM,
                shooterRunning && (timeSinceStart >= 0.15));
    }
}

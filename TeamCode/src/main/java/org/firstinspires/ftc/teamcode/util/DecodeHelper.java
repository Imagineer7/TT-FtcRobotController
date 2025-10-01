/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * DECODE Helper Class for artifact launching and collection functionality
 * This class provides methods for teleop button integration and autonomous operation
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AutoHelper;

/**
 * DecodeHelper - A utility class for DECODE season artifact launching
 * 
 * Features:
 * - Single shot launching for teleop button presses
 * - Continuous shooting when button held down
 * - Auto-compatible methods for autonomous operations
 * - State management for proper timing and sequencing
 */
public class DecodeHelper {
    
    // Hardware components
    private DcMotor shooter;
    private CRServo feedServo1;
    private CRServo feedServo2;
    
    // Timing and state management
    private ElapsedTime timer;
    private Telemetry telemetry;
    
    // Configuration constants (optimized for RS-555 motor: 6,000 RPM no-load)
    private static final double SHOOTER_POWER = 0.75;    // Long range: ~4500 RPM (75% of 6000 RPM)
    private static final double SHORT_SHOOTER_POWER = 0.55; // Short range: ~3300 RPM (55% of 6000 RPM)
    private static final double FEED_POWER = 1.0;
    private static final double FEED_TIME = 0.3; // Time to run feed servos for one shot
    private static final double SHOT_INTERVAL_LONG = 1.5; // Minimum time between long range shots
    private static final double SHOT_INTERVAL_SHORT = 1.2; // Minimum time between short range shots (faster for closer shots)
    private static final double SHOOTER_SPINUP_TIME = 2.0; // Time for shooter to reach speed
    
    // State variables
    private boolean shooterRunning = false;
    private boolean isShooting = false;
    private double lastShotTime = 0;
    private boolean prevButtonState = false;
    private boolean longRange = true;

    private final ElapsedTime clock = new ElapsedTime(); // never reset after construction
    private double shooterStartTime = Double.NEGATIVE_INFINITY;
    private double feedStartTime   = Double.NEGATIVE_INFINITY;
    
    // RPM tracking variables  
    private int lastEncoderPosition = 0;
    private double lastRpmCheckTime = 0;
    private double currentRPM = 0;
    private double lastStableRpmTime = 0;
    private boolean rpmIsStable = false;
    private static final double COUNTS_PER_REV = 28.0; // RS-555 motor: 28 PPR (Pulses Per Revolution) at output shaft

    // Mode-specific spinup (tune these!)
    private static final double SPINUP_LONG  = 2.0;
    private static final double SPINUP_SHORT = 1.2;  // adjust after testing
    
    // RPM-based smart spinup constants (based on RS-555 motor specs: 6,000 RPM no-load, 28 PPR encoder)
    private static final double TARGET_RPM_LONG = 4500;   // Target RPM for long range shots (75% of max)
    private static final double TARGET_RPM_SHORT = 3300;  // Target RPM for short range shots (55% of max)
    private static final double RPM_TOLERANCE = 75;       // RPM within this range considered "ready" (wider for high-speed motor)
    private static final double RPM_STABILITY_TIME = 0.25; // Time RPM must be stable before ready (shorter for fast motor)
    private static final double MAX_SPINUP_TIME = 3.0;    // Fallback timeout for safety (shorter for fast motor)
    private static final boolean USE_RPM_SPINUP = true;   // Enable/disable RPM-based spinup
    
    /**
     * Get the appropriate shot interval based on current range mode
     * @return shot interval in seconds for current mode
     */
    private double getCurrentShotInterval() {
        return longRange ? SHOT_INTERVAL_LONG : SHOT_INTERVAL_SHORT;
    }
    
    /**
     * Get target RPM for current range mode
     * @return target RPM for current mode
     */
    private double getTargetRPM() {
        return longRange ? TARGET_RPM_LONG : TARGET_RPM_SHORT;
    }
    
    /**
     * Update RPM calculation and stability tracking
     * Call this regularly (every loop) when shooter is running
     */
    private void updateRPM() {
        if (!shooterRunning) {
            currentRPM = 0;
            rpmIsStable = false;
            return;
        }
        
        double currentTime = clock.seconds();
        int currentPosition = shooter.getCurrentPosition();
        
        // Calculate RPM if enough time has passed (at least 0.1 seconds for accuracy)
        if (currentTime - lastRpmCheckTime >= 0.1) {
            double deltaTime = currentTime - lastRpmCheckTime;
            int deltaPosition = currentPosition - lastEncoderPosition;
            
            // RPM = (encoder counts / time) * (60 seconds/minute) / (counts per revolution)
            currentRPM = Math.abs((deltaPosition / deltaTime) * 60.0 / COUNTS_PER_REV);
            
            // Update tracking variables
            lastRpmCheckTime = currentTime;
            lastEncoderPosition = currentPosition;
            
            // Check if RPM is stable (within tolerance of target)
            double targetRpm = getTargetRPM();
            boolean withinTolerance = Math.abs(currentRPM - targetRpm) <= RPM_TOLERANCE;
            
            if (withinTolerance) {
                if (!rpmIsStable) {
                    // Just entered stable range
                    lastStableRpmTime = currentTime;
                    rpmIsStable = true;
                }
            } else {
                // Left stable range
                rpmIsStable = false;
            }
        }
    }
    
    /**
     * Check if shooter RPM is ready for firing
     * @return true if RPM is at target and stable for required time
     */
    private boolean isRPMReady() {
        if (!USE_RPM_SPINUP || !shooterRunning) return false;
        
        updateRPM(); // Update RPM calculation
        
        double currentTime = clock.seconds();
        
        // Safety timeout - fall back to time-based if taking too long
        if (currentTime - shooterStartTime > MAX_SPINUP_TIME) {
            return true;
        }
        
        // Check if RPM is stable for required time
        return rpmIsStable && (currentTime - lastStableRpmTime >= RPM_STABILITY_TIME);
    }
    
    /**
     * Constructor - Initialize the DecodeHelper
     * @param hardwareMap The robot's hardware map
     * @param telemetry Telemetry for debugging output
     */
    public DecodeHelper(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize hardware
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        feedServo1 = hardwareMap.get(CRServo.class, "servo1");
        feedServo2 = hardwareMap.get(CRServo.class, "servo2");
        
        // Configure shooter motor
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize timing and telemetry
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        
        // Reset state
        reset();
    }
    
    /**
     * Reset all systems to initial state
     */
    public void reset() {
        stopShooter();
        stopFeedServos();
        isShooting = false;
        lastShotTime = 0;
        prevButtonState = false;
        timer.reset();
    }
    
    /**
     * Start the shooter motor
     */
    public void startShooter() {
        shooter.setPower(longRange ? SHOOTER_POWER : SHORT_SHOOTER_POWER);
        shooterRunning = true;
        shooterStartTime = clock.seconds();
        
        // Reset RPM tracking
        lastEncoderPosition = shooter.getCurrentPosition();
        lastRpmCheckTime = shooterStartTime;
        currentRPM = 0;
        rpmIsStable = false;
    }

    public void stopShooter() {
        shooter.setPower(0.0);
        shooterRunning = false;
    }
    
    /**
     * Start the feed servos to launch an artifact
     */
    private void startFeedServos() {
        feedServo1.setPower(-FEED_POWER);
        feedServo2.setPower(FEED_POWER);
    }
    
    /**
     * Stop the feed servos
     */
    private void stopFeedServos() {
        feedServo1.setPower(0.0);
        feedServo2.setPower(0.0);
    }

    public void setRangeLong(boolean enableLong) {
        if (this.longRange != enableLong) {
            this.longRange = enableLong;
            if (shooterRunning) {
                // Re-apply power and require fresh spin-up for the new mode
                shooter.setPower(longRange ? SHOOTER_POWER : SHORT_SHOOTER_POWER);
                shooterStartTime = clock.seconds();
                
                // Reset RPM tracking for new target
                lastEncoderPosition = shooter.getCurrentPosition();
                lastRpmCheckTime = shooterStartTime;
                currentRPM = 0;
                rpmIsStable = false;
            }
        } else {
            this.longRange = enableLong;
        }
    }
    
    /**
     * Check if shooter is at speed and ready to fire
     * @return true if shooter has been running long enough to be at speed
     */
    public boolean isShooterReady() {
        double t = clock.seconds();
        double shotInterval = getCurrentShotInterval();
        
        // Check shot interval timing
        boolean intervalReady = (t - lastShotTime >= shotInterval);
        
        // Use RPM-based spinup if enabled, otherwise fall back to time-based
        boolean spinupReady;
        if (USE_RPM_SPINUP) {
            spinupReady = isRPMReady();
        } else {
            double spinup = longRange ? SPINUP_LONG : SPINUP_SHORT;
            spinupReady = (t - shooterStartTime >= spinup);
        }
        
        return shooterRunning && spinupReady && intervalReady;
    }
    
    /**
     * Fire a single shot (non-blocking)
     * Call this method repeatedly in your main loop
     * @return true if a shot was fired this cycle
     */
    public boolean fireSingleShot() {
        double t = clock.seconds();

        if (!isShooting && isShooterReady()) {
            startFeedServos();
            isShooting = true;
            feedStartTime = t;
            lastShotTime  = t;
            return true;
        }
        if (isShooting && (t - feedStartTime >= FEED_TIME)) {
            stopFeedServos();
            isShooting = false;
        }
        return false;
    }
    
    /**
     * Handle teleop button press for shooting
     * This method manages button state and provides single-shot or continuous shooting
     * Call this in your teleop loop with the current button state
     * 
     * @param buttonPressed Current state of the shoot button
     * @return true if a shot was initiated this cycle
     */
    public boolean handleShootButton(boolean buttonPressed, boolean longRangeMode) {
        setRangeLong(longRangeMode);  // <- applies power + spinup if changed while running
        boolean shotFired = false;

        if (buttonPressed && !prevButtonState) {      // press edge
            if (!shooterRunning) startShooter();      // stamps shooterStartTime
        } else if (!buttonPressed && prevButtonState) { // release edge
            stopShooter();
            if (isShooting) { stopFeedServos(); isShooting = false; }
        }

        if (isShooting) {
            fireSingleShot();                         // let it finish FEED_TIME
        } else if (buttonPressed && isShooterReady()) {
            shotFired = fireSingleShot();
        }

        prevButtonState = buttonPressed;
        return shotFired;
    }
    
    /**
     * Autonomous shooting method - fires a specified number of shots
     * This is a blocking method suitable for autonomous routines
     * 
     * @param numShots Number of shots to fire
     * @param keepShooterRunning Whether to keep shooter running after shots complete
     */
    public void autoShoot(int numShots, boolean keepShooterRunning, boolean longRangeMode) {
        if (numShots <= 0) return;
        setRangeLong(longRangeMode);
        if (!shooterRunning) startShooter();

        // spin-up using the same readiness gate
        while (!isShooterReady()) {
            telemetry.addData("Shooter", "Spinning up...");
            telemetry.update();
        }
        for (int i = 0; i < numShots; i++) {
            startFeedServos();
            double start = clock.seconds();
            while (clock.seconds() - start < FEED_TIME) {
                telemetry.addData("Feeding", "%.2fs", clock.seconds() - start);
                telemetry.update();
            }
            stopFeedServos();

            if (i < numShots - 1) {
                double w = clock.seconds();
                double shotInterval = getCurrentShotInterval();
                while (clock.seconds() - w < shotInterval) {
                    telemetry.addData("Interval", "%.2fs (%.1fs required)", clock.seconds() - w, shotInterval);
                    telemetry.update();
                }
            }
            lastShotTime = clock.seconds();
        }
        if (!keepShooterRunning) stopShooter();
        telemetry.addData("Auto Shoot", "Complete - %d shots fired", numShots);
        telemetry.update();
    }

    
    /**
     * Manual shooter control for fine-tuning
     * @param power Power level (0.0 to 1.0)
     */
    public void setShooterPower(double power) {
        shooter.setPower(power);
        shooterRunning = (power > 0);
    }
    
    /**
     * Manual feed servo control
     * @param power Power level (-1.0 to 1.0)
     */
    public void setFeedPower(double power) {
        feedServo1.setPower(power);
        feedServo2.setPower(-power);
    }
    
    /**
     * Get current shooter power
     * @return Current shooter motor power
     */
    public double getShooterPower() {
        return shooter.getPower();
    }
    
    /**
     * Check if shooter is currently running
     * @return true if shooter is running
     */
    public boolean isShooterRunning() {
        return shooterRunning;
    }
    
    /**
     * Check if currently in the process of shooting
     * @return true if feed servos are currently running
     */
    public boolean isShooting() {
        return isShooting;
    }
    
    /**
     * Get time since last shot
     * @return seconds since last shot was fired
     */
    public double getTimeSinceLastShot() {
        if(lastShotTime>timer.seconds()){
            return 0;//to prevent negative return.
        }else{
            return timer.seconds() - lastShotTime;
        }
    }
    
    /**
     * Get current shooter RPM
     * @return current measured RPM
     */
    public double getCurrentRPM() {
        updateRPM(); // Ensure RPM is current
        return currentRPM;
    }
    
    /**
     * Get target RPM for current mode
     * @return target RPM for current range mode
     */
    public double getCurrentTargetRPM() {
        return getTargetRPM();
    }
    
    /**
     * Check if RPM is stable and at target
     * @return true if RPM is stable within tolerance
     */
    public boolean isRPMStable() {
        updateRPM(); // Ensure RPM is current
        return rpmIsStable;
    }
    
    /**
     * Update telemetry with current status
     * Call this in your main loop to see DecodeHelper status
     */
    public void updateTelemetry() {
        double currentPower = getShooterPower();
        double targetPower = longRange ? SHOOTER_POWER : SHORT_SHOOTER_POWER;
        double spinupTime = longRange ? SPINUP_LONG : SPINUP_SHORT;
        double shotInterval = getCurrentShotInterval();
        
        telemetry.addData("=== SHOOTER STATUS ===", "");
        telemetry.addData("Mode", "%s RANGE", longRange ? "LONG" : "SHORT");
        telemetry.addData("Power", "%.2f / %.2f (current/target)", currentPower, targetPower);
        telemetry.addData("Shooter Ready", isShooterReady() ? "✓ READY" : "⏳ Not Ready");
        telemetry.addData("Currently Shooting", isShooting() ? "🔥 FIRING" : "⏸ Idle");
        
        telemetry.addData("=== RPM STATUS ===", "");
        if (USE_RPM_SPINUP && shooterRunning) {
            double currentRpm = getCurrentRPM();
            double targetRpm = getCurrentTargetRPM();
            telemetry.addData("RPM", "%.0f / %.0f (current/target)", currentRpm, targetRpm);
            telemetry.addData("RPM Stable", isRPMStable() ? "✓ STABLE" : "⚡ Stabilizing");
            telemetry.addData("RPM Error", "%.0f RPM", Math.abs(currentRpm - targetRpm));
            if (rpmIsStable) {
                double stableTime = clock.seconds() - lastStableRpmTime;
                telemetry.addData("Stable Time", "%.1f / %.1f sec", stableTime, RPM_STABILITY_TIME);
            }
        } else if (USE_RPM_SPINUP) {
            telemetry.addData("RPM", "Shooter stopped");
        } else {
            telemetry.addData("RPM", "Time-based spinup (RPM disabled)");
        }
        
        telemetry.addData("=== TIMING ===", "");
        telemetry.addData("Time Since Last Shot", "%.1f / %.1f sec", getTimeSinceLastShot(), shotInterval);
        if (!USE_RPM_SPINUP) {
            telemetry.addData("Spinup Required", "%.1fs", spinupTime);
        }
        
        if (shooterRunning) {
            double timeRunning = clock.seconds() - shooterStartTime;
            telemetry.addData("Shooter Running", "%.1fs", timeRunning);
            if (timeRunning < spinupTime) {
                telemetry.addData("Spinup Progress", "%.1f%% (%.1fs remaining)", 
                    (timeRunning / spinupTime) * 100, spinupTime - timeRunning);
            }
        }
    }
    
    // ========== AAF (AUTONOMOUS ACTION FRAMEWORK) INTEGRATION ==========

    public java.util.function.Supplier<Boolean> createNonBlockingShootAction(int numShots, boolean keepShooterRunning, boolean longRangeMode) {
        return new java.util.function.Supplier<Boolean>() {
            private int shotsFired = 0;
            private boolean init = false;

            @Override public Boolean get() {
                if (!init) {
                    setRangeLong(longRangeMode);
                    if (!shooterRunning) startShooter();
                    init = true;
                    return false;
                }
                if (shotsFired >= numShots) {
                    if (!keepShooterRunning) stopShooter();
                    return true;
                }
                if (!isShooting && isShooterReady()) {
                    fireSingleShot();
                    shotsFired++;
                } else if (isShooting) {
                    fireSingleShot(); // allow to stop after FEED_TIME
                }
                return false;
            }
        };
    }
    
    /**
     * Creates a Supplier<Boolean> for shooting that's compatible with AutoHelper.addStep()
     * This allows DecodeHelper shooting to be integrated into AAF autonomous sequences
     * 
     * @param numShots Number of shots to fire
     * @param keepShooterRunning Whether to keep shooter running after completion
     * @return Supplier that returns true when shooting is complete
     */
    public java.util.function.Supplier<Boolean> createShootAction(int numShots, boolean keepShooterRunning) {
        return () -> {
            autoShoot(numShots, keepShooterRunning, true); // default to long range
            return true; // autoShoot is blocking, so always returns true when complete
        };
    }
    
    /**
     * Creates a non-blocking shooting action for AAF that can be called repeatedly
     * Use this for more advanced autonomous routines where you need to do other things while shooting
     * 
     * @param numShots Number of shots to fire
     * @param keepShooterRunning Whether to keep shooter running after completion
     * @return Supplier that manages shooting state and returns true when complete
     */
    public java.util.function.Supplier<Boolean> createNonBlockingShootAction(int numShots, boolean keepShooterRunning) {
        return new java.util.function.Supplier<Boolean>() {
            private int shotsFired = 0;
            private boolean initialized = false;
            private double lastShotStartTime = 0;
            private boolean currentlyShooting = false;
            
            @Override
            public Boolean get() {
                double currentTime = timer.seconds();
                
                // Initialize on first call
                if (!initialized) {
                    if (!shooterRunning) {
                        startShooter();
                    }
                    initialized = true;
                    lastShotStartTime = currentTime - SHOOTER_SPINUP_TIME; // Allow immediate first shot
                    return false; // Not complete yet
                }
                
                // Check if we're done
                if (shotsFired >= numShots) {
                    if (!keepShooterRunning) {
                        stopShooter();
                    }
                    return true; // Complete
                }
                
                // Handle shooting timing
                double shotInterval = getCurrentShotInterval();
                if (!currentlyShooting && isShooterReady() && 
                    (currentTime - lastShotStartTime >= shotInterval)) {
                    // Start next shot
                    startFeedServos();
                    currentlyShooting = true;
                    lastShotStartTime = currentTime;
                } else if (currentlyShooting && 
                          (currentTime - lastShotStartTime >= FEED_TIME)) {
                    // Stop current shot
                    stopFeedServos();
                    currentlyShooting = false;
                    shotsFired++;
                    
                    // Update telemetry
                    telemetry.addData("Non-blocking Shot", "%d of %d complete", shotsFired, numShots);
                    telemetry.update();
                }
                
                return false; // Not complete yet
            }
        };
    }
    
    /**
     * Create a simple shooter start action for AAF
     * @return Supplier that starts the shooter and returns true
     */
    public java.util.function.Supplier<Boolean> createStartShooterAction() {
        return () -> {
            startShooter();
            return true;
        };
    }
    
    /**
     * Create a simple shooter stop action for AAF
     * @return Supplier that stops the shooter and returns true
     */
    public java.util.function.Supplier<Boolean> createStopShooterAction() {
        return () -> {
            stopShooter();
            return true;
        };
    }
    
    /**
     * Create a wait-for-shooter-ready action for AAF
     * @return Supplier that returns true when shooter is ready to fire
     */
    public java.util.function.Supplier<Boolean> createWaitForShooterReadyAction() {
        return () -> isShooterReady();
    }
    
    /**
     * Create an action that fires a single shot (non-blocking)
     * Call this repeatedly until it returns true
     * @return Supplier that manages single shot and returns true when complete
     */
    public java.util.function.Supplier<Boolean> createSingleShotAction() {
        return new java.util.function.Supplier<Boolean>() {
            private boolean shotInitiated = false;
            private double shotStartTime = 0;
            
            @Override
            public Boolean get() {
                double currentTime = timer.seconds();
                
                if (!shotInitiated && isShooterReady()) {
                    // Start the shot
                    startFeedServos();
                    shotInitiated = true;
                    shotStartTime = currentTime;
                    lastShotTime = currentTime;
                    return false; // Not complete yet
                } else if (shotInitiated && (currentTime - shotStartTime >= FEED_TIME)) {
                    // Complete the shot
                    stopFeedServos();
                    return true; // Complete
                }
                
                return shotInitiated ? false : isShooterReady(); // Wait for shooter ready if not initiated
            }
        };
    }
    
    // ========== AAF UTILITY METHODS ==========
    
    /**
     * Helper method to add shooting steps to an AutoHelper instance
     * This provides a convenient way to add common shooting patterns
     * 
     * Usage:
     * DecodeHelper decodeHelper = new DecodeHelper(hardwareMap, telemetry);
     * decodeHelper.addShootingSequence(autoHelper, 3, "Fire 3 artifacts at basket");
     * 
     * @param autoHelper The AutoHelper instance to add steps to
     * @param numShots Number of shots to fire
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addShootingSequence(
            AutoHelper autoHelper, 
            int numShots, 
            String description) {
        return autoHelper.addStep(description, createShootAction(numShots, false));
    }
    
    /**
     * Helper method to add a shooter start step to AutoHelper
     * @param autoHelper The AutoHelper instance to add steps to
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addStartShooterStep(
            AutoHelper autoHelper, 
            String description) {
        return autoHelper.addStep(description, createStartShooterAction());
    }
    
    /**
     * Helper method to add a shooter stop step to AutoHelper
     * @param autoHelper The AutoHelper instance to add steps to
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addStopShooterStep(
            AutoHelper autoHelper, 
            String description) {
        return autoHelper.addStep(description, createStopShooterAction());
    }
    
    /**
     * Helper method to add a wait-for-shooter-ready step to AutoHelper
     * @param autoHelper The AutoHelper instance to add steps to
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addWaitForShooterReadyStep(
            AutoHelper autoHelper, 
            String description) {
        return autoHelper.addStep(description, createWaitForShooterReadyAction());
    }
}
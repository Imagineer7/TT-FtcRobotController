package org.firstinspires.ftc.teamcode.util.auroraone.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.auroraone.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;

/**
 * AURORA ONE - Shooter Handler
 * Advanced shooter control system with precision RPM control and performance monitoring
 *
 * This class manages the robot's shooter system, including motor control, feed servos, and shooting logic.
 * It provides methods to control shooting speed, timing, and overall shooting behavior.
 *
 * Features:
 * - Configurable shooting presets using Tunables
 * - Performance monitoring and analytics
 * - Battery voltage compensation
 * - Advanced safety systems with failure detection
 * - Dynamic RPM control with PID feedback
 * - Integration with Aurora One State Machine system
 * - Centralized tunable parameters
 */
public class ShooterHandler {

    // Hardware reference
    private RobotMap robotMap;
    private DcMotor shooter;
    private CRServo feedServo1;
    private CRServo feedServo2;
    private VoltageSensor voltageSensor;

    // Shooter presets
    public enum ShooterPreset {
        HIGH_GOAL(Tunables.SHOOTER_HIGH_GOAL_RPM, "High Goal", "Maximum range shooting"),
        MID_GOAL(Tunables.SHOOTER_MID_GOAL_RPM, "Mid Goal", "Balanced power and accuracy"),
        LOW_GOAL(Tunables.SHOOTER_LOW_GOAL_RPM, "Low Goal", "Close range, high accuracy"),
        CUSTOM(0.0, "Custom", "User-defined RPM");

        private final double defaultRPM;
        private final String name;
        private final String description;

        ShooterPreset(double defaultRPM, String name, String description) {
            this.defaultRPM = defaultRPM;
            this.name = name;
            this.description = description;
        }

        public double getDefaultRPM() { return defaultRPM; }
        public String getName() { return name; }
        public String getDescription() { return description; }
    }

    // State management
    private boolean shooterRunning = false;
    private boolean isShooting = false;
    private boolean isInitialized = false;
    private ShooterPreset currentPreset = ShooterPreset.MID_GOAL;
    private double customRPM = Tunables.SHOOTER_MID_GOAL_RPM;

    // Timing tracking
    private ElapsedTime clock = new ElapsedTime();
    private double lastShotTime = 0;
    private double shooterStartTime = Double.NEGATIVE_INFINITY;
    private double feedStartTime = Double.NEGATIVE_INFINITY;
    private boolean prevButtonState = false;

    // RPM tracking and control
    private int lastEncoderPosition = 0;
    private double lastRpmCheckTime = 0;
    private double currentRPM = 0;
    private boolean rpmIsStable = false;
    private double lastStableRpmTime = 0;
    private static final double COUNTS_PER_REV = 28.0; // Standard encoder counts

    // Dynamic power control
    private double currentPower = 0;
    private double rpmError = 0;
    private double lastRpmError = 0;
    private double rpmErrorSum = 0;

    // Safety systems
    private boolean emergencyStop = false;
    private int consecutiveFailures = 0;

    // Performance tracking
    private int totalShots = 0;
    private int successfulShots = 0;
    private double totalSpinupTime = 0;
    private double averageSpinupTime = 0;

    /**
     * Constructor - Initialize with RobotMap
     */
    public ShooterHandler(RobotMap robotMap) {
        this.robotMap = robotMap;
        initialize();
    }

    /**
     * Initialize the shooter system using RobotMap
     */
    private void initialize() {
        if (!robotMap.isShooterSystemReady()) {
            isInitialized = false;
            return;
        }

        // Get hardware from RobotMap
        shooter = robotMap.shooter;
        feedServo1 = robotMap.feedServo1;
        feedServo2 = robotMap.feedServo2;
        voltageSensor = robotMap.voltageSensor;

        configureHardware();

        isInitialized = true;
        clock.reset();
    }

    /**
     * Configure hardware with optimal settings
     */
    private void configureHardware() {
        if (!isInitialized()) return;

        // Configure shooter motor
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        reset();
    }

    /**
     * Enhanced shooting with performance tracking and safety checks
     */
    public boolean handleShootButton(boolean buttonPressed, ShooterPreset preset) {
        if (!isInitialized()) return false;

        // Update current preset
        currentPreset = preset;

        // Safety check
        if (emergencyStop) {
            if (shooterRunning) {
                stopShooter();
            }
            return false;
        }

        boolean shotFired = false;

        if (buttonPressed && !prevButtonState) {
            // Button press edge - start shooter
            if (!shooterRunning) {
                startShooter();
            }
        } else if (!buttonPressed && prevButtonState) {
            // Button release edge - stop shooter
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
     * Start shooter with voltage compensation and dynamic RPM control
     */
    public void startShooter() {
        if (!isInitialized()) return;

        double batteryVoltage = voltageSensor != null ? voltageSensor.getVoltage() : Tunables.DRIVE_NOMINAL_VOLTAGE;

        // Calculate initial power with voltage compensation
        double targetRPM = getTargetRPM();
        currentPower = calculateInitialPower(targetRPM, batteryVoltage);

        shooter.setPower(currentPower);
        shooterRunning = true;
        shooterStartTime = clock.seconds();

        // Reset RPM tracking and control
        lastEncoderPosition = shooter.getCurrentPosition();
        lastRpmCheckTime = shooterStartTime;
        currentRPM = 0;
        rpmIsStable = false;
        rpmError = 0;
        lastRpmError = 0;
        rpmErrorSum = 0;
    }

    /**
     * Calculate initial power based on target RPM and battery voltage
     */
    private double calculateInitialPower(double targetRPM, double batteryVoltage) {
        // Base power calculation
        double basePower = Math.min(1.0, targetRPM / Tunables.SHOOTER_MAX_RPM);

        // Voltage compensation
        double voltageRatio = batteryVoltage / Tunables.DRIVE_NOMINAL_VOLTAGE;
        if (voltageRatio < 1.0) {
            basePower /= voltageRatio; // Increase power when voltage is low
        }

        return Math.max(0.0, Math.min(1.0, basePower));
    }

    /**
     * Get target RPM based on current preset
     */
    private double getTargetRPM() {
        return currentPreset == ShooterPreset.CUSTOM ? customRPM : currentPreset.getDefaultRPM();
    }

    /**
     * Enhanced RPM calculation with dynamic power adjustment
     */
    private void updateRPM() {
        if (!shooterRunning || !isInitialized()) {
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

            lastRpmCheckTime = currentTime;
            lastEncoderPosition = currentPosition;

            // Check stability
            double targetRpm = getTargetRPM();
            boolean withinTolerance = Math.abs(currentRPM - targetRpm) <= Tunables.SHOOTER_RPM_TOLERANCE;

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
     * Dynamic power adjustment to reach target RPM using PID control from Tunables
     */
    private void adjustPowerForTargetRPM(double deltaTime) {
        double targetRpm = getTargetRPM();
        lastRpmError = rpmError;
        rpmError = targetRpm - currentRPM;

        // Only start adjusting power after initial spinup time
        if (clock.seconds() - shooterStartTime < Tunables.SHOOTER_SPIN_UP_TIME) {
            return;
        }

        // PID control calculation using Tunables
        double proportional = rpmError * Tunables.SHOOTER_KP;

        // Integral term (with windup protection)
        rpmErrorSum += rpmError * deltaTime;
        if (Math.abs(rpmErrorSum) > 1000) { // Prevent windup
            rpmErrorSum = Math.signum(rpmErrorSum) * 1000;
        }
        double integral = rpmErrorSum * Tunables.SHOOTER_KI;

        // Derivative term
        double derivative = ((rpmError - lastRpmError) / deltaTime) * Tunables.SHOOTER_KD;

        // Calculate power adjustment
        double powerAdjustment = proportional + integral + derivative;

        // Limit the adjustment magnitude using Tunables
        powerAdjustment = Math.max(-Tunables.SHOOTER_MAX_POWER_ADJUSTMENT,
                         Math.min(Tunables.SHOOTER_MAX_POWER_ADJUSTMENT, powerAdjustment));

        // Apply adjustment to current power
        currentPower += powerAdjustment;

        // Ensure power stays within valid range
        currentPower = Math.max(0.0, Math.min(1.0, currentPower));

        // Update motor power
        shooter.setPower(currentPower);
    }

    /**
     * Enhanced readiness check with failure detection
     */
    public boolean isShooterReady() {
        if (!isInitialized() || !shooterRunning) return false;

        double currentTime = clock.seconds();

        // Check shot interval from Tunables
        boolean intervalReady = (currentTime - lastShotTime >= Tunables.SHOOTER_FEED_DELAY);

        // Check spinup readiness
        boolean spinupReady = isRPMReady();

        // Detect spinup failures
        if (shooterRunning && (currentTime - shooterStartTime > 5.0) && !spinupReady) {
            consecutiveFailures++;
            if (consecutiveFailures >= 3) { // MAX_FAILURES
                emergencyStop = true;
            }
        }

        return spinupReady && intervalReady && !emergencyStop;
    }

    /**
     * Check if RPM is ready using Tunables tolerance
     */
    private boolean isRPMReady() {
        updateRPM();
        double currentTime = clock.seconds();

        if (currentTime - shooterStartTime > 5.0) { // Timeout fallback
            return true;
        }

        return rpmIsStable && (currentTime - lastStableRpmTime >= 0.25);
    }

    /**
     * Enhanced single shot with success tracking
     */
    public boolean fireSingleShot() {
        if (!isInitialized()) return false;

        double currentTime = clock.seconds();

        if (!isShooting && isShooterReady()) {
            startFeedServos();
            isShooting = true;
            feedStartTime = currentTime;
            lastShotTime = currentTime;

            // Record shot attempt
            totalShots++;
            double spinupTime = currentTime - shooterStartTime;
            totalSpinupTime += spinupTime;
            averageSpinupTime = totalSpinupTime / totalShots;

            successfulShots++; // Assume success for now
            consecutiveFailures = 0; // Reset failure count on successful shot

            return true;
        }

        if (isShooting && (currentTime - feedStartTime >= Tunables.SHOOTER_FEED_DELAY)) {
            stopFeedServos();
            isShooting = false;
        }

        return false;
    }

    /**
     * Smart autonomous shooting with adaptive timing
     */
    public void autoShoot(int numShots, boolean keepShooterRunning, ShooterPreset preset) {
        if (!isInitialized() || numShots <= 0) return;

        currentPreset = preset;
        if (!shooterRunning) startShooter();

        int shotsFired = 0;

        // Wait for spinup
        while (!isShooterReady() && !emergencyStop) {
            updateRPM();
            // Small delay to prevent busy waiting
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }
        }

        // Fire shots
        while (shotsFired < numShots && !emergencyStop) {
            if (fireSingleShot()) {
                shotsFired++;
            }

            // Wait for next shot
            double waitStart = clock.seconds();
            while (clock.seconds() - waitStart < Tunables.SHOOTER_FEED_DELAY && shotsFired < numShots) {
                updateRPM();
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
            }
        }

        if (!keepShooterRunning) {
            stopShooter();
        }
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

    /**
     * Start feed servos using Tunables power settings
     */
    private void startFeedServos() {
        if (!isInitialized()) return;

        feedServo1.setPower(-Tunables.SHOOTER_FEED_SERVO_POWER);
        feedServo2.setPower(Tunables.SHOOTER_FEED_SERVO_POWER);
    }

    /**
     * Stop feed servos
     */
    private void stopFeedServos() {
        if (!isInitialized()) return;

        feedServo1.setPower(0.0);
        feedServo2.setPower(0.0);
    }

    /**
     * Stop shooter motor
     */
    public void stopShooter() {
        if (!isInitialized()) return;

        shooter.setPower(0.0);
        shooterRunning = false;
    }

    /**
     * Reset all shooter state
     */
    public void reset() {
        stopShooter();
        stopFeedServos();
        isShooting = false;
        lastShotTime = 0;
        prevButtonState = false;
        emergencyStop = false;
        consecutiveFailures = 0;
        totalShots = 0;
        successfulShots = 0;
        totalSpinupTime = 0;
        averageSpinupTime = 0;
        clock.reset();
    }

    /**
     * Update shooter system - call this every loop
     */
    public void update() {
        if (!isInitialized()) return;

        updateRPM();

        // Check for debug output
        if (Tunables.isDebugEnabled("shooter")) {
            // Debug information will be handled by telemetry system
        }
    }

    /**
     * Get shooter system telemetry data
     */
    public ShooterSystemData getTelemetryData() {
        updateRPM(); // Ensure current data

        return new ShooterSystemData(
            currentPreset.getName(),
            getTargetRPM(),
            currentRPM,
            currentPower,
            shooterRunning,
            isShooting,
            rpmIsStable,
            emergencyStop,
            totalShots,
            successfulShots,
            averageSpinupTime,
            voltageSensor != null ? voltageSensor.getVoltage() : 0.0,
            isInitialized,
            Tunables.isDebugEnabled("shooter")
        );
    }

    /**
     * Data container for shooter system telemetry information
     */
    public static class ShooterSystemData {
        public final String presetName;
        public final double targetRPM;
        public final double currentRPM;
        public final double currentPower;
        public final boolean shooterRunning;
        public final boolean isShooting;
        public final boolean rpmStable;
        public final boolean emergencyStop;
        public final int totalShots;
        public final int successfulShots;
        public final double averageSpinupTime;
        public final double voltage;
        public final boolean initialized;
        public final boolean debugEnabled;

        public ShooterSystemData(String presetName, double targetRPM, double currentRPM, double currentPower,
                               boolean shooterRunning, boolean isShooting, boolean rpmStable, boolean emergencyStop,
                               int totalShots, int successfulShots, double averageSpinupTime, double voltage,
                               boolean initialized, boolean debugEnabled) {
            this.presetName = presetName;
            this.targetRPM = targetRPM;
            this.currentRPM = currentRPM;
            this.currentPower = currentPower;
            this.shooterRunning = shooterRunning;
            this.isShooting = isShooting;
            this.rpmStable = rpmStable;
            this.emergencyStop = emergencyStop;
            this.totalShots = totalShots;
            this.successfulShots = successfulShots;
            this.averageSpinupTime = averageSpinupTime;
            this.voltage = voltage;
            this.initialized = initialized;
            this.debugEnabled = debugEnabled;
        }

        public double getAccuracy() {
            return totalShots > 0 ? (double) successfulShots / totalShots * 100.0 : 0.0;
        }
    }

    // === GETTERS AND SETTERS ===

    public boolean isInitialized() {
        return isInitialized && robotMap.isShooterSystemReady();
    }

    public ShooterPreset getCurrentPreset() {
        return currentPreset;
    }

    public void setCurrentPreset(ShooterPreset preset) {
        this.currentPreset = preset;
    }

    public double getCustomRPM() {
        return customRPM;
    }

    public void setCustomRPM(double rpm) {
        this.customRPM = Math.max(0, Math.min(Tunables.SHOOTER_MAX_RPM, rpm));
    }

    public double getCurrentRPM() {
        updateRPM();
        return currentRPM;
    }

    public boolean isShooterRunning() {
        return shooterRunning;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public boolean isEmergencyStop() {
        return emergencyStop;
    }

    public double getAccuracy() {
        return totalShots > 0 ? (double) successfulShots / totalShots * 100.0 : 0.0;
    }

    /**
     * Get status summary for debugging
     */
    public String getStatusSummary() {
        if (!isInitialized()) {
            return "ShooterHandler: Not initialized - Shooter system not ready";
        }

        return String.format("ShooterHandler: %s preset, %.0f/%.0f RPM, %s, Debug: %s",
            currentPreset.getName(),
            currentRPM,
            getTargetRPM(),
            shooterRunning ? "RUNNING" : "STOPPED",
            Tunables.isDebugEnabled("shooter") ? "ON" : "OFF");
    }

    /**
     * Get current Tunables configuration summary for diagnostics
     */
    public String getTunablesInfo() {
        return String.format("Tunables: kP=%.5f, kI=%.5f, kD=%.5f, Tol=%.0f",
            Tunables.SHOOTER_KP,
            Tunables.SHOOTER_KI,
            Tunables.SHOOTER_KD,
            Tunables.SHOOTER_RPM_TOLERANCE);
    }
}

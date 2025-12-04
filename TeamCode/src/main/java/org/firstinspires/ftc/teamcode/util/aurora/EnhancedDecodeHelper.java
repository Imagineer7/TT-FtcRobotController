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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

/**
 * EnhancedDecodeHelper - Next-generation shooter control system
 *
 * New Features:
 * - Configurable shooting presets
 * - Performance monitoring and analytics
 * - Battery voltage compensation
 * - Predictive maintenance alerts
 * - Advanced safety systems
 * - Machine learning shot optimization
 */
public class EnhancedDecodeHelper {

    // Hardware components
    private DcMotor shooter;
    private CRServo feedServo1;
    private CRServo feedServo2;
    private Servo light;
    private VoltageSensor voltageSensor;
    private GoBildaPinpointDriver odometry;

    // Enhanced systems
    private ShooterConfig config;
    private PerformanceMonitor monitor;
    private RpmLearningSystem rpmLearning;
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
    private String debugStopReason = "NONE"; // Track why shooter was stopped (for debugging)

    // RPM tracking with momentum detection
    private int lastEncoderPosition = 0;
    private double lastRpmCheckTime = 0;
    private double currentRPM = 0;
    private double previousRPM = 0;
    private double rpmVelocity = 0; // Rate of RPM change
    private boolean rpmIsStable = false;
    private double lastStableRpmTime = 0;
    private double lastRpmInRangeTime = 0; // Track when RPM returns to range after a shot
    private int consecutiveStableReadings = 0; // Count consecutive stable RPM readings
    private static final double COUNTS_PER_REV = 28.0;
    private static final double RPM_CHECK_INTERVAL_NORMAL = 0.1; // 100ms normal
    private static final double RPM_CHECK_INTERVAL_RECOVERY = 0.05; // 50ms during recovery for faster response

    // Dynamic RPM control (Optimized for fast recovery after shooting)
    private double currentPower = 0;
    private double rpmError = 0;
    private double lastRpmError = 0;
    private double rpmErrorSum = 0;

    // ML Performance tracking for learning
    private double mlDataCollectionStartTime = 0;
    private double mlTargetRpmAtStart = 0;
    private double mlMaxOvershoot = 0;
    private boolean mlDataCollectionActive = false;
    private double mlLastStableTime = 0;

    // Adaptive PID gains - now sourced from ML system
    private static final double RPM_KP = 0.00020; // Default fallback values
    private static final double RPM_KP_RECOVERY = 0.00042;
    private static final double RPM_KI = 0.00005;
    private static final double RPM_KI_RECOVERY = 0.00004;
    private static final double RPM_KD = 0.00005;
    private static final double RPM_KD_RECOVERY = 0.00015;

    private static final double MAX_POWER_ADJUSTMENT = 0.20; // Steady-state adjustment limit
    private static final double MAX_POWER_ADJUSTMENT_RECOVERY = 0.25; // Recovery limit - reduced
    private static final double MAX_POWER_RATE = 0.12; // Steady-state rate limit
    private static final double MAX_POWER_RATE_RECOVERY = 0.20; // Recovery rate - reduced for smoother control

    // Recovery mode tracking with overshoot detection
    private boolean inRecoveryMode = false;
    private double recoveryStartTime = 0;
    private double lastTargetRpm = 0;
    private boolean wasUnderTarget = true; // Track if we're approaching from below
    private static final double RECOVERY_ERROR_THRESHOLD = 150; // RPM error to trigger recovery mode
    private static final double RECOVERY_BOOST_DURATION = 0.5; // Extended for smoother transition

     // Shot compensation boost - adaptive learning system
    private boolean shotBoostActive = false;
    private double shotBoostStartTime = 0;

    // Adaptive boost parameters (learned from each shot)
    private double shotBoostDelay = 0.020; // Default: 20ms delay after feed servos fire
    private double shotBoostDuration = 0.150; // Default: 150ms boost duration (reduced to prevent overshoot)
    private double shotBoostPower = 0.12; // Default: Extra 12% power during boost (reduced to prevent overshoot)

    // Learning system state
    private double lastShotRpmDrop = 0; // Track RPM drop from last shot
    private double lastShotRpmBeforeFiring = 0; // Store RPM before last shot for percentage calc
    private double lastShotRecoveryTime = 0; // Track how long recovery took
    private int shotsAnalyzed = 0; // Count shots used for learning
    private boolean learningEnabled = false; // Default OFF - enable only in training mode

    // Enhanced trajectory tracking
    private double shotRpmBeforeFiring = 0; // RPM just before shot
    private double shotRpmMin = 0; // Lowest RPM reached during shot
    private double shotRpmRecoveryStart = 0; // When recovery began
    private double shotRpmOvershoot = 0; // Peak overshoot above target
    private boolean trackingShot = false;

    // Overcompensation detection
    private double previousShotRpmDrop = 0; // Track previous shot's RPM drop
    private double previousShotOvershoot = 0; // Track previous shot's overshoot
    private double previousShotRecoveryTime = 0; // Track previous shot's recovery time
    private int consecutiveOvershoots = 0; // Track consecutive overshoot occurrences
    private int consecutiveSlowRecoveries = 0; // Track consecutive slow recoveries
    private boolean isOvercompensating = false; // Flag when system is making too aggressive changes

    // Adaptive learning rates
    private double learningRate = 0.015; // 1.5% adjustment (reduced from 5%)
    private double learningRateFineTune = 0.005; // 0.5% for fine-tuning after 20 shots

    // NEW: Shot interval learning
    private double learnedShotInterval = 0.150; // Start at 150ms default
    private double minShotInterval = 0.120; // Absolute minimum (120ms)
    private double maxShotInterval = 0.250; // Maximum if needed
    private boolean lastShotSuccessful = false;
    private int consecutiveSuccessfulShots = 0;

    // NEW: Adaptive RPM tolerance learning
    private double firstShotRpmTolerance = 25.0; // Tight for first shot
    private double learnedRapidFireTolerance = 40.0; // Looser for rapid shots
    private double minRapidFireTolerance = 25.0;
    private double maxRapidFireTolerance = 60.0;

    // Feed timing - NOT learned (fixed by config to prevent multi-ball ejection)
    private double lastRpmBeforeFeed = 0;
    private double rpmDropDuringFeed = 0;

    // Safety systems
    private boolean emergencyStop = false;
    private int consecutiveFailures = 0;
    private static final int MAX_FAILURES = 3;

    /**
     * Constructor using unified hardware configuration (PREFERRED)
     * @param hardware The unified hardware configuration
     */
    public EnhancedDecodeHelper(AuroraHardwareConfig hardware) {
        // Get hardware from unified config
        shooter = hardware.getShooterMotor();
        feedServo1 = hardware.getFeedServo1();
        feedServo2 = hardware.getFeedServo2();
        light = hardware.getLightServo();
        voltageSensor = hardware.getVoltageSensor();
        odometry = hardware.getOdometry();

        // Configure shooter motor
        if (shooter != null) {
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize enhanced systems
        config = new ShooterConfig();
        monitor = new PerformanceMonitor();
        rpmLearning = new RpmLearningSystem();
        clock = new ElapsedTime();

        reset();

        // Load previously learned boost parameters if available
        loadBoostParametersFromFile();
    }

    /**
     * Constructor with enhanced initialization (without odometry - for autonomous)
     * @deprecated Use EnhancedDecodeHelper(AuroraHardwareConfig) instead
     */
    @Deprecated
    public EnhancedDecodeHelper(HardwareMap hardwareMap) {
        this(hardwareMap, false);  // Default: no odometry
    }

    /**
     * Constructor with enhanced initialization
     * @param hardwareMap The hardware map
     * @param enableOdometry Set to true to initialize odometry (for TeleOp), false for autonomous
     * @deprecated Use EnhancedDecodeHelper(AuroraHardwareConfig) instead
     */
    @Deprecated
    public EnhancedDecodeHelper(HardwareMap hardwareMap, boolean enableOdometry) {
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

        // Initialize odometry ONLY if explicitly enabled (for TeleOp use)
        if (enableOdometry) {
            try {
                odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

                // Configure odometry with specified settings
                // forwardPodY(-6.62) and strafePodX(4.71)
                // X pod (forward) offset = strafePodX = 4.71
                // Y pod (strafe) offset = forwardPodY = -6.62
                odometry.setOffsets(4.71, -6.62, DistanceUnit.INCH);
                odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                odometry.setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED  // Strafe pod reversed
                );
                odometry.resetPosAndIMU();
            } catch (Exception e) {
                odometry = null;
            }
        } else {
            odometry = null;  // Explicitly disable odometry for autonomous
        }

        // Configure shooter motor
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use encoders for measurement only
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize enhanced systems
        config = new ShooterConfig();
        monitor = new PerformanceMonitor();
        rpmLearning = new RpmLearningSystem();
        clock = new ElapsedTime();

        reset();

        // Load previously learned boost parameters if available
        loadBoostParametersFromFile();
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

        // Safety check - only emergency stop should halt shooter
        // Performance degradation is logged but doesn't stop shooter in TeleOp
        if (emergencyStop) {
            if (shooterRunning) {
                debugStopReason = "EMERGENCY_STOP"; // Store for telemetry
                stopShooter();
            }
            updateLightIndicator();
            return false;
        }

        // Log performance degradation but don't stop shooter
        if (monitor.isPerformanceDegraded() && shooterRunning) {
            debugStopReason = "PERF_WARNING"; // Warning only, not stopping
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
            debugStopReason = "BUTTON_RELEASED"; // DEBUG
            stopShooter();
            if (isShooting) {
                stopFeedServos();
                isShooting = false;
            }
        }

        // Maintain shooter RPM control while running
        if (shooterRunning && !warmupMode) {
            updateRPM();
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
        debugStopReason = "NONE"; // Clear stop reason when starting

        // Start ML data collection for this spinup cycle
        startMlDataCollection(targetRpm);

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
     * Enhanced RPM calculation with adaptive update rate and momentum tracking
     * Now includes trajectory tracking during shots
     */
    private void updateRPM() {
        if (!shooterRunning) {
            currentRPM = 0;
            previousRPM = 0;
            rpmVelocity = 0;
            rpmIsStable = false;
            return;
        }

        double currentTime = clock.seconds();
        int currentPosition = shooter.getCurrentPosition();

        // Use faster check interval during recovery for quicker response
        double checkInterval = inRecoveryMode ? RPM_CHECK_INTERVAL_RECOVERY : RPM_CHECK_INTERVAL_NORMAL;

        if (currentTime - lastRpmCheckTime >= checkInterval) {
            double deltaTime = currentTime - lastRpmCheckTime;
            int deltaPosition = currentPosition - lastEncoderPosition;

            previousRPM = currentRPM;
            currentRPM = Math.abs((deltaPosition / deltaTime) * 60.0 / COUNTS_PER_REV);

            // Calculate RPM velocity (rate of change) for momentum prediction
            rpmVelocity = (currentRPM - previousRPM) / deltaTime;

            // Track shot trajectory for learning
            if (trackingShot && learningEnabled) {
                double targetRpm = config.getTargetRPM();

                // Track minimum RPM during shot
                if (shotRpmMin == 0 || currentRPM < shotRpmMin) {
                    shotRpmMin = currentRPM;
                    shotRpmRecoveryStart = currentTime;
                }

                // Track overshoot above target
                if (currentRPM > targetRpm) {
                    double overshoot = currentRPM - targetRpm;
                    shotRpmOvershoot = Math.max(shotRpmOvershoot, overshoot);
                }

                // Track recovery time (when RPM returns to acceptable range)
                if (shotRpmMin > 0 && Math.abs(currentRPM - targetRpm) < 50) {
                    if (lastShotRecoveryTime == 0) {
                        lastShotRecoveryTime = currentTime - shotRpmRecoveryStart;
                    }
                }
            }

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
     * Adaptive PID control with fast recovery mode and steady-state stability
     * Now uses learned gains from ML system
     */
    private void adjustPowerForTargetRPM(double deltaTime) {
        // Skip if in warmup mode (warmup has its own control)
        if (warmupMode) {
            return;
        }

        double targetRpm = config.getTargetRPM();
        lastRpmError = rpmError;
        rpmError = targetRpm - currentRPM;

        // Update ML system with voltage compensation learning
        if (voltageSensor != null) {
            rpmLearning.updateVoltageCompensation(voltageSensor.getVoltage(), targetRpm, currentRPM);
        }

        // Continue ML data collection
        updateMlDataCollection();

        // Let feedforward work first - reduced delay for faster initial response
        double timeSinceStart = clock.seconds() - shooterStartTime;
        if (timeSinceStart < 0.4) { // Reduced from 0.6s to 0.4s
            return;
        }

        double currentTime = clock.seconds();
        double errorMagnitude = Math.abs(rpmError);

        // Handle shot compensation boost with smooth ramp and overshoot protection
        double boostPowerContribution = 0.0;
        if (shotBoostActive) {
            double timeSinceBoostStart = currentTime - shotBoostStartTime;
            double boostStartTime = shotBoostDelay;
            double boostEndTime = shotBoostDelay + shotBoostDuration;

            // Check for overshoot - cancel boost if RPM exceeds target by 50 RPM (more aggressive)
            boolean hasOvershot = currentRPM > (targetRpm + 50);

            if (hasOvershot) {
                // Cancel boost immediately to prevent overshoot
                shotBoostActive = false;
                if (learningEnabled) {
                    analyzeAndLearnFromShot();
                }
            } else if (timeSinceBoostStart >= boostStartTime && timeSinceBoostStart < boostEndTime) {
                // Calculate boost progress (0.0 to 1.0)
                double boostProgress = (timeSinceBoostStart - boostStartTime) / shotBoostDuration;

                // Reduce boost as we approach target RPM to prevent overshoot
                double rpmErrorRatio = Math.max(0.0, Math.min(1.0, Math.abs(rpmError) / 200.0));

                // Apply boost with smooth ramp-down in last 30% to prevent sudden drop
                if (boostProgress < 0.7) {
                    // Full boost for first 70% of duration, scaled by RPM error
                    boostPowerContribution = shotBoostPower * rpmErrorRatio;
                } else {
                    // Linear ramp-down for last 30%
                    double rampFactor = (1.0 - boostProgress) / 0.3;
                    boostPowerContribution = shotBoostPower * rampFactor * rpmErrorRatio;
                }
            } else if (timeSinceBoostStart >= boostEndTime) {
                // Boost complete - analyze shot performance for learning
                if (learningEnabled) {
                    analyzeAndLearnFromShot();
                }
                shotBoostActive = false;
            }
        }

        // Determine if we should be in recovery mode (aggressive) or steady state (stable)
        if (errorMagnitude > RECOVERY_ERROR_THRESHOLD) {
            // Large error detected - enter or continue recovery mode
            if (!inRecoveryMode) {
                inRecoveryMode = true;
                recoveryStartTime = currentTime;
                // Clear integral term when entering recovery to prevent windup
                rpmErrorSum = 0;
            }
        } else if (inRecoveryMode) {
            // Check if we should exit recovery mode
            double timeInRecovery = currentTime - recoveryStartTime;
            if (timeInRecovery > RECOVERY_BOOST_DURATION || errorMagnitude < 50) {
                // Exit recovery mode after duration or when close to target
                inRecoveryMode = false;
            }
        }

        // Get learned PID gains from ML system
        double[] learnedGains = inRecoveryMode ?
            rpmLearning.getLearnedGainsRecovery() :
            rpmLearning.getLearnedGainsSteadyState();
        double kp = learnedGains[0];
        double ki = learnedGains[1];
        double kd = learnedGains[2];
        double maxAdjustment = inRecoveryMode ? MAX_POWER_ADJUSTMENT_RECOVERY : MAX_POWER_ADJUSTMENT;
        double maxRate = inRecoveryMode ? MAX_POWER_RATE_RECOVERY : MAX_POWER_RATE;

        // Apply voltage compensation from ML system
        double voltageCompensation = rpmLearning.getVoltageCompensationFactor();
        if (voltageSensor != null) {
            double voltageRatio = voltageSensor.getVoltage() / 12.0;
            voltageCompensation *= voltageRatio;
        }
        kp *= voltageCompensation;

        // Momentum-based gain scaling - reduce gains if RPM is recovering quickly
        if (inRecoveryMode && rpmVelocity > 0 && errorMagnitude < 200) {
            // RPM is increasing and we're getting close - scale back to prevent overshoot
            double momentumFactor = Math.max(0.7, 1.0 - (rpmVelocity / 1000.0));
            kp *= momentumFactor;
            maxAdjustment *= momentumFactor;
        }

        // Proportional term
        double proportional = rpmError * kp;

        // Integral term with adaptive windup protection
        double integralGain = ki;
        if (!inRecoveryMode && errorMagnitude < 75) {
            // Near target in steady state - reduce integral to prevent oscillation
            integralGain *= 0.5;
        }

        rpmErrorSum += rpmError * deltaTime;

        // Dynamic windup limit - tighter in steady state, reduced in recovery to prevent overshoot
        double windupLimit = inRecoveryMode ? 600 : Math.max(400, 1500 - errorMagnitude * 2);
        if (Math.abs(rpmErrorSum) > windupLimit) {
            rpmErrorSum = Math.signum(rpmErrorSum) * windupLimit;
        }
        double integral = rpmErrorSum * integralGain;

        // Derivative term for damping (more aggressive in recovery)
        double derivative = ((rpmError - lastRpmError) / deltaTime) * kd;

        // Calculate power adjustment
        double powerAdjustment = proportional + integral + derivative;

        // Limit the adjustment magnitude
        powerAdjustment = Math.max(-maxAdjustment, Math.min(maxAdjustment, powerAdjustment));

        // Apply rate limiting for smooth power changes
        double targetPower = currentPower + powerAdjustment + boostPowerContribution;
        double powerDelta = targetPower - currentPower;

        // Limit how fast power can change based on mode
        if (Math.abs(powerDelta) > maxRate) {
            powerDelta = Math.signum(powerDelta) * maxRate;
        }

        currentPower += powerDelta;

        // Ensure power stays within valid range
        currentPower = Math.max(0.0, Math.min(1.0, currentPower));

        // Update motor power
        shooter.setPower(currentPower);
    }

    /**
     * Enhanced readiness check with failure detection and RPM validation
     * Now uses learned shot interval and adaptive RPM tolerance
     */
    public boolean isShooterReady() {
        double currentTime = clock.seconds();

        // Use learned shot interval (or config default if not learning)
        double shotInterval = learningEnabled ? learnedShotInterval : config.getShotInterval();

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

                // STRICT TOLERANCE: Only fire within 15 RPM of target (no learned tolerance override)
                // This prevents second shot overshoot issues
                double strictTolerance = 15.0;
                double preferredTolerance = strictTolerance;
                double acceptableTolerance = strictTolerance;

                boolean inPreferredZone = rpmDifference <= preferredTolerance;
                boolean inAcceptableZone = rpmDifference <= acceptableTolerance;

                if (inPreferredZone) {
                    // RPM is in preferred zone - short but consistent wait
                    if (lastRpmInRangeTime == 0) {
                        lastRpmInRangeTime = currentTime;
                        consecutiveStableReadings = 1;
                    } else {
                        consecutiveStableReadings++;
                    }

                    // Require 0.08s AND 2 consecutive readings for consistency (reduced from 0.12s)
                    if ((currentTime - lastRpmInRangeTime < 0.08) || (consecutiveStableReadings < 2)) {
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

                    // Require 0.15s AND 3 consecutive readings in acceptable zone (reduced from 0.2s)
                    if ((currentTime - lastRpmInRangeTime < 0.15) || (consecutiveStableReadings < 3)) {
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
     * Start ML data collection for current spinup/shooting cycle
     */
    private void startMlDataCollection(double targetRpm) {
        if (!rpmLearning.isLearningEnabled()) return;

        mlDataCollectionStartTime = clock.seconds();
        mlTargetRpmAtStart = targetRpm;
        mlMaxOvershoot = 0;
        mlDataCollectionActive = true;
        mlLastStableTime = 0;
    }

    /**
     * Update ML data collection during RPM control
     */
    private void updateMlDataCollection() {
        if (!mlDataCollectionActive || !rpmLearning.isLearningEnabled()) return;

        double currentTime = clock.seconds();
        double overshoot = Math.max(0, currentRPM - mlTargetRpmAtStart);
        mlMaxOvershoot = Math.max(mlMaxOvershoot, overshoot);

        // Check if RPM has stabilized for the first time
        if (mlLastStableTime == 0 && rpmIsStable) {
            mlLastStableTime = currentTime;
        }
    }

    /**
     * Finalize ML data collection and record performance sample
     */
    private void finalizeMlDataCollection() {
        if (!mlDataCollectionActive || !rpmLearning.isLearningEnabled()) return;

        double currentTime = clock.seconds();
        double settlingTime = mlLastStableTime > 0 ?
            (mlLastStableTime - mlDataCollectionStartTime) :
            (currentTime - mlDataCollectionStartTime);

        double voltage = voltageSensor != null ? voltageSensor.getVoltage() : 12.0;

        // Record performance sample for ML learning
        rpmLearning.recordPerformance(
            mlTargetRpmAtStart,
            currentRPM,
            mlMaxOvershoot,
            settlingTime,
            voltage
        );

        mlDataCollectionActive = false;
    }

    /**
     * Enhanced single shot with success tracking and fast recovery preparation
     * Now uses learned feed time and tracks shot trajectory
     */
    public boolean fireSingleShot() {
        double currentTime = clock.seconds();

        // Check if we can start a new shot
        if (!isShooting && isShooterReady()) {
            // Finalize ML data collection before shooting
            finalizeMlDataCollection();

            // Start trajectory tracking
            if (learningEnabled) {
                shotRpmBeforeFiring = currentRPM;
                shotRpmMin = 0;
                shotRpmOvershoot = 0;
                shotRpmRecoveryStart = 0;
                lastShotRecoveryTime = 0;
                trackingShot = true;
                lastRpmBeforeFeed = currentRPM;
            }

            startFeedServos();
            isShooting = true;
            feedStartTime = currentTime;
            lastShotTime = currentTime;
            firstShotFired = true; // Mark that first shot has been fired

            // Activate shot compensation boost
            shotBoostActive = true;
            shotBoostStartTime = currentTime;

            // Prepare for fast recovery after shot
            rpmErrorSum = 0; // Clear integral term to prevent windup during shot
            inRecoveryMode = true; // Preemptively enter recovery mode
            recoveryStartTime = currentTime;

            // Reset RPM stability tracking for next shot
            lastRpmInRangeTime = 0;
            consecutiveStableReadings = 0;

            // Start new ML data collection cycle for post-shot recovery
            startMlDataCollection(config.getTargetRPM());

            // Record shot attempt
            double spinupTime = currentTime - shooterStartTime;
            monitor.recordShotAttempt(true, spinupTime); // Assume success for now
            consecutiveFailures = 0; // Reset failure count on successful shot

            return true;
        }

        // Handle ongoing shot - stop feed servos when feed time expires
        // Feed time is NOT learned (fixed from config to prevent multi-ball ejection)
        double feedTime = config.getFeedTime();
        if (isShooting && (currentTime - feedStartTime >= feedTime)) {
            stopFeedServos();
            isShooting = false;

            // Calculate RPM drop during feed for learning analysis
            if (learningEnabled && lastRpmBeforeFeed > 0) {
                rpmDropDuringFeed = lastRpmBeforeFeed - currentRPM;
            }
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

        // Strict 15 RPM tolerance check for first shot
        double targetRpm = config.getTargetRPM();
        double rpmDifference = Math.abs(currentRPM - targetRpm);
        boolean withinStrictTolerance = rpmDifference <= 15.0;

        return rpmIsStable && withinStrictTolerance && (currentTime - lastStableRpmTime >= 0.25);
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
        // Finalize any ongoing ML data collection
        finalizeMlDataCollection();

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
    public RpmLearningSystem getRpmLearning() { return rpmLearning; }
    public boolean isEmergencyStop() { return emergencyStop; }
    public double getCurrentRPM() { updateRPM(); return currentRPM; }
    public boolean isShooterRunning() { return shooterRunning; }
    public boolean isShooting() { return isShooting; }
    public boolean isWarmupMode() { return warmupMode; }
    public double getTargetRPM() { return warmupMode ? config.getWarmupTargetRPM() : config.getTargetRPM(); }
    public double getShooterMotorPower() { return shooter.getPower(); }
    public int getShooterMotorPosition() { return shooter.getCurrentPosition(); }
    public String getDebugStopReason() { return debugStopReason; }

    /**
     * Handle ML system controls (save, reset)
     * Call this in your TeleOp loop with appropriate gamepad inputs
     */
    public void handleMlControls(boolean saveButton, boolean resetButtonHeld, double resetHoldTime) {
        // Save ML data
        if (saveButton) {
            if (rpmLearning.saveLearningData()) {
                // Successfully saved - could trigger telemetry message
            }
        }

        // Reset ML data (requires holding button for 2+ seconds)
        if (resetButtonHeld && resetHoldTime >= 2.0) {
            rpmLearning.resetLearning();
        }
    }

    /**
     * Get ML system telemetry
     */
    public String getMlTelemetry() {
        return rpmLearning.getLearningTelemetry();
    }

    /**
     * Get ML performance metrics
     */
    public String getMlPerformanceMetrics() {
        return rpmLearning.getPerformanceMetrics();
    }

    /**
     * Get RPM learning telemetry (alias for compatibility)
     */
    public String getRpmLearningTelemetry() {
        return rpmLearning.getLearningTelemetry();
    }

    /**
     * Get RPM performance metrics (alias for compatibility)
     */
    public String getRpmPerformanceMetrics() {
        return rpmLearning.getPerformanceMetrics();
    }

    /**
     * Get total shots fired
     */
    public int getTotalShots() {
        return monitor.getTotalShots();
    }

    /**
     * Get average spinup time
     */
    public double getAverageSpinupTime() {
        return monitor.getAverageSpinupTime();
    }

    /**
     * Get success rate percentage
     */
    public double getSuccessRate() {
        return monitor.getSuccessRate();
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

    // ========== ODOMETRY METHODS ==========

    /**
     * Update odometry position tracking
     * Call this regularly (in your main loop) to keep position data current
     */
    public void updateOdometry() {
        if (odometry != null) {
            odometry.update();
        }
    }

    /**
     * Get current robot position from odometry
     * @return Pose2D containing X, Y position and heading, or null if odometry not available
     */
    public Pose2D getPosition() {
        if (odometry != null) {
            return odometry.getPosition();
        }
        return null;
    }

    /**
     * Get current robot velocity from odometry
     * @return Pose2D containing X, Y velocity and angular velocity, or null if odometry not available
     */
    public Pose2D getVelocity() {
        if (odometry != null) {
            return odometry.getVelocity();
        }
        return null;
    }

    /**
     * Get X position in specified units
     * @param unit Distance unit (INCH, CM, MM, etc.)
     * @return X position or 0.0 if odometry not available
     */
    public double getPosX(DistanceUnit unit) {
        if (odometry != null) {
            return odometry.getPosX(unit);
        }
        return 0.0;
    }

    /**
     * Get Y position in specified units
     * @param unit Distance unit (INCH, CM, MM, etc.)
     * @return Y position or 0.0 if odometry not available
     */
    public double getPosY(DistanceUnit unit) {
        if (odometry != null) {
            return odometry.getPosY(unit);
        }
        return 0.0;
    }

    /**
     * Get heading in radians
     * @return Heading in radians or 0.0 if odometry not available
     */
    public double getHeading() {
        if (odometry != null) {
            Pose2D pos = odometry.getPosition();
            return pos.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        }
        return 0.0;
    }

    /**
     * Set robot position (useful for initialization or field-relative positioning)
     * @param pose Pose2D containing desired X, Y, and heading
     */
    public void setPosition(Pose2D pose) {
        if (odometry != null) {
            odometry.setPosition(pose);
        }
    }

    /**
     * Reset odometry position to origin (0, 0, 0) and recalibrate IMU
     * Robot MUST be stationary when calling this
     */
    public void resetOdometry() {
        if (odometry != null) {
            odometry.resetPosAndIMU();
        }
    }

    /**
     * Check if odometry is available and working
     * @return true if odometry hardware is present and ready
     */
    public boolean hasOdometry() {
        if (odometry == null) {
            return false;
        }
        try {
            GoBildaPinpointDriver.DeviceStatus status = odometry.getDeviceStatus();
            return status == GoBildaPinpointDriver.DeviceStatus.READY;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Get direct access to odometry driver for advanced use
     * @return GoBildaPinpointDriver instance or null if not available
     */
    public GoBildaPinpointDriver getOdometry() {
        return odometry;
    }

    // ========== ADAPTIVE LEARNING SYSTEM ==========

    /**
     * Analyze shot performance and adapt boost parameters + PID gains
     * Called automatically after each shot completes
     * Enhanced to minimize RPM drop and recovery time with overcompensation detection
     */
    private void analyzeAndLearnFromShot() {
        if (!learningEnabled) return;

        shotsAnalyzed++;
        trackingShot = false;

        // Get current recovery metrics
        double targetRpm = config.getTargetRPM();
        double finalError = Math.abs(currentRPM - targetRpm);
        double rpmDrop = shotRpmBeforeFiring - shotRpmMin; // How much RPM dropped
        double rpmDropPercentage = (shotRpmBeforeFiring > 0) ? (rpmDrop / shotRpmBeforeFiring * 100.0) : 0.0;

        // DETECT OVERCOMPENSATION - Check if performance is oscillating
        if (shotsAnalyzed >= 3) {
            // Check for oscillating overshoot (overshoot getting worse after corrections)
            if (shotRpmOvershoot > 25.0 && previousShotOvershoot > 25.0) {
                consecutiveOvershoots++;
            } else {
                consecutiveOvershoots = Math.max(0, consecutiveOvershoots - 1);
            }

            // Check for oscillating recovery time (alternating between fast and slow)
            if (lastShotRecoveryTime > 0 && previousShotRecoveryTime > 0) {
                double recoveryTimeDelta = Math.abs(lastShotRecoveryTime - previousShotRecoveryTime);
                if (recoveryTimeDelta > 0.08) { // Large swing in recovery time
                    consecutiveSlowRecoveries++;
                } else {
                    consecutiveSlowRecoveries = Math.max(0, consecutiveSlowRecoveries - 1);
                }
            }

            // Check for alternating RPM drop (getting worse then better repeatedly)
            boolean rpmDropIncreased = (rpmDropPercentage > previousShotRpmDrop + 5.0);
            boolean rpmDropDecreased = (rpmDropPercentage < previousShotRpmDrop - 5.0);

            // Detect overcompensation pattern
            if (consecutiveOvershoots >= 2 || consecutiveSlowRecoveries >= 2) {
                isOvercompensating = true;
            } else if (consecutiveOvershoots == 0 && consecutiveSlowRecoveries == 0) {
                isOvercompensating = false;
            }
        }

        // Use adaptive learning rate (fine-tune after 20 shots, or if overcompensating)
        double lr = learningRate;
        if (shotsAnalyzed >= 20) {
            lr = learningRateFineTune;
        }

        // CRITICAL: Reduce learning rate if overcompensating
        if (isOvercompensating) {
            lr *= 0.3; // Use only 30% of normal learning rate when overcompensating
        }

        // Track shot success - must be within 15 RPM for success
        lastShotSuccessful = (finalError <= 15.0); // Strict tolerance

        if (shotsAnalyzed >= 2) { // Start learning after just 2 shots

            // NEW GOAL: Minimize RPM drop and recovery time
            // Focus on adjusting boost and PID parameters, NOT shot interval

            // Calculate recovery performance score (lower is better)
            // Score = RPM drop percentage + recovery time (seconds) * 100
            double recoveryScore = rpmDropPercentage + (lastShotRecoveryTime * 100.0);

            // OVERCOMPENSATION CASE: If oscillating, make minimal conservative adjustments
            if (isOvercompensating) {
                // Back off aggressiveness - make small corrections toward middle ground
                if (shotRpmOvershoot > 35.0) {
                    // Still overshooting significantly - gently reduce boost
                    shotBoostPower = Math.max(0.10, shotBoostPower * (1.0 - lr * 0.5));
                } else if (rpmDropPercentage > 18.0) {
                    // Still dropping too much - gently increase boost
                    shotBoostPower = Math.min(0.22, shotBoostPower * (1.0 + lr * 0.5));
                }
                // Reset overshoot counters after adjustment
                if (consecutiveOvershoots >= 3) {
                    consecutiveOvershoots = 1;
                }
                if (consecutiveSlowRecoveries >= 3) {
                    consecutiveSlowRecoveries = 1;
                }
            }
            // CASE 1: Large RPM drop (>20%) - need stronger boost or better PID recovery gains
            else if (rpmDropPercentage > 20.0) {
                // Check if we're getting worse (overcompensating in wrong direction)
                if (previousShotRpmDrop > 0 && rpmDropPercentage > previousShotRpmDrop + 3.0) {
                    // Getting worse! Reduce adjustment magnitude
                    lr *= 0.5;
                }

                // Increase boost power to reduce initial drop
                shotBoostPower = Math.min(0.30, shotBoostPower * (1.0 + lr * 1.5));
                shotBoostDelay = Math.max(0.010, shotBoostDelay - 0.002); // Start boost earlier

                // Also increase PID recovery gains for faster recovery
                double[] recoveryGains = rpmLearning.getLearnedGainsRecovery();
                double newKpRecovery = Math.min(0.00080, recoveryGains[0] * (1.0 + lr));
                double newKdRecovery = Math.min(0.00040, recoveryGains[2] * (1.0 + lr * 0.5));
                rpmLearning.adjustRecoveryGains(newKpRecovery, recoveryGains[1], newKdRecovery);

                consecutiveSuccessfulShots = 0; // Reset success counter
            }
            // CASE 2: Small RPM drop (<10%) but slow recovery (>0.15s)
            else if (rpmDropPercentage < 10.0 && lastShotRecoveryTime > 0.15) {
                // Good drop resistance, but need faster recovery
                // Increase recovery PID gains, slightly reduce boost
                double[] recoveryGains = rpmLearning.getLearnedGainsRecovery();
                double newKpRecovery = Math.min(0.00080, recoveryGains[0] * (1.0 + lr * 1.2));
                double newKiRecovery = Math.min(0.00020, recoveryGains[1] * (1.0 + lr * 0.8));
                rpmLearning.adjustRecoveryGains(newKpRecovery, newKiRecovery, recoveryGains[2]);

                // Slightly reduce boost to prevent potential overshoot
                shotBoostPower = Math.max(0.08, shotBoostPower * (1.0 - lr * 0.3));

                consecutiveSuccessfulShots = 0; // Reset success counter
            }
            // CASE 3: Overshoot detected (>30 RPM above target)
            else if (shotRpmOvershoot > 30.0) {
                // Check if overshoot is getting worse despite corrections
                if (previousShotOvershoot > 0 && shotRpmOvershoot > previousShotOvershoot + 10.0) {
                    // Overshoot increasing! More aggressive reduction needed
                    lr *= 1.5;
                }

                // Reduce boost power and duration to prevent overshoot
                shotBoostPower = Math.max(0.08, shotBoostPower * (1.0 - lr * 0.8));
                shotBoostDuration = Math.max(0.100, shotBoostDuration - 0.010);

                // Increase damping (Kd) to reduce overshoot
                double[] recoveryGains = rpmLearning.getLearnedGainsRecovery();
                double newKdRecovery = Math.min(0.00040, recoveryGains[2] * (1.0 + lr * 1.0));
                rpmLearning.adjustRecoveryGains(recoveryGains[0], recoveryGains[1], newKdRecovery);

                consecutiveSuccessfulShots = 0; // Reset success counter
            }
            // CASE 4: Good RPM drop (<12%) AND fast recovery (<0.12s) AND no overshoot (<20 RPM)
            else if (rpmDropPercentage < 12.0 && lastShotRecoveryTime > 0 && lastShotRecoveryTime < 0.12 && shotRpmOvershoot < 20.0) {
                // Excellent performance! Fine-tune for consistency
                learningRateFineTune = Math.max(0.002, learningRateFineTune * 0.95);
                consecutiveSuccessfulShots++;
                lastShotSuccessful = true;

                // Optionally, slightly increase boost to push performance even further
                // Only if we've had consistent success (5+ shots)
                if (consecutiveSuccessfulShots >= 5) {
                    shotBoostPower = Math.min(0.25, shotBoostPower * (1.0 + lr * 0.1));
                }
            }
            // CASE 5: Moderate RPM drop (12-20%) with acceptable recovery
            else if (rpmDropPercentage >= 12.0 && rpmDropPercentage <= 20.0 && lastShotRecoveryTime < 0.15) {
                // Decent performance, small adjustments
                shotBoostPower = Math.min(0.25, shotBoostPower * (1.0 + lr * 0.5));
                shotBoostDelay = Math.max(0.010, shotBoostDelay - 0.001);

                consecutiveSuccessfulShots = 0; // Reset success counter
            }
            // CASE 6: Final error still high after recovery (>20 RPM)
            else if (finalError > 20.0) {
                // Recovery didn't complete properly - increase boost duration
                shotBoostDuration = Math.min(0.300, shotBoostDuration + 0.010);

                // Increase recovery Ki to eliminate steady-state error
                double[] recoveryGains = rpmLearning.getLearnedGainsRecovery();
                double newKiRecovery = Math.min(0.00020, recoveryGains[1] * (1.0 + lr * 0.6));
                rpmLearning.adjustRecoveryGains(recoveryGains[0], newKiRecovery, recoveryGains[2]);

                consecutiveSuccessfulShots = 0; // Reset success counter
            }

            // DO NOT learn shot interval - keep it fixed for consistent timing
            // learnShotInterval(); // REMOVED

            // DO NOT learn RPM tolerance - keep strict 15 RPM for accuracy
            // learnRpmTolerance(); // REMOVED
        }

        // Clamp all values to safe ranges
        shotBoostPower = Math.max(0.08, Math.min(0.30, shotBoostPower));
        shotBoostDuration = Math.max(0.100, Math.min(0.300, shotBoostDuration));
        shotBoostDelay = Math.max(0.010, Math.min(0.050, shotBoostDelay));

        // Store current metrics for next shot comparison (overcompensation detection)
        previousShotRpmDrop = rpmDropPercentage;
        previousShotOvershoot = shotRpmOvershoot;
        previousShotRecoveryTime = lastShotRecoveryTime;

        lastShotRpmDrop = rpmDrop;
        lastShotRpmBeforeFiring = shotRpmBeforeFiring; // Store for percentage calculation

        // Store recovery time for next analysis
        if (lastShotRecoveryTime == 0 && shotRpmRecoveryStart > 0) {
            lastShotRecoveryTime = clock.seconds() - shotRpmRecoveryStart;
        }

        // Reset tracking variables
        shotRpmMin = 0;
        shotRpmOvershoot = 0;
        shotRpmBeforeFiring = 0;
        shotRpmRecoveryStart = 0;
    }

    /**
     * Learn optimal shot interval timing
     */
    private void learnShotInterval() {
        if (!learningEnabled) return;

        if (lastShotSuccessful) {
            consecutiveSuccessfulShots++;

            // After 5 consecutive successful shots, try faster interval
            if (consecutiveSuccessfulShots >= 5) {
                learnedShotInterval = Math.max(minShotInterval,
                    learnedShotInterval - 0.005); // Reduce by 5ms
                consecutiveSuccessfulShots = 0; // Reset counter
            }
        } else {
            // Shot failed (RPM not ready) - increase interval
            learnedShotInterval = Math.min(maxShotInterval,
                learnedShotInterval + 0.010); // Add 10ms
            consecutiveSuccessfulShots = 0;
        }
    }

    /**
     * Learn optimal RPM tolerance for rapid fire
     */
    private void learnRpmTolerance() {
        if (!learningEnabled || !firstShotFired) return;

        if (lastShotSuccessful) {
            // If shots are working at current tolerance, try tightening
            if (consecutiveSuccessfulShots >= 8) {
                learnedRapidFireTolerance = Math.max(minRapidFireTolerance,
                    learnedRapidFireTolerance - 2.0); // Tighten by 2 RPM
            }
        } else {
            // If shot failed, loosen tolerance
            learnedRapidFireTolerance = Math.min(maxRapidFireTolerance,
                learnedRapidFireTolerance + 5.0); // Loosen by 5 RPM
        }
    }

    /**
     * Get learning telemetry data for display
     * @return Formatted string with learning parameters
     */
    public String getLearningTelemetry() {
        String overcompStatus = isOvercompensating ? " ⚠BACKING OFF" : "";
        return String.format("Boost Learning | Shots: %d | Delay: %.0fms | Duration: %.0fms | Power: %.1f%%%s",
                shotsAnalyzed, shotBoostDelay * 1000, shotBoostDuration * 1000, shotBoostPower * 100, overcompStatus);
    }

    /**
     * Check if system is currently overcompensating
     * @return true if oscillating behavior detected
     */
    public boolean isOvercompensating() {
        return isOvercompensating;
    }

    /**
     * Get overcompensation detection telemetry
     * @return Formatted string with overcompensation status
     */
    public String getOvercompensationTelemetry() {
        return String.format("Overcomp: %s | Overshoots: %d | SlowRecov: %d",
                isOvercompensating ? "YES" : "NO",
                consecutiveOvershoots,
                consecutiveSlowRecoveries);
    }

    /**
     * Get detailed learning telemetry with all parameters
     */
    public String getDetailedLearningTelemetry() {
        return String.format("Shots: %d | Interval: %.0fms | Tolerance: ±%.0f RPM",
                shotsAnalyzed, learnedShotInterval * 1000, learnedRapidFireTolerance);
    }

    /**
     * Get trajectory telemetry for current shot
     */
    public String getTrajectoryTelemetry() {
        if (!trackingShot) return "Not tracking";
        return String.format("RPM Before: %.0f | Min: %.0f | Drop: %.0f | Overshoot: %.0f",
                shotRpmBeforeFiring, shotRpmMin,
                shotRpmBeforeFiring - shotRpmMin, shotRpmOvershoot);
    }

    /**
     * Get detailed boost parameters for tuning
     */
    public String getBoostParameters() {
        return String.format("Delay: %.3f | Duration: %.3f | Power: %.3f",
                shotBoostDelay, shotBoostDuration, shotBoostPower);
    }

    /**
     * Manually adjust boost delay (for live tuning)
     * @param delta Change in seconds (e.g., 0.005 for +5ms)
     */
    public void adjustBoostDelay(double delta) {
        shotBoostDelay = Math.max(0.010, Math.min(0.050, shotBoostDelay + delta));
    }

    /**
     * Manually adjust boost duration (for live tuning)
     * @param delta Change in seconds (e.g., 0.010 for +10ms)
     */
    public void adjustBoostDuration(double delta) {
        shotBoostDuration = Math.max(0.100, Math.min(0.300, shotBoostDuration + delta));
    }

    /**
     * Manually adjust boost power (for live tuning)
     * @param delta Change in power (e.g., 0.01 for +1%)
     */
    public void adjustBoostPower(double delta) {
        shotBoostPower = Math.max(0.10, Math.min(0.30, shotBoostPower + delta));
    }

    /**
     * Get all learned parameters (for saving/loading)
     * Returns: [delay, duration, power, interval, tolerance, shotsAnalyzed]
     */
    public double[] getAllLearnedParameters() {
        return new double[] {
            shotBoostDelay,
            shotBoostDuration,
            shotBoostPower,
            learnedShotInterval,
            learnedRapidFireTolerance,
            shotsAnalyzed
        };
    }

    /**
     * Set all learned parameters (for loading saved values)
     */
    public void setAllLearnedParameters(double delay, double duration, double power,
                                       double shotInterval, double rpmTolerance,
                                       int shots) {
        shotBoostDelay = Math.max(0.010, Math.min(0.050, delay));
        shotBoostDuration = Math.max(0.100, Math.min(0.300, duration));
        shotBoostPower = Math.max(0.10, Math.min(0.30, power));
        learnedShotInterval = Math.max(minShotInterval, Math.min(maxShotInterval, shotInterval));
        learnedRapidFireTolerance = Math.max(minRapidFireTolerance, Math.min(maxRapidFireTolerance, rpmTolerance));
        shotsAnalyzed = shots;
    }

    /**
     * Get current boost parameters (for saving/loading) - backward compatibility
     */
    public double[] getBoostParametersArray() {
        return new double[] { shotBoostDelay, shotBoostDuration, shotBoostPower };
    }

    /**
     * Set boost parameters (for loading saved values) - backward compatibility
     * @param delay Boost delay in seconds
     * @param duration Boost duration in seconds
     * @param power Boost power (0.0 to 1.0)
     */
    public void setBoostParameters(double delay, double duration, double power) {
        shotBoostDelay = Math.max(0.010, Math.min(0.050, delay));
        shotBoostDuration = Math.max(0.100, Math.min(0.300, duration));
        shotBoostPower = Math.max(0.10, Math.min(0.30, power));
    }

    /**
     * Enable or disable adaptive learning
     * Controls BOTH shot boost learning AND PID learning systems
     * @param enabled true to enable learning (training mode), false to use fixed parameters (competition mode)
     */
    public void setLearningEnabled(boolean enabled) {
        learningEnabled = enabled;
        rpmLearning.setLearningEnabled(enabled); // Also control PID learning
    }

    /**
     * Check if learning is enabled
     */
    public boolean isLearningEnabled() {
        return learningEnabled;
    }

    /**
     * Reset learning statistics
     */
    public void resetLearning() {
        shotsAnalyzed = 0;
        lastShotRpmDrop = 0;
        lastShotRecoveryTime = 0;
    }

    /**
     * Get number of shots analyzed for learning
     */
    public int getShotsAnalyzed() {
        return shotsAnalyzed;
    }

    /**
     * Get last shot RPM drop (absolute value)
     */
    public double getLastShotRpmDrop() {
        return lastShotRpmDrop;
    }

    /**
     * Get last shot RPM drop as percentage of pre-shot RPM
     */
    public double getLastShotRpmDropPercentage() {
        if (lastShotRpmBeforeFiring > 0) {
            return (lastShotRpmDrop / lastShotRpmBeforeFiring) * 100.0;
        }
        return 0.0;
    }

    /**
     * Get last shot recovery time in seconds
     */
    public double getLastShotRecoveryTime() {
        return lastShotRecoveryTime;
    }

    /**
     * Load all learned parameters from persistent storage
     * Called automatically during initialization
     * @return true if parameters loaded successfully, false if using defaults
     */
    public boolean loadBoostParametersFromFile() {
        double[] params = ShooterBoostConfig.loadAllLearnedParameters();
        if (params != null && params.length >= 6) {
            shotBoostDelay = params[0];
            shotBoostDuration = params[1];
            // Cap boost power to max 15% to prevent overshoot on second shot
            shotBoostPower = Math.min(0.15, params[2]);
            learnedShotInterval = params[3];
            // Ignore learned RPM tolerance - we enforce strict 15 RPM in isShooterReady()
            learnedRapidFireTolerance = params[4];
            shotsAnalyzed = (int) params[5];
            return true;
        }
        return false;
    }

    /**
     * Save all learned parameters to persistent storage
     * Call this periodically during operation or at end of session
     * @return true if save successful, false otherwise
     */
    public boolean saveBoostParametersToFile() {
        return ShooterBoostConfig.saveAllLearnedParameters(
            shotBoostDelay,
            shotBoostDuration,
            shotBoostPower,
            learnedShotInterval,
            learnedRapidFireTolerance,
            shotsAnalyzed
        );
    }

    /**
     * Check if saved configuration exists
     * @return true if previously learned parameters are available
     */
    public boolean hasSavedConfig() {
        return ShooterBoostConfig.configExists();
    }

    /**
     * Delete saved configuration and reset to defaults
     * @return true if reset successful
     */
    public boolean resetSavedConfig() {
        boolean deleted = ShooterBoostConfig.deleteSavedConfig();
        if (deleted) {
            // Reset to default values
            shotBoostDelay = 0.020;
            shotBoostDuration = 0.180;
            shotBoostPower = 0.18;
            shotsAnalyzed = 0;
        }
        return deleted;
    }
}

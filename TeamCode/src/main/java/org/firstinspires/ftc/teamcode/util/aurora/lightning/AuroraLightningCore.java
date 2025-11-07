package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.SmartMechanumDrive;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanner;
import org.firstinspires.ftc.teamcode.util.tool.Pose;

import java.util.List;

/**
 * AuroraLightningCore - High-level autonomous control system
 *
 * Integrates PoseController, PositionManager, and odometry for advanced
 * autonomous navigation with waypoint following, path planning, and intelligent
 * movement features.
 *
 * Features:
 * - Advanced PID-based position control with feed-forward
 * - Waypoint navigation with automatic advancement
 * - Path planning with multiple modes (straight, smooth, spline)
 * - Overshoot detection and correction
 * - Pause points in paths
 * - Non-blocking operation
 */
public class AuroraLightningCore {

    // ======== Core Systems ========
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private AuroraManager robotManager;
    private SmartMechanumDrive driveSystem;
    private PositionManager positionManager;
    private PoseController poseController;
    private PathPlanner pathPlanner;
    private EnhancedDecodeHelper shooter;  // Optional shooter system

    // ======== Timing ========
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime operationTimer = new ElapsedTime();

    // ======== State Management ========
    private boolean isInitialized = false;
    private boolean emergencyStop = false;
    private FieldMap.Alliance alliance;

    // ======== Default Configuration ========
    // Position control gains (can be tuned per robot)
    public double defaultKP_X = 0.06;
    public double defaultKP_Y = 0.06;
    public double defaultKP_Theta = 0.03;

    // Velocity feed-forward gains
    public double defaultKV_X = 0.0;
    public double defaultKV_Y = 0.0;
    public double defaultKV_Theta = 0.0;

    // Tolerances
    public double defaultPositionTolerance = 2.0;  // inches
    public double defaultAngleTolerance = 5.0;     // degrees

    // Path planning
    public PathPlanner.PathMode defaultPathMode = PathPlanner.PathMode.SMOOTH_CURVE;
    public double defaultSmoothingFactor = 0.7;

    /**
     * Constructor - Initialize AuroraLightningCore
     *
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param alliance Alliance color (RED or BLUE)
     */
    public AuroraLightningCore(HardwareMap hardwareMap, Telemetry telemetry, FieldMap.Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    /**
     * Alternative constructor using AuroraManager
     *
     * @param robotManager Existing AuroraManager instance
     * @param alliance Alliance color
     */
    public AuroraLightningCore(AuroraManager robotManager, FieldMap.Alliance alliance) {
        this.robotManager = robotManager;
        this.driveSystem = robotManager.getDriveSystem();
        this.alliance = alliance;

        // Extract hardwareMap and telemetry from robotManager
        this.hardwareMap = robotManager.getHardwareMap();
        this.telemetry = robotManager.getTelemetry();
    }

    /**
     * Initialize all subsystems
     * Call this in your OpMode init()
     *
     * @return true if initialization successful
     */
    public boolean initialize() {
        try {
            telemetry.addData("AuroraLightning", "Initializing...");
            telemetry.update();

            // Initialize drive system if not already provided
            if (driveSystem == null && robotManager != null) {
                driveSystem = robotManager.getDriveSystem();
            }

            // Initialize position manager
            positionManager = new PositionManager();
            positionManager.LightningPositioningManager(hardwareMap, telemetry, alliance);

            // Initialize pose controller
            poseController = new PoseController(driveSystem, positionManager);

            // Configure default gains
            poseController.setAllGains(
                defaultKP_X, defaultKP_Y, defaultKP_Theta,
                defaultKV_X, defaultKV_Y, defaultKV_Theta
            );

            // Set default tolerances
            poseController.posTolerance = defaultPositionTolerance;
            poseController.angTolerance = Math.toRadians(defaultAngleTolerance);

            // Initialize path planner
            pathPlanner = new PathPlanner();
            pathPlanner.smoothingFactor = defaultSmoothingFactor;

            // Initialize shooter (optional - may fail if hardware not present)
            try {
                shooter = new EnhancedDecodeHelper(hardwareMap);
                telemetry.addData("Shooter", "Initialized");
            } catch (Exception e) {
                shooter = null;
                telemetry.addData("Shooter", "Not available");
            }

            // Reset runtime
            runtime.reset();

            isInitialized = true;
            telemetry.addData("AuroraLightning", "Initialized successfully!");
            telemetry.update();

            return true;

        } catch (Exception e) {
            telemetry.addData("AuroraLightning", "Init failed: " + e.getMessage());
            telemetry.update();
            isInitialized = false;
            return false;
        }
    }

    /**
     * Main update loop - MUST be called every iteration
     * Non-blocking - updates all subsystems
     */
    public void update() {
        if (!isInitialized) return;

        if (emergencyStop) {
            poseController.stopPath();
            driveSystem.setDriveInputs(0, 0, 0);
            driveSystem.update();
            return;
        }

        // Update pose controller (handles position tracking and path following)
        poseController.update();
    }

    // ======== Position Control Methods ========

    /**
     * Move to absolute field position with specific heading
     * Non-blocking - call update() in loop
     *
     * @param x Target X position (inches)
     * @param y Target Y position (inches)
     * @param heading Target heading (degrees)
     */
    public void moveTo(double x, double y, double heading) {
        if (!isInitialized) return;
        poseController.setTarget(x, y, heading);
    }

    /**
     * Move to position and face that point
     * Non-blocking - call update() in loop
     *
     * @param x Target X position (inches)
     * @param y Target Y position (inches)
     */
    public void moveToFacePoint(double x, double y) {
        if (!isInitialized) return;
        poseController.setTargetFacePoint(x, y);
    }

    /**
     * Move forward/backward relative to robot's current orientation
     * Non-blocking - call update() in loop
     *
     * @param inches Distance to move (positive = forward, negative = backward)
     */
    public void moveAxial(double inches) {
        if (!isInitialized) return;
        poseController.moveAxial(inches);
    }

    /**
     * Strafe left/right relative to robot's current orientation
     * Non-blocking - call update() in loop
     *
     * @param inches Distance to strafe (positive = left, negative = right)
     */
    public void moveLateral(double inches) {
        if (!isInitialized) return;
        poseController.moveLateral(inches);
    }

    /**
     * Rotate relative to current heading
     * Non-blocking - call update() in loop
     *
     * @param degrees Angle to rotate (positive = CCW, negative = CW)
     */
    public void rotate(double degrees) {
        if (!isInitialized) return;
        poseController.rotate(degrees);
    }

    /**
     * Move relative to robot's current pose
     * Non-blocking - call update() in loop
     *
     * @param axialInches Forward/back offset (positive = forward)
     * @param lateralInches Left/right offset (positive = left)
     * @param headingDegrees Rotation offset (positive = CCW)
     */
    public void moveRelative(double axialInches, double lateralInches, double headingDegrees) {
        if (!isInitialized) return;
        poseController.moveRelative(axialInches, lateralInches, headingDegrees);
    }

    /**
     * Hold current position
     */
    public void holdPosition() {
        if (!isInitialized) return;
        poseController.holdPosition();
    }

    /**
     * Stop all movement and clear target
     */
    public void stop() {
        if (!isInitialized) return;
        poseController.clearTarget();
        poseController.stopPath();
    }

    // ======== Path Planning and Waypoint Navigation ========

    /**
     * Plan and follow a path through waypoints
     * Non-blocking - call update() in loop
     *
     * @param waypoints List of waypoints to follow
     * @param mode Path generation mode (STRAIGHT, SMOOTH_CURVE, CUBIC_SPLINE)
     */
    public void followPath(List<Pose> waypoints, PathPlanner.PathMode mode) {
        if (!isInitialized || waypoints == null || waypoints.isEmpty()) return;

        // Generate detailed path
        List<Pose> detailedPath = pathPlanner.generatePath(waypoints, mode);

        // Load into pose controller
        poseController.setPath(detailedPath);
        poseController.startPath();
    }

    /**
     * Follow a path using default path mode
     *
     * @param waypoints List of waypoints to follow
     */
    public void followPath(List<Pose> waypoints) {
        followPath(waypoints, defaultPathMode);
    }

    /**
     * Add a single waypoint to the current path
     *
     * @param x X position (inches)
     * @param y Y position (inches)
     * @param heading Heading (degrees)
     */
    public void addWaypoint(double x, double y, double heading) {
        if (!isInitialized) return;
        poseController.addWaypoint(x, y, heading);
    }

    /**
     * Clear all waypoints
     */
    public void clearWaypoints() {
        if (!isInitialized) return;
        poseController.clearWaypoints();
    }

    /**
     * Start following the loaded path
     */
    public void startPath() {
        if (!isInitialized) return;
        poseController.startPath();
    }

    /**
     * Start following the path in reverse
     */
    public void startPathReverse() {
        if (!isInitialized) return;
        poseController.startPathReverse();
    }

    /**
     * Stop path following
     */
    public void stopPath() {
        if (!isInitialized) return;
        poseController.stopPath();
    }

    /**
     * Pause path following
     */
    public void pausePath() {
        if (!isInitialized) return;
        poseController.pausePath();
    }

    /**
     * Resume path following
     */
    public void resumePath() {
        if (!isInitialized) return;
        poseController.resumePath();
    }

    /**
     * Execute only a portion of the path
     *
     * @param startPercent Start percentage (0.0 to 1.0)
     * @param endPercent End percentage (0.0 to 1.0)
     */
    public void setPathRange(double startPercent, double endPercent) {
        if (!isInitialized) return;
        poseController.setPathRange(startPercent, endPercent);
    }

    /**
     * Add a pause at a specific waypoint
     *
     * @param waypointIndex Index of waypoint to pause at
     * @param durationSeconds Duration to pause (0 = indefinite)
     */
    public void addPauseAtWaypoint(int waypointIndex, double durationSeconds) {
        if (!isInitialized) return;
        poseController.addPauseAtWaypoint(waypointIndex, durationSeconds);
    }

    // ======== Status and Query Methods ========

    /**
     * Check if robot is at target position
     *
     * @return true if at target (within tolerance)
     */
    public boolean atTarget() {
        if (!isInitialized) return false;
        return poseController.atTarget();
    }

    /**
     * Check if currently following a path
     *
     * @return true if path is active
     */
    public boolean isFollowingPath() {
        if (!isInitialized) return false;
        return poseController.isFollowingPath();
    }

    /**
     * Check if path is complete
     *
     * @return true if all waypoints visited
     */
    public boolean isPathComplete() {
        if (!isInitialized) return false;
        return poseController.isPathComplete();
    }

    /**
     * Get progress through current path
     *
     * @return Progress percentage (0.0 to 1.0)
     */
    public double getPathProgress() {
        if (!isInitialized) return 0.0;
        return poseController.getPathProgress();
    }

    /**
     * Get current robot position
     *
     * @return [x, y, heading] array
     */
    public double[] getCurrentPosition() {
        if (!isInitialized) return new double[]{0, 0, 0};
        return positionManager.getCurrentPosition();
    }

    /**
     * Get current robot velocity
     *
     * @return [vx, vy, omega] array
     */
    public double[] getCurrentVelocity() {
        if (!isInitialized) return new double[]{0, 0, 0};
        return poseController.getRobotVelocity();
    }

    // ======== Configuration Methods ========

    /**
     * Configure position control gains
     *
     * @param kP_x X-axis proportional gain
     * @param kP_y Y-axis proportional gain
     * @param kP_theta Heading proportional gain
     */
    public void setPositionGains(double kP_x, double kP_y, double kP_theta) {
        if (!isInitialized) return;
        poseController.kP_x = kP_x;
        poseController.kP_y = kP_y;
        poseController.kP_theta = kP_theta;
    }

    /**
     * Configure velocity feed-forward gains
     *
     * @param kV_x X-axis velocity gain
     * @param kV_y Y-axis velocity gain
     * @param kV_theta Heading velocity gain
     */
    public void setVelocityGains(double kV_x, double kV_y, double kV_theta) {
        if (!isInitialized) return;
        poseController.kV_x = kV_x;
        poseController.kV_y = kV_y;
        poseController.kV_theta = kV_theta;
    }

    /**
     * Set position tolerance
     *
     * @param inches Distance tolerance in inches
     */
    public void setPositionTolerance(double inches) {
        if (!isInitialized) return;
        poseController.posTolerance = inches;
    }

    /**
     * Set heading tolerance
     *
     * @param degrees Angle tolerance in degrees
     */
    public void setHeadingTolerance(double degrees) {
        if (!isInitialized) return;
        poseController.angTolerance = Math.toRadians(degrees);
    }

    /**
     * Enable or disable overshoot detection
     *
     * @param enable true to enable
     */
    public void setOvershootDetection(boolean enable) {
        if (!isInitialized) return;
        poseController.enableOvershootDetection = enable;
    }

    /**
     * Set the path planning mode
     *
     * @param mode Path generation mode
     */
    public void setPathMode(PathPlanner.PathMode mode) {
        this.defaultPathMode = mode;
    }

    /**
     * Set path smoothing factor (for SMOOTH_CURVE mode)
     *
     * @param factor Smoothing factor (0.0 to 1.0)
     */
    public void setSmoothingFactor(double factor) {
        if (pathPlanner != null) {
            pathPlanner.smoothingFactor = factor;
        }
    }

    /**
     * Trigger emergency stop
     */
    public void emergencyStop() {
        emergencyStop = true;
    }

    /**
     * Clear emergency stop
     */
    public void clearEmergencyStop() {
        emergencyStop = false;
    }

    // ======== Shooter Control Methods ========

    /**
     * Check if shooter system is available
     *
     * @return true if shooter hardware is initialized
     */
    public boolean hasShooter() {
        return shooter != null;
    }

    /**
     * Spin up shooter to target RPM with warmup
     * Non-blocking - shooter will maintain RPM in background
     *
     * @param preset Shooter preset (HIGH_BASKET, LOW_BASKET, etc.)
     */
    public void startShooter(ShooterConfig.ShooterPreset preset) {
        if (shooter == null) return;
        shooter.getConfig().setPreset(preset);
        shooter.startShooter();
    }

    /**
     * Start shooter in warmup mode (reduced RPM to save power)
     * Useful during autonomous setup or between scoring runs
     *
     * @param preset Shooter preset to warm up
     */
    public void startShooterWarmup(ShooterConfig.ShooterPreset preset) {
        if (shooter == null) return;
        shooter.handleWarmupButton(true, preset);
    }

    /**
     * Stop shooter warmup mode
     */
    public void stopShooterWarmup() {
        if (shooter == null) return;
        shooter.handleWarmupButton(false, ShooterConfig.ShooterPreset.LONG_RANGE);
    }

    /**
     * Fire a single shot when shooter is ready
     * Non-blocking - returns true when shot is fired
     *
     * @return true if shot was fired this call
     */
    public boolean fireShot() {
        if (shooter == null) return false;
        return shooter.fireSingleShot();
    }

    /**
     * Fire multiple shots in sequence (blocking)
     * Maintains consistent RPM between shots
     *
     * @param numShots Number of shots to fire
     * @param preset Shooter preset to use
     * @param keepRunning Keep shooter spinning after last shot
     */
    public void fireShots(int numShots, ShooterConfig.ShooterPreset preset, boolean keepRunning) {
        if (shooter == null) return;
        shooter.autoShootSmart(numShots, keepRunning, preset);
    }

    /**
     * Check if shooter is at target RPM and ready to fire
     *
     * @return true if shooter is ready
     */
    public boolean isShooterReady() {
        if (shooter == null) return false;
        return shooter.isShooterReady();
    }

    /**
     * Stop the shooter motor
     */
    public void stopShooter() {
        if (shooter == null) return;
        shooter.stopShooter();
    }

    /**
     * Get current shooter RPM
     *
     * @return Current RPM (0 if not running)
     */
    public double getShooterRPM() {
        if (shooter == null) return 0;
        return shooter.getCurrentRPM();
    }

    /**
     * Get target shooter RPM based on current mode
     *
     * @return Target RPM
     */
    public double getShooterTargetRPM() {
        if (shooter == null) return 0;
        return shooter.getTargetRPM();
    }

    /**
     * Check if shooter is currently running
     *
     * @return true if shooter motor is spinning
     */
    public boolean isShooterRunning() {
        if (shooter == null) return false;
        return shooter.isShooterRunning();
    }

    /**
     * Check if shooter is in warmup mode
     *
     * @return true if in warmup mode
     */
    public boolean isShooterWarmup() {
        if (shooter == null) return false;
        return shooter.isWarmupMode();
    }

    /**
     * Autonomous sequence: Move to position and shoot
     * Combines navigation with shooting in one operation
     *
     * @param x Target X position
     * @param y Target Y position
     * @param heading Target heading
     * @param shooterPreset Shooter configuration
     * @param numShots Number of shots to fire
     * @param warmupDuringMove Start warming up shooter during movement
     */
    public void moveAndShoot(double x, double y, double heading,
                           ShooterConfig.ShooterPreset shooterPreset,
                           int numShots, boolean warmupDuringMove) {
        if (!isInitialized) return;

        // Start moving to position
        moveTo(x, y, heading);

        // Optionally start warming up during movement
        if (warmupDuringMove && shooter != null) {
            startShooterWarmup(shooterPreset);
        }

        // Wait until at position
        while (!atTarget() && !emergencyStop) {
            update();

            // Continue warmup if enabled
            if (warmupDuringMove && shooter != null) {
                shooter.handleWarmupButton(true, shooterPreset);
            }
        }

        // Stop warmup and start full power shooter
        if (warmupDuringMove && shooter != null) {
            stopShooterWarmup();
        }

        // Fire shots
        if (shooter != null && !emergencyStop) {
            fireShots(numShots, shooterPreset, false);
        }
    }

    /**
     * Autonomous sequence: Follow path and shoot at waypoints
     * Specify which waypoints to shoot at
     *
     * @param waypoints Path to follow
     * @param mode Path generation mode
     * @param shootAtIndices Waypoint indices to shoot at (e.g., [2, 5, 8])
     * @param shooterPreset Shooter configuration
     * @param shotsPerWaypoint Number of shots at each shooting waypoint
     */
    public void followPathAndShoot(List<Pose> waypoints, PathPlanner.PathMode mode,
                                   int[] shootAtIndices,
                                   ShooterConfig.ShooterPreset shooterPreset,
                                   int shotsPerWaypoint) {
        if (!isInitialized || shooter == null || waypoints == null) return;

        // Generate and load path
        List<Pose> detailedPath = pathPlanner.generatePath(waypoints, mode);
        poseController.setPath(detailedPath);

        // Add pause points at shooting waypoints
        for (int index : shootAtIndices) {
            if (index >= 0 && index < detailedPath.size()) {
                poseController.addPauseAtWaypoint(index, 0); // Indefinite pause
            }
        }

        // Start path and warmup
        poseController.startPath();
        startShooterWarmup(shooterPreset);

        int currentShootIndex = 0;

        // Execute path with shooting
        while (!isPathComplete() && !emergencyStop) {
            update();

            // Maintain warmup during movement
            shooter.handleWarmupButton(true, shooterPreset);

            // Check if paused at a shooting waypoint
            if (poseController.isPausedAtWaypoint() && currentShootIndex < shootAtIndices.length) {
                // Stop warmup, fire shots
                stopShooterWarmup();
                fireShots(shotsPerWaypoint, shooterPreset, false);

                // Resume path and restart warmup
                poseController.resumeFromPause();
                startShooterWarmup(shooterPreset);
                currentShootIndex++;
            }
        }

        // Stop shooter and warmup
        stopShooterWarmup();
        stopShooter();
    }

    // ======== Accessors ========

    /**
     * Get the PoseController instance for advanced control
     *
     * @return PoseController instance
     */
    public PoseController getPoseController() {
        return poseController;
    }

    /**
     * Get the PositionManager instance
     *
     * @return PositionManager instance
     */
    public PositionManager getPositionManager() {
        return positionManager;
    }

    /**
     * Get the PathPlanner instance
     *
     * @return PathPlanner instance
     */
    public PathPlanner getPathPlanner() {
        return pathPlanner;
    }

    /**
     * Get the EnhancedDecodeHelper (shooter) instance
     *
     * @return Shooter instance, or null if not available
     */
    public EnhancedDecodeHelper getShooter() {
        return shooter;
    }

    /**
     * Check if system is initialized
     *
     * @return true if ready to use
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Get runtime since initialization
     *
     * @return Elapsed time in seconds
     */
    public double getRuntime() {
        return runtime.seconds();
    }

    // ======== Telemetry Methods ========

    /**
     * Add comprehensive telemetry data for debugging
     */
    public void addTelemetry() {
        if (!isInitialized) {
            telemetry.addData("AuroraLightning", "NOT INITIALIZED");
            return;
        }

        telemetry.addData("=== AURORA LIGHTNING ===", "");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());

        double[] pos = getCurrentPosition();
        telemetry.addData("Position", "(%.1f, %.1f) @ %.1f°", pos[0], pos[1], pos[2]);

        double[] vel = getCurrentVelocity();
        telemetry.addData("Velocity", "X:%.1f Y:%.1f Ω:%.1f", vel[0], vel[1], vel[2]);

        telemetry.addData("At Target", atTarget() ? "YES" : "NO");

        if (isFollowingPath()) {
            telemetry.addData("Path Progress", "%.0f%% (%d/%d waypoints)",
                getPathProgress() * 100,
                poseController.getTotalWaypoints() - poseController.getRemainingWaypoints(),
                poseController.getTotalWaypoints());
        }

        // Shooter status
        if (shooter != null) {
            telemetry.addData("--- Shooter ---", "");

            String shooterMode = "OFF";
            if (isShooterWarmup()) {
                shooterMode = "WARMUP";
            } else if (isShooterRunning()) {
                shooterMode = "ACTIVE";
            }
            telemetry.addData("Shooter Mode", shooterMode);

            if (isShooterRunning() || isShooterWarmup()) {
                double currentRpm = getShooterRPM();
                double targetRpm = getShooterTargetRPM();
                telemetry.addData("RPM", "%.0f / %.0f %s",
                    currentRpm, targetRpm,
                    isShooterReady() ? "✓" : "...");

                double rpmError = Math.abs(currentRpm - targetRpm);
                telemetry.addData("RPM Error", "%.0f", rpmError);
            }

            if (shooter.isShooting()) {
                telemetry.addData("Status", "🔥 FIRING");
            }
        }

        if (emergencyStop) {
            telemetry.addData("STATUS", "⚠️ EMERGENCY STOP ⚠️");
        }
    }
}


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aurora.vision.AuroraAprilTagLocalizer;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Predicate;

/**
 * PedroAutonomousBuilder - Simplified state machine builder for Pedro Pathing autonomous
 *
 * Features:
 * - Easy path sequencing with automatic state management
 * - Action insertion between paths (shooting, mechanisms, etc.)
 * - Continuous position tracking during actions
 * - Automatic shooter integration with EnhancedDecodeHelper
 * - Heading alignment actions
 * - Pause/resume functionality
 *
 * Example Usage:
 * <pre>
 * builder.addPath(paths.Path1)
 *        .addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)
 *        .addPath(paths.Path2)
 *        .addTurnToHeading(Math.toRadians(90))
 *        .addCustomAction("Deploy Intake", (follower, time) -> {
 *            // Custom mechanism code here
 *            return time > 1.0; // Complete after 1 second
 *        })
 *        .addPath(paths.Path3);
 * </pre>
 */
public class PedroAutonomousBuilder {

    // State machine components
    private final List<AutonomousStep> steps;
    private final Follower follower;
    private EnhancedDecodeHelper shooter;
    private AuroraAprilTagLocalizer aprilTagLocalizer;

    private int currentStepIndex;
    private double stepStartTime;
    private boolean isExecuting;

    // Connector path configuration
    private double reconnectDistanceThreshold = 6.0; // inches
    private double reconnectHeadingThreshold = Math.toRadians(20); // radians
    private boolean autoConnectEnabled = true;

    // Starting pose configuration
    private Pose defaultStartPose = new Pose(0, 0, 0);
    private boolean useAprilTagForStart = false;
    private double aprilTagDetectionTimeout = 2.0; // seconds
    private double minAprilTagConfidence = 0.5;

    /**
     * Create a new autonomous builder
     * @param follower Pedro Pathing follower instance
     */
    public PedroAutonomousBuilder(Follower follower) {
        this.follower = follower;
        this.steps = new ArrayList<>();
        this.currentStepIndex = 0;
        this.stepStartTime = 0;
        this.isExecuting = false;
    }

    /**
     * Set the shooter instance for shooting actions
     * @param shooter EnhancedDecodeHelper instance
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder withShooter(EnhancedDecodeHelper shooter) {
        this.shooter = shooter;
        return this;
    }

    /**
     * Set the AprilTag localizer for position detection
     * @param localizer AuroraAprilTagLocalizer instance
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder withAprilTagLocalizer(AuroraAprilTagLocalizer localizer) {
        this.aprilTagLocalizer = localizer;
        return this;
    }

    /**
     * Add a path to follow
     * @param path PathChain to follow
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addPath(PathChain path) {
        steps.add(new PathStep(path, "Path"));
        return this;
    }

    /**
     * Add a path to follow with automatic connector insertion
     * Automatically inserts a connector path if robot pose differs significantly from path start
     * @param path PathChain to follow
     * @param pathStartPose The starting pose of this path (needed for connector generation)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addPath(PathChain path, Pose pathStartPose) {
        if (autoConnectEnabled) {
            // Check if connector is needed and add it
            PathChain connector = createConnectorIfNeeded(pathStartPose);
            if (connector != null) {
                steps.add(new PathStep(connector, "Connector"));
            }
        }
        steps.add(new PathStep(path, "Path"));
        return this;
    }

    /**
     * Add a shooting action using the EnhancedDecodeHelper
     * @param numShots Number of shots to fire
     * @param preset Shooter preset to use
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addShootAction(int numShots, ShooterConfig.ShooterPreset preset) {
        if (shooter == null) {
            throw new IllegalStateException("Shooter not configured. Call withShooter() first.");
        }
        steps.add(new ShootAction(shooter, numShots, preset));
        return this;
    }

    /**
     * Add a turn to specific heading action
     * @param targetHeading Target heading in radians
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addTurnToHeading(double targetHeading) {
        steps.add(new TurnToHeadingAction(targetHeading));
        return this;
    }

    /**
     * Add a wait/pause action
     * @param durationSeconds How long to wait in seconds
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addWait(double durationSeconds) {
        steps.add(new WaitAction(durationSeconds));
        return this;
    }

    /**
     * Add a custom action with a lambda function
     * @param name Action name for telemetry
     * @param action BiConsumer that takes (follower, elapsedTime) and returns true when complete
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addCustomAction(String name, ActionFunction action) {
        steps.add(new CustomAction(name, action));
        return this;
    }

    /**
     * Add a conditional branch - executes different paths based on a condition
     * @param condition Predicate that evaluates the condition
     * @param trueBuilder Builder to execute if condition is true
     * @param falseBuilder Builder to execute if condition is false
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder addConditional(Predicate<Follower> condition,
                                                  PedroAutonomousBuilder trueBuilder,
                                                  PedroAutonomousBuilder falseBuilder) {
        steps.add(new ConditionalStep(condition, trueBuilder, falseBuilder));
        return this;
    }

    /**
     * Configure the distance threshold for auto-connecting paths
     * @param thresholdInches Distance threshold in inches (default: 6.0)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setReconnectDistanceThreshold(double thresholdInches) {
        this.reconnectDistanceThreshold = thresholdInches;
        return this;
    }

    /**
     * Configure the heading threshold for auto-connecting paths
     * @param thresholdRadians Heading threshold in radians (default: 20 degrees)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setReconnectHeadingThreshold(double thresholdRadians) {
        this.reconnectHeadingThreshold = thresholdRadians;
        return this;
    }

    /**
     * Enable or disable automatic connector path insertion
     * @param enabled true to auto-insert connectors (default: true)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setAutoConnectEnabled(boolean enabled) {
        this.autoConnectEnabled = enabled;
        return this;
    }

    /**
     * Set the default starting pose (fallback if AprilTag detection fails)
     * @param x X coordinate in inches
     * @param y Y coordinate in inches
     * @param heading Heading in radians
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setDefaultStartPose(double x, double y, double heading) {
        this.defaultStartPose = new Pose(x, y, heading);
        return this;
    }

    /**
     * Set the default starting pose (fallback if AprilTag detection fails)
     * @param pose Starting pose
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setDefaultStartPose(Pose pose) {
        this.defaultStartPose = pose;
        return this;
    }

    /**
     * Enable AprilTag-based starting pose detection
     * @param enabled true to use AprilTag for starting pose (default: false)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setUseAprilTagForStart(boolean enabled) {
        this.useAprilTagForStart = enabled;
        return this;
    }

    /**
     * Set the timeout for AprilTag detection at start
     * @param timeoutSeconds Maximum time to wait for AprilTag detection (default: 2.0)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setAprilTagDetectionTimeout(double timeoutSeconds) {
        this.aprilTagDetectionTimeout = timeoutSeconds;
        return this;
    }

    /**
     * Set minimum confidence threshold for AprilTag starting pose
     * @param confidence Minimum confidence value 0.0-1.0 (default: 0.5)
     * @return this builder for chaining
     */
    public PedroAutonomousBuilder setMinAprilTagConfidence(double confidence) {
        this.minAprilTagConfidence = Math.max(0.0, Math.min(1.0, confidence));
        return this;
    }

    /**
     * Get the starting pose for the robot
     * If AprilTag detection is enabled and configured, attempts to get pose from AprilTags
     * Falls back to default pose if AprilTag detection fails or times out
     *
     * @return Starting pose (either from AprilTag or default)
     */
    public Pose getStartingPose() {
        if (!useAprilTagForStart || aprilTagLocalizer == null) {
            return defaultStartPose;
        }

        // Try to get pose from AprilTag with timeout
        double startTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = 0;

        while (elapsedTime < aprilTagDetectionTimeout) {
            // Update AprilTag localizer
            aprilTagLocalizer.updatePosition();

            // Check if we have a valid position with sufficient confidence
            if (aprilTagLocalizer.hasValidPosition() &&
                aprilTagLocalizer.getPositionConfidence() >= minAprilTagConfidence) {

                // Get position in path follower coordinates
                double[] position = aprilTagLocalizer.getCurrentPositionPathFollower();
                return new Pose(position[0], position[1], position[2]);
            }

            // Small delay to avoid busy waiting
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }

            elapsedTime = (System.currentTimeMillis() / 1000.0) - startTime;
        }

        // Timeout or no valid detection - use default
        return defaultStartPose;
    }

    /**
     * Initialize the robot starting pose
     * Call this before start() to set the initial position
     * Uses AprilTag detection if enabled, otherwise uses default pose
     *
     * @return The pose that was set (for verification)
     */
    public Pose initializeStartingPose() {
        Pose startPose = getStartingPose();
        follower.setStartingPose(startPose);
        return startPose;
    }

    /**
     * Create a connector path from the robot's current pose to smoothly transition
     * Used internally to smooth transitions when paths are called out of sequence or
     * the starting position differs significantly from the next path's start.
     *
     * Since PathChain doesn't expose getStartPose(), we use a helper that accepts
     * the target starting pose explicitly.
     *
     * @param targetStartPose The intended start pose of the target path
     * @return A connector PathChain if needed, or null if current pose is already close enough
     */
    private PathChain createConnectorIfNeeded(Pose targetStartPose) {
        Pose currentPose = follower.getPose();

        // Calculate distance between current and target start
        double dx = currentPose.getX() - targetStartPose.getX();
        double dy = currentPose.getY() - targetStartPose.getY();
        double distance = Math.hypot(dx, dy);

        // Calculate heading difference (normalized to [-PI, PI])
        double headingDiff = normalizeAngle(currentPose.getHeading() - targetStartPose.getHeading());
        double headingError = Math.abs(headingDiff);

        // Check if connector is needed
        if (distance < reconnectDistanceThreshold && headingError < reconnectHeadingThreshold) {
            return null; // Already close enough, no connector needed
        }

        // Build a smooth Bezier curve from current pose to target start pose
        // Use midpoint as control point for smooth transition
        double midX = (currentPose.getX() + targetStartPose.getX()) * 0.5;
        double midY = (currentPose.getY() + targetStartPose.getY()) * 0.5;

        return follower
            .pathBuilder()
            .addPath(new BezierCurve(
                new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                new Pose(midX, midY), // Control point for smooth curve
                new Pose(targetStartPose.getX(), targetStartPose.getY(), targetStartPose.getHeading())
            ))
            .setLinearHeadingInterpolation(currentPose.getHeading(), targetStartPose.getHeading())
            .build();
    }

    /**
     * Create a connector path from the robot's current pose to a target path's start pose
     * This version is kept for backward compatibility when you have the target start pose.
     * For automatic connector insertion with addPath(), use the Pose-based version.
     *
     * @param targetPath The target path to connect to
     * @param targetStartPose The start pose of the target path
     * @return A connector PathChain if needed, or null if current pose is already close enough
     */
    public PathChain createConnectorIfNeeded(PathChain targetPath, Pose targetStartPose) {
        return createConnectorIfNeeded(targetStartPose);
    }

    /**
     * Normalize an angle to the range [-PI, PI]
     * @param angle Angle in radians
     * @return Normalized angle in radians
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Start executing the autonomous sequence
     */
    public void start() {
        if (steps.isEmpty()) {
            return;
        }
        currentStepIndex = 0;
        stepStartTime = System.currentTimeMillis() / 1000.0;
        isExecuting = true;
        steps.get(0).onStart(follower, stepStartTime);
    }

    /**
     * Update the autonomous - call this in your loop() method
     * IMPORTANT: Also call follower.update() before this
     * @return Current step name for telemetry
     */
    public String update() {
        if (!isExecuting || currentStepIndex >= steps.size()) {
            return "FINISHED";
        }

        // Always update follower position tracking
        follower.update();

        AutonomousStep currentStep = steps.get(currentStepIndex);
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - stepStartTime;

        // Update current step
        boolean isComplete = currentStep.update(follower, elapsedTime);

        if (isComplete) {
            // Move to next step
            currentStepIndex++;
            if (currentStepIndex < steps.size()) {
                stepStartTime = currentTime;
                steps.get(currentStepIndex).onStart(follower, stepStartTime);
            } else {
                isExecuting = false;
            }
        }

        return currentStep.getName();
    }

    /**
     * Check if autonomous is finished
     * @return true if all steps are complete
     */
    public boolean isFinished() {
        return !isExecuting || currentStepIndex >= steps.size();
    }

    /**
     * Get current step index
     * @return Current step index (0-based)
     */
    public int getCurrentStepIndex() {
        return currentStepIndex;
    }

    /**
     * Get total number of steps
     * @return Total step count
     */
    public int getTotalSteps() {
        return steps.size();
    }

    /**
     * Get current robot pose (always tracked)
     * @return Current pose
     */
    public Pose getCurrentPose() {
        return follower.getPose();
    }

    // ==================== STEP DEFINITIONS ====================

    /**
     * Base interface for autonomous steps
     */
    private interface AutonomousStep {
        void onStart(Follower follower, double startTime);
        boolean update(Follower follower, double elapsedTime);
        String getName();
    }

    /**
     * Path following step
     */
    private static class PathStep implements AutonomousStep {
        private final PathChain path;
        private String name;

        public PathStep(PathChain path) {
            this.path = path;
            this.name = "Path";
        }

        public PathStep(PathChain path, String name) {
            this.path = path;
            this.name = name;
        }

        @Override
        public void onStart(Follower follower, double startTime) {
            follower.followPath(path);
        }

        @Override
        public boolean update(Follower follower, double elapsedTime) {
            return !follower.isBusy();
        }

        @Override
        public String getName() {
            return name;
        }
    }

    /**
     * Shooting action step using EnhancedDecodeHelper
     */
    private static class ShootAction implements AutonomousStep {
        private final EnhancedDecodeHelper shooter;
        private final int numShots;
        private final ShooterConfig.ShooterPreset preset;
        private boolean started;
        private int shotsFired;
        private double lastShotTime;

        public ShootAction(EnhancedDecodeHelper shooter, int numShots, ShooterConfig.ShooterPreset preset) {
            this.shooter = shooter;
            this.numShots = numShots;
            this.preset = preset;
        }

        @Override
        public void onStart(Follower follower, double startTime) {
            started = false;
            shotsFired = 0;
            lastShotTime = 0;
            shooter.getConfig().setPreset(preset);
            shooter.startShooter();
        }

        @Override
        public boolean update(Follower follower, double elapsedTime) {
            // Keep updating follower position even while shooting
            // This is important for accurate localization

            // Check if shooter is ready
            if (!started && shooter.isShooterReady()) {
                started = true;
            }

            // Fire shots
            if (started && shotsFired < numShots) {
                if (shooter.fireSingleShot()) {
                    shotsFired++;
                    lastShotTime = elapsedTime;
                }
            }

            // Complete when all shots fired and feeding is done
            boolean allShotsFired = shotsFired >= numShots;
            boolean notCurrentlyShooting = !shooter.isShooting();
            boolean enoughTimeElapsed = elapsedTime > lastShotTime + 0.5;

            if (allShotsFired && notCurrentlyShooting && enoughTimeElapsed) {
                shooter.stopShooter();
                return true;
            }

            return false;
        }

        @Override
        public String getName() {
            return "Shooting (" + shotsFired + "/" + numShots + ")";
        }
    }

    /**
     * Turn to heading action
     */
    private static class TurnToHeadingAction implements AutonomousStep {
        private final double targetHeading;
        private static final double HEADING_TOLERANCE = Math.toRadians(2); // 2 degrees

        public TurnToHeadingAction(double targetHeading) {
            this.targetHeading = targetHeading;
        }

        @Override
        public void onStart(Follower follower, double startTime) {
            // Create a point turn path to target heading
            Pose currentPose = follower.getPose();
            follower.followPath(
                follower.pathBuilder()
                    .addPath(new com.pedropathing.geometry.BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                        new Pose(currentPose.getX(), currentPose.getY(), targetHeading)
                    ))
                    .setConstantHeadingInterpolation(targetHeading)
                    .build()
            );
        }

        @Override
        public boolean update(Follower follower, double elapsedTime) {
            if (!follower.isBusy()) {
                double currentHeading = follower.getPose().getHeading();
                double headingError = Math.abs(normalizeAngle(currentHeading - targetHeading));
                return headingError < HEADING_TOLERANCE;
            }
            return false;
        }

        @Override
        public String getName() {
            return "Turn to " + Math.toDegrees(targetHeading) + "°";
        }

        private double normalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
    }

    /**
     * Wait/pause action
     */
    private static class WaitAction implements AutonomousStep {
        private final double duration;

        public WaitAction(double duration) {
            this.duration = duration;
        }

        @Override
        public void onStart(Follower follower, double startTime) {
            // Nothing to start
        }

        @Override
        public boolean update(Follower follower, double elapsedTime) {
            return elapsedTime >= duration;
        }

        @Override
        public String getName() {
            return "Wait (" + String.format(Locale.US, "%.1f", duration) + "s)";
        }
    }

    /**
     * Custom action with lambda function
     */
    private static class CustomAction implements AutonomousStep {
        private final String name;
        private final ActionFunction action;

        public CustomAction(String name, ActionFunction action) {
            this.name = name;
            this.action = action;
        }

        @Override
        public void onStart(Follower follower, double startTime) {
            // Lambda handles initialization
        }

        @Override
        public boolean update(Follower follower, double elapsedTime) {
            return action.execute(follower, elapsedTime);
        }

        @Override
        public String getName() {
            return name;
        }
    }

    /**
     * Conditional branch step
     */
    private static class ConditionalStep implements AutonomousStep {
        private final Predicate<Follower> condition;
        private final PedroAutonomousBuilder trueBuilder;
        private final PedroAutonomousBuilder falseBuilder;
        private PedroAutonomousBuilder selectedBuilder;

        public ConditionalStep(Predicate<Follower> condition,
                              PedroAutonomousBuilder trueBuilder,
                              PedroAutonomousBuilder falseBuilder) {
            this.condition = condition;
            this.trueBuilder = trueBuilder;
            this.falseBuilder = falseBuilder;
        }

        @Override
        public void onStart(Follower follower, double startTime) {
            selectedBuilder = condition.test(follower) ? trueBuilder : falseBuilder;
            selectedBuilder.start();
        }

        @Override
        public boolean update(Follower follower, double elapsedTime) {
            selectedBuilder.update();
            return selectedBuilder.isFinished();
        }

        @Override
        public String getName() {
            return "Conditional: " + (selectedBuilder == trueBuilder ? "True" : "False");
        }
    }

    /**
     * Functional interface for custom actions
     */
    @FunctionalInterface
    public interface ActionFunction {
        /**
         * Execute the action
         * @param follower Follower instance for position tracking
         * @param elapsedTime Time elapsed since action started
         * @return true when action is complete
         */
        boolean execute(Follower follower, double elapsedTime);
    }
}


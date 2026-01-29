package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.aurora.AuroraHardwareConfig;
import org.firstinspires.ftc.teamcode.util.aurora.BasicIndexingHelper;
import org.firstinspires.ftc.teamcode.util.aurora.BasicFiringHelper;
import org.firstinspires.ftc.teamcode.util.aurora.Shooter;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

@Autonomous(name = "Pedro Pathing Twelve", group = "Autonomous")
@Configurable // Panels
public class PedroTwelve extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer; // Timer for time-based actions within states
    private ElapsedTime autonomousTimer; // Timer for entire autonomous period

    // Hardware and helpers
    private AuroraHardwareConfig hardware;
    private BasicIndexingHelper indexingHelper;
    private Shooter shooter;
    private BasicFiringHelper firingHelper;

    // Firing state tracking
    private enum FiringSequenceState {
        IDLE,
        FIRING,
        TRANSFERRING_FRONT,
        FIRING_2,
        TRANSFERRING_BACK,
        FIRING_3,
        COMPLETE
    }
    private FiringSequenceState firingSequenceState = FiringSequenceState.IDLE;
    private ElapsedTime firingSequenceTimer;
    private boolean firingSequenceDone = false;
    private int shotsFired = 0; // Track shots fired in current sequence

    // Collection configuration
    private static final long COLLECTION_DURATION_MS = 3000; // Duration to run collection (2 seconds)

    // Timing configuration
    private static final double AUTONOMOUS_TIME_LIMIT = 28.0; // seconds (leave 2s buffer)
    private static final double TIME_FOR_FINAL_PATH = 2.5; // seconds needed for final path

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        autonomousTimer = new ElapsedTime();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware and indexing helper
        hardware = new AuroraHardwareConfig(hardwareMap, telemetry);
        hardware.initialize();

        indexingHelper = new BasicIndexingHelper(hardware, telemetry);

        // Initialize shooter and firing helper
        shooter = new Shooter(hardware, new ShooterConfig(), telemetry);
        shooter.enable();
        firingHelper = new BasicFiringHelper(shooter, indexingHelper, hardware, telemetry);
        firingHelper.setEnabled(true);

        firingSequenceTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 122, Math.toRadians(143)));

        paths = new Paths(follower); // Build paths

        // ===== SPEED CONFIGURATION SECTION =====
        // Customize the speed for each path here (default is 1.0 = normal speed)
        // Example: paths.speedPath2 = 0.5;  // Make Path2 half speed
        // Example: paths.speedPath9 = 0.3;  // Make Path9 very slow

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Called continuously after init() until play is pressed
    }

    @Override
    public void start() {
        // Called once when play is pressed
        autonomousTimer.reset(); // Start timing the autonomous period
        setPathState(0);

        // Start both rollers at half speed for entire autonomous
        indexingHelper.setFrontRollerPower(0.6);
        indexingHelper.setBackRollerPower(0.6);
    }

    @Override
    public void loop() {
        // Update Pedro Pathing - must be called every loop
        follower.update();

        // Update indexing helper - handles timed operations
        indexingHelper.update();

        // Update firing systems
        firingHelper.update();

        // Keep rollers running at constant power, even if helpers stop them
        maintainRollerPower();

        // Update autonomous state machine
        autonomousPathUpdate();

        // Log values to Driver Station for debugging
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Timer", String.format("%.1f", pathTimer.seconds()));
        panelsTelemetry.debug("Auto Time", String.format("%.1f / %.1f", autonomousTimer.seconds(), AUTONOMOUS_TIME_LIMIT));
        panelsTelemetry.debug("Time Remaining", String.format("%.1f", AUTONOMOUS_TIME_LIMIT - autonomousTimer.seconds()));
        panelsTelemetry.debug("Transfer Active", indexingHelper.isTransferActive());
        panelsTelemetry.debug("Shots Fired", shotsFired);
        panelsTelemetry.debug("Firing State", firingSequenceState.toString());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path9;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path10;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path11;
        public PathChain Path7;
        public PathChain Path8;

        // Speed multipliers for each path (1.0 = normal speed, 0.5 = half speed, etc.)
        // Minimum speed is 0.3 otherwise the robot won't move.
        public double speedPath1 = 1.0;
        public double speedPath2 = 1.0;
        public double speedPath9 = 0.3;
        public double speedPath3 = 1.0;
        public double speedPath4 = 1.0;
        public double speedPath10 = 0.3;
        public double speedPath5 = 1.0;
        public double speedPath6 = 1.0;
        public double speedPath11 = 0.3;
        public double speedPath7 = 1.0;
        public double speedPath8 = 1.0;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.154, 122.498),

                                    new Pose(59.041, 84.217)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(134))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.041, 84.217),
                                    new Pose(69.719, 70.726),
                                    new Pose(40.434, 80.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.434, 80.557),

                                    new Pose(26.339, 80.548)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.339, 80.548),

                                    new Pose(58.724, 84.231)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.724, 84.231),
                                    new Pose(77.288, 54.050),
                                    new Pose(40.833, 59.729)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.833, 59.729),

                                    new Pose(24.882, 59.955)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.882, 59.955),

                                    new Pose(58.887, 84.290)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.887, 84.290),
                                    new Pose(71.211, 33.342),
                                    new Pose(42.276, 35.923)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.276, 35.923),

                                    new Pose(24.063, 36.167)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.063, 36.167),

                                    new Pose(59.063, 84.629)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.063, 84.629),

                                    new Pose(31.095, 84.145)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }

    /**
     * Autonomous state machine that manages path following and robot actions.
     *
     * This method is called continuously in the loop() and uses a switch statement
     * to manage different states. You can add actions (e.g., intake, shooter, etc.)
     * at any state by adding code before calling setPathState() to advance.
     *
     * Path Sequence: Path1 → Path2 → Path9 → Path3 → Path4 → Path10 → Path5 → Path6 → Path11 → Path7 → Path8
     *
     * SPEED CONTROL:
     * Speeds can be configured in init() via: paths.speedPathX = value
     * Then in each state, use: followPathWithSpeed(paths.PathX, paths.speedPathX);
     * Speed values: 1.0 = normal, 0.5 = half speed, 0.3 = very slow, 1.5 = faster, etc.
     *
     * Extension Points for Future Features:
     * - After follower.followPath(): Add intake/shooter actions
     * - Inside state cases before setPathState(): Check follower state, time, or position
     * - Example: if (!follower.isBusy()) { performAction(); setPathState(nextState); }
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            // ========== STATE 0: Follow Path 1 ==========
            case 0:
                /* Start following Path 1 */
                followPathWithSpeed(paths.Path1, paths.speedPath1);
                setPathState(1);
                break;

            // ========== STATE 1: Wait for Path 1 Completion ==========
            case 1:
                /* This state waits for the robot to finish Path 1 */
                if (!follower.isBusy()) {
                    /* Path 1 Complete - START FIRING SEQUENCE */
                    follower.breakFollowing(); // Stop following to allow firing
                    startFiringSequence();
                    setPathState(2);
                }
                break;

            // ========== STATE 2: Handle Firing Sequence After Path 1 ==========
            case 2:
                /* Update firing sequence - handles all firing state transitions */
                updateFiringSequence();

                /* When firing sequence completes, move to next path */
                if (firingSequenceDone) {
                    firingSequenceDone = false;
                    firingSequenceState = FiringSequenceState.IDLE;
                    /* Firing Complete - Move to Path 2 */
                    followPathWithSpeed(paths.Path2, paths.speedPath2);
                    setPathState(3);
                }
                break;

            // ========== STATE 3: Wait for Path 2 Completion ==========
            case 3:
                /* Waiting for Path 2 to complete */
                if (!follower.isBusy()) {
                    /* Path 2 Complete - Move to Path 9 and START COLLECTING */
                    followPathWithSpeed(paths.Path9, paths.speedPath9);
                    startCollecting(); // Start artifact collection during Path 9
                    setPathState(4);
                }
                break;

            // ========== STATE 4: Wait for Path 9 Completion (COLLECTING) ==========
            case 4:
                /* Waiting for Path 9 to complete while collecting */
                // Give follower time to register the path before checking if busy
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Path 9 Complete - Move to Path 3 (collection continues) */
                    followPathWithSpeed(paths.Path3, paths.speedPath3);
                    setPathState(5);
                }
                break;

            // ========== STATE 5: Wait for Path 3 Completion (STILL COLLECTING) ==========
            case 5:
                /* Waiting for Path 3 to complete while still collecting */
                // Give follower time to register the path before checking if busy
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Collection complete - Path 3 Complete - START FIRING SEQUENCE */
                    stopCollecting(); // Stop collection after Path 3 complete
                    follower.breakFollowing(); // Stop following to allow firing
                    startFiringSequence();
                    setPathState(6);
                }
                break;

            // ========== STATE 6: Handle Firing Sequence After Path 3 ==========
            case 6:
                /* Update firing sequence - handles all firing state transitions */
                updateFiringSequence();

                /* When firing sequence completes, check if we have time for more or should skip to end */
                if (firingSequenceDone) {
                    firingSequenceDone = false;
                    firingSequenceState = FiringSequenceState.IDLE;

                    /* Check if we're running out of time */
                    if (isRunningOutOfTime()) {
                        /* Not enough time for more collection/firing - skip to final path */
                        stopCollecting(); // Ensure collection is stopped
                        followPathWithSpeed(paths.Path8, paths.speedPath8);
                        setPathState(15);
                    } else {
                        /* Still have time - Move to Path 4 */
                        followPathWithSpeed(paths.Path4, paths.speedPath4);
                        setPathState(7);
                    }
                }
                break;

            // ========== STATE 7: Wait for Path 4 Completion ==========
            case 7:
                /* Waiting for Path 4 to complete */
                if (!follower.isBusy()) {
                    /* Path 4 Complete - Move to Path 10 and START COLLECTING */
                    followPathWithSpeed(paths.Path10, paths.speedPath10);
                    startCollecting(); // Start artifact collection during Path 10
                    setPathState(8);
                }
                break;

            // ========== STATE 8: Wait for Path 10 Completion (COLLECTING) ==========
            case 8:
                /* Waiting for Path 10 to complete while collecting */
                // Give follower time to register the path before checking if busy
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Path 10 Complete - Move to Path 5 (collection continues) */
                    followPathWithSpeed(paths.Path5, paths.speedPath5);
                    setPathState(9);
                }
                break;

            // ========== STATE 9: Wait for Path 5 Completion (STILL COLLECTING) ==========
            case 9:
                /* Waiting for Path 5 to complete while still collecting */
                // Give follower time to register the path before checking if busy
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Collection complete - Path 5 Complete - START FIRING SEQUENCE */
                    stopCollecting(); // Stop collection after Path 5 complete
                    follower.breakFollowing(); // Stop following to allow firing
                    startFiringSequence();
                    setPathState(10);
                }
                break;

            // ========== STATE 10: Handle Firing Sequence After Path 5 ==========
            case 10:
                /* Update firing sequence - handles all firing state transitions */
                updateFiringSequence();

                /* When firing sequence completes, move to next path */
                if (firingSequenceDone) {
                    firingSequenceDone = false;
                    firingSequenceState = FiringSequenceState.IDLE;
                    /* Firing Complete - Move to Path 6 */
                    followPathWithSpeed(paths.Path6, paths.speedPath6);
                    setPathState(11);
                }
                break;

            // ========== STATE 11: Wait for Path 6 Completion ==========
            case 11:
                /* Waiting for Path 6 to complete */
                if (!follower.isBusy()) {
                    /* Path 6 Complete - Move to Path 11 and START COLLECTING */
                    followPathWithSpeed(paths.Path11, paths.speedPath11);
                    startCollecting(); // Start artifact collection during Path 11
                    setPathState(12);
                }
                break;

            // ========== STATE 12: Wait for Path 11 Completion (COLLECTING) ==========
            case 12:
                /* Waiting for Path 11 to complete while collecting */
                // Give follower time to register the path before checking if busy
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Path 11 Complete - Move to Path 7 (collection continues) */
                    followPathWithSpeed(paths.Path7, paths.speedPath7);
                    setPathState(13);
                }
                break;

            // ========== STATE 13: Wait for Path 7 Completion (STILL COLLECTING) ==========
            case 13:
                /* Waiting for Path 7 to complete while still collecting */
                // Give follower time to register the path before checking if busy
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Collection complete - Path 7 Complete - START FIRING SEQUENCE */
                    stopCollecting(); // Stop collection after Path 7 complete
                    follower.breakFollowing(); // Stop following to allow firing
                    startFiringSequence();
                    setPathState(14);
                }
                break;

            // ========== STATE 14: Handle Firing Sequence After Path 7 ==========
            case 14:
                /* Update firing sequence - handles all firing state transitions */
                updateFiringSequence();

                /* When firing sequence completes, move to final path */
                if (firingSequenceDone) {
                    firingSequenceDone = false;
                    firingSequenceState = FiringSequenceState.IDLE;
                    /* Firing Complete - Move to Path 8 (final path) */
                    followPathWithSpeed(paths.Path8, paths.speedPath8);
                    setPathState(15);
                }
                break;

            // ========== STATE 15: Wait for Path 8 Completion (Final Path) ==========
            case 15:
                /* Waiting for Path 8 to complete */
                if (!follower.isBusy()) {

                    /* All paths complete - stop the robot */
                    setPathState(-1);
                }
                break;

            // ========== STATE -1: Autonomous Complete ==========
            case -1:
                /* Robot is idle - all paths have been followed */
                /* Follower will maintain the last pose */
                break;
        }
    }

    /**
     * Sets the autonomous state and resets the path timer.
     * Call this method to transition between states in the state machine.
     *
     * @param pState The new path state
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    /**
     * Follows a path with a custom speed multiplier.
     *
     * Speed multiplier values:
     * - 1.0 = normal speed (default)
     * - 0.5 = half speed (slower)
     * - 0.25 = quarter speed (very slow)
     * - 1.5 = 1.5x speed (faster)
     *
     * @param path The PathChain to follow
     * @param speedMultiplier The speed multiplier (0.0 to 2.0 recommended)
     */
    public void followPathWithSpeed(PathChain path, double speedMultiplier) {
        follower.setMaxPower(speedMultiplier);
        follower.followPath(path);
    }

    /**
     * Check if we're running out of time and need to skip to final path.
     * Leaves buffer time to complete the final path (Path 8).
     *
     * @return true if we need to abort and go to final path
     */
    public boolean isRunningOutOfTime() {
        double timeRemaining = AUTONOMOUS_TIME_LIMIT - autonomousTimer.seconds();
        return timeRemaining < TIME_FOR_FINAL_PATH;
    }

    /**
     * Start collecting artifacts using the front intake.
     * This is a reusable method that can be called from any state to begin collection.
     *
     * Collection sequence:
     * - Front transfer servo moves artifact to center (1.0 power for specified duration)
     * - Back transfer servo runs in reverse to assist (-1.0 power for specified duration)
     *
     * Note: Both rollers are already running at 0.5 power throughout autonomous.
     *
     * This method is non-blocking - it starts timed operations that run in the background.
     * The indexingHelper.update() call in loop() handles the timing automatically.
     *
     * @param durationMs Duration to run the collection sequence (milliseconds)
     */
    public void startCollecting(long durationMs) {
        // Transfer front intake to center (timed operation)
        indexingHelper.transferFrontIntakeToCenterTimed(durationMs);

        // Run back transfer servo in reverse to assist
        indexingHelper.setBackTransferTimed(1.0, durationMs);

        // Note: Rollers are already running at 0.5 power (set in start() method)
    }

    /**
     * Start collecting artifacts with the default duration.
     * Uses COLLECTION_DURATION_MS (2000ms by default).
     */
    public void startCollecting() {
        startCollecting(COLLECTION_DURATION_MS);
    }

    /**
     * Stop collection servos (transfer and injector servos).
     * Note: Rollers continue running at 0.5 power throughout autonomous.
     */
    public void stopCollecting() {
        // Stop transfer servos
        indexingHelper.stopFrontTransfer();
        indexingHelper.stopBackTransfer();

        // Stop injector and uptake servos
        indexingHelper.stopInjector();
        indexingHelper.stopUptake();

        // Stop bottom intake servos
        indexingHelper.stopFrontBottomIntake();
        indexingHelper.stopBackBottomIntake();

        // Keep rollers running at 0.5 power
        maintainRollerPower();
    }

    /**
     * Start a 3-shot firing sequence with automatic transfers.
     *
     * Sequence:
     * 1. Spin up shooter to high basket RPM
     * 2. Wait for shooter ready
     * 3. Fire shot 1 (center artifact)
     * 4. Transfer front to center (timed)
     * 5. Fire shot 2
     * 6. Transfer back to center (timed)
     * 7. Fire shot 3
     * 8. Stop shooter
     */
    public void startFiringSequence() {
        firingSequenceState = FiringSequenceState.FIRING;
        firingSequenceTimer.reset();
        firingSequenceDone = false;
        shotsFired = 0; // Reset shot counter

        // Start shooter spinup to high basket (LONG_RANGE preset = 2800 RPM)
        boolean started = firingHelper.startFiring(
                ShooterConfig.ShooterPreset.MID_RANGE.getTargetRPM(), // Mid-range is what we want
                ShooterConfig.ShooterPreset.MID_RANGE.getName(),
                true
        );
        if (!started) {
            firingSequenceState = FiringSequenceState.IDLE;
            firingSequenceDone = true;
        }
    }

    /**
     * Update the firing sequence state machine.
     *
     * This coordinates with BasicFiringHelper which manages the shooter spinup and keep-alive mode.
     * The firingHelper is responsible for:
     * - Spinning up the shooter
     * - Maintaining RPM between shots (keep-alive mode)
     * - Handling the firing operation
     *
     * This method coordinates:
     * - When to fire each shot (let firingHelper know to fire via fireShot())
     * - Artifact transfers between shots
     * - Sequence completion
     */
    public void updateFiringSequence() {
        if (firingSequenceState == FiringSequenceState.IDLE) {
            return; // No firing sequence active
        }

        switch (firingSequenceState) {
            case FIRING:
                // Wait for shooter to spin up and be ready
                if (firingHelper.isReadyForNextShot()) {
                    // Shooter ready - fire the first shot
                    firingHelper.fireShot();
                    shotsFired++;
                    firingSequenceState = FiringSequenceState.TRANSFERRING_FRONT;
                    firingSequenceTimer.reset();
                }
                break;

            case TRANSFERRING_FRONT:
                // Wait for uptake to finish feeding (first shot complete)
                if (!indexingHelper.isUptakeBusy()) {
                    // Wait a brief moment for uptake to fully clear before starting transfer
                    if (firingSequenceTimer.milliseconds() > 300) {
                        // Check if running out of time before starting next shot
                        if (isRunningOutOfTime()) {
                            // Abort sequence - go straight to completion
                            firingSequenceState = FiringSequenceState.COMPLETE;
                            firingSequenceTimer.reset();
                        } else {
                            // Start transfer from front intake to center
                            indexingHelper.transferFrontIntakeToCenterTimed(2500); // 2.5 seconds
                            firingSequenceState = FiringSequenceState.FIRING_2;
                            firingSequenceTimer.reset();
                        }
                    }
                }
                break;

            case FIRING_2:
                // Wait for transfer to complete, then fire next shot
                if (!indexingHelper.isTransferActive()) {
                    // Transfer complete - fire second shot
                    // firingHelper is in READY_TO_FIRE state (keep-alive mode)
                    firingHelper.fireShot();
                    shotsFired++;
                    firingSequenceState = FiringSequenceState.TRANSFERRING_BACK;
                    firingSequenceTimer.reset();
                }
                break;

            case TRANSFERRING_BACK:
                // Wait for uptake to finish feeding (shot 2 complete)
                if (!indexingHelper.isUptakeBusy()) {
                    // Wait a brief moment before starting transfer
                    if (firingSequenceTimer.milliseconds() > 300) {
                        // Check if running out of time before starting last shot
                        if (isRunningOutOfTime()) {
                            // Abort sequence - go straight to completion
                            firingSequenceState = FiringSequenceState.COMPLETE;
                            firingSequenceTimer.reset();
                        } else {
                            // Start transfer from back intake to center
                            indexingHelper.transferBackIntakeToCenterTimed(2500); // 2.5 seconds
                            firingSequenceState = FiringSequenceState.FIRING_3;
                            firingSequenceTimer.reset();
                        }
                    }
                }
                break;

            case FIRING_3:
                // Wait for transfer to complete, then fire last shot
                if (!indexingHelper.isTransferActive()) {
                    // Transfer complete - fire third shot
                    firingHelper.fireShot();
                    shotsFired++;
                    firingSequenceState = FiringSequenceState.COMPLETE;
                    firingSequenceTimer.reset();
                } else if (firingSequenceTimer.milliseconds() > 3000) {
                    // Timeout on transfer, force completion
                    firingHelper.fireShot();
                    shotsFired++;
                    firingSequenceState = FiringSequenceState.COMPLETE;
                    firingSequenceTimer.reset();
                }
                break;

            case COMPLETE:
                // Wait for last shot to complete (uptake feed finished) before stopping shooter
                if (!indexingHelper.isUptakeBusy()) {
                    // Uptake is done feeding - wait additional time to ensure shot fully completes
                    if (firingSequenceTimer.milliseconds() > 600) {
                        // 600ms safety buffer to ensure shot is fully fired
                        firingHelper.cancelFiring();
                        firingHelper.stopShooter();
                        firingSequenceDone = true;
                    }
                } else {
                    // Uptake still busy - reset timer
                    firingSequenceTimer.reset();
                }
                break;

            default:
                firingSequenceState = FiringSequenceState.IDLE;
                break;
        }
    }

    /**
     * Keep both rollers running at the requested power.
     * This defensively re-applies power if helpers stop them during actions.
     */
    private void maintainRollerPower() {
        indexingHelper.setFrontRollerPower(0.6);
        indexingHelper.setBackRollerPower(0.6);
    }
}

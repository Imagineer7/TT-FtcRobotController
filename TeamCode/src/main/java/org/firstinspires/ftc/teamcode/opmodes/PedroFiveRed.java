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

@Autonomous(name = "Pedro Five RED", group = "Autonomous")
@Configurable // Panels
public class PedroFiveRed extends OpMode {
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
    private static final long COLLECTION_DURATION_MS = 2500; // Duration to run collection

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
        follower.setStartingPose(new Pose(121.629, 123.149, Math.toRadians(38)));

        paths = new Paths(follower); // Build paths

        // ===== SPEED CONFIGURATION SECTION =====
        paths.speedPath3 = 0.3; // Path 3 at low speed for collection

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

        // Start both rollers at 0.9 power for entire autonomous
        indexingHelper.setFrontRollerPower(0.9);
        indexingHelper.setBackRollerPower(0.9);
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
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        // Speed multipliers for each path
        public double speedPath1 = 1.0;
        public double speedPath2 = 1.0;
        public double speedPath3 = 0.3;
        public double speedPath4 = 1.0;
        public double speedPath5 = 1.0;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121.629, 123.149),

                                    new Pose(86.045, 85.231)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(45))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.045, 85.231),
                                    new Pose(84.072, 74.541),
                                    new Pose(100.824, 81.796)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.824, 81.796),

                                    new Pose(119.281, 81.462)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.281, 81.462),

                                    new Pose(85.756, 85.267)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.756, 85.267),

                                    new Pose(115.181, 85.122)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }

    /**
     * Autonomous state machine for Red side 5-path routine.
     *
     * Sequence:
     * Path1 → Fire (3 shots)
     * Path2 → (start collection at end)
     * Path3 → (continue collection, speed reduced)
     * Path4 → (end collection at start), → Fire (3 shots)
     * Path5 → (final path, exit)
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
                    follower.breakFollowing();
                    startFiringSequence();
                    setPathState(2);
                }
                break;

            // ========== STATE 2: Handle Firing Sequence After Path 1 ==========
            case 2:
                /* Update firing sequence */
                updateFiringSequence();

                /* When firing sequence completes, check if we have time for collection or skip to end */
                if (firingSequenceDone) {
                    firingSequenceDone = false;
                    firingSequenceState = FiringSequenceState.IDLE;

                    /* Check if we're running out of time */
                    if (isRunningOutOfTime()) {
                        /* Not enough time for collection/second firing - skip to final path */
                        stopCollecting(); // Ensure collection is stopped
                        followPathWithSpeed(paths.Path5, paths.speedPath5);
                        setPathState(7);
                    } else {
                        /* Still have time - Move to Path 2 */
                        followPathWithSpeed(paths.Path2, paths.speedPath2);
                        setPathState(3);
                    }
                }
                break;

            // ========== STATE 3: Wait for Path 2 Completion (START COLLECTING) ==========
            case 3:
                /* Waiting for Path 2 to complete */
                if (!follower.isBusy()) {
                    /* Path 2 Complete - Move to Path 3 and START COLLECTING */
                    followPathWithSpeed(paths.Path3, paths.speedPath3);
                    startCollecting();
                    setPathState(4);
                }
                break;

            // ========== STATE 4: Wait for Path 3 Completion (COLLECTING) ==========
            case 4:
                /* Waiting for Path 3 to complete while collecting */
                if (pathTimer.milliseconds() < 50) {
                    break; // Wait at least 50ms after starting path
                }

                if (!follower.isBusy()) {
                    /* Path 3 Complete - Move to Path 4 (collection continues) */
                    followPathWithSpeed(paths.Path4, paths.speedPath4);
                    setPathState(5);
                }
                break;

            // ========== STATE 5: Wait for Path 4 Completion (END COLLECTING, THEN FIRE) ==========
            case 5:
                /* Waiting for Path 4 to complete while collecting */
                if (pathTimer.milliseconds() < 50) {
                    break;
                }

                if (!follower.isBusy()) {
                    /* Path 4 Complete - STOP COLLECTING and START FIRING SEQUENCE */
                    stopCollecting();
                    follower.breakFollowing();
                    startFiringSequence();
                    setPathState(6);
                }
                break;

            // ========== STATE 6: Handle Firing Sequence After Path 4 ==========
            case 6:
                /* Update firing sequence */
                updateFiringSequence();

                /* When firing sequence completes, move to Path 5 */
                if (firingSequenceDone) {
                    firingSequenceDone = false;
                    firingSequenceState = FiringSequenceState.IDLE;
                    /* Firing Complete - Move to Path 5 (final path) */
                    followPathWithSpeed(paths.Path5, paths.speedPath5);
                    setPathState(7);
                }
                break;

            // ========== STATE 7: Wait for Path 5 Completion (Final Path) ==========
            case 7:
                /* Waiting for Path 5 to complete */
                if (!follower.isBusy()) {
                    /* All paths complete - stop the robot */
                    setPathState(-1);
                }
                break;

            // ========== STATE -1: Autonomous Complete ==========
            case -1:
                /* Robot is idle - all paths have been followed */
                break;
        }
    }

    /**
     * Sets the autonomous state and resets the path timer.
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
     * @param path The PathChain to follow
     * @param speedMultiplier The speed multiplier (0.0 to 2.0 recommended)
     */
    public void followPathWithSpeed(PathChain path, double speedMultiplier) {
        follower.setMaxPower(speedMultiplier);
        follower.followPath(path);
    }

    /**
     * Check if we're running out of time and need to skip to final path.
     *
     * @return true if we need to abort and go to final path
     */
    public boolean isRunningOutOfTime() {
        double timeRemaining = AUTONOMOUS_TIME_LIMIT - autonomousTimer.seconds();
        return timeRemaining < TIME_FOR_FINAL_PATH;
    }

    /**
     * Start collecting artifacts using the transfer servos.
     * Rollers already running at 0.9 power.
     *
     * @param durationMs Duration to run the collection sequence (milliseconds)
     */
    public void startCollecting(long durationMs) {
        // Transfer front intake to center (timed operation)
        indexingHelper.transferFrontIntakeToCenterTimed(durationMs);

        // Run back transfer servo to assist
        indexingHelper.setBackTransferTimed(1.0, durationMs);
    }

    /**
     * Start collecting artifacts with the default duration.
     */
    public void startCollecting() {
        startCollecting(COLLECTION_DURATION_MS);
    }

    /**
     * Stop collection servos.
     * Note: Rollers continue running at 0.9 power throughout autonomous.
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

        // Keep rollers running at 0.9 power
        maintainRollerPower();
    }

    /**
     * Start a 3-shot firing sequence with automatic transfers.
     *
     * Sequence:
     * 1. Spin up shooter to mid-range RPM
     * 2. Wait for shooter ready
     * 3. Fire shot 1 (center artifact)
     * 4. Transfer front to center
     * 5. Fire shot 2
     * 6. Transfer back to center
     * 7. Fire shot 3
     * 8. Stop shooter
     */
    public void startFiringSequence() {
        firingSequenceState = FiringSequenceState.FIRING;
        firingSequenceTimer.reset();
        firingSequenceDone = false;
        shotsFired = 0; // Reset shot counter

        // Start shooter spinup
        boolean started = firingHelper.startFiring(
                ShooterConfig.ShooterPreset.MID_RANGE.getTargetRPM(),
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
     */
    public void updateFiringSequence() {
        if (firingSequenceState == FiringSequenceState.IDLE) {
            return; // No firing sequence active
        }

        switch (firingSequenceState) {
            case FIRING:
                // Wait for shooter to spin up and be ready
                if (firingSequenceTimer.milliseconds() < 500) {
                    break; // Non-blocking delay before first shot
                }
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
                        // Start transfer from front intake to center
                        indexingHelper.transferFrontIntakeToCenterTimed(2500);
                        firingSequenceState = FiringSequenceState.FIRING_2;
                        firingSequenceTimer.reset();
                    }
                }
                break;

            case FIRING_2:
                // Wait for transfer to complete, then fire next shot
                if (!indexingHelper.isTransferActive()) {
                    // Transfer complete - fire second shot
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
                        // Start transfer from back intake to center
                        indexingHelper.transferBackIntakeToCenterTimed(2500);
                        firingSequenceState = FiringSequenceState.FIRING_3;
                        firingSequenceTimer.reset();
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
        indexingHelper.setFrontRollerPower(0.9);
        indexingHelper.setBackRollerPower(0.9);
    }
}


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

@Autonomous(name = "Pedro Pathing Twelve", group = "Autonomous")
@Configurable // Panels
public class PedroTwelve extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer; // Timer for time-based actions within states

    @Override
    public void init() {
        pathTimer = new ElapsedTime();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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
        setPathState(0);
    }

    @Override
    public void loop() {
        // Update Pedro Pathing - must be called every loop
        follower.update();

        // Update autonomous state machine
        autonomousPathUpdate();

        // Log values to Driver Station for debugging
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
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
        public double speedPath1 = 1.0;
        public double speedPath2 = 1.0;
        public double speedPath9 = 0.4;
        public double speedPath3 = 1.0;
        public double speedPath4 = 1.0;
        public double speedPath10 = 0.4;
        public double speedPath5 = 1.0;
        public double speedPath6 = 1.0;
        public double speedPath11 = 0.4;
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
                                    new Pose(60.145, 77.957),
                                    new Pose(42.824, 84.249)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.824, 84.249),

                                    new Pose(23.624, 84.131)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.624, 84.131),

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
                    /* ===== EXTENSION POINT: Add Actions Here =====
                     * Example: Grab sample, score, etc.
                     * - follower.isBusy(): True if still following path
                     * - pathTimer.getElapsedTimeSeconds(): Elapsed time since state entered
                     * - follower.getPose().getX() / getY(): Current robot position
                     */

                    /* Path 1 Complete - Move to Path 2 */
                    followPathWithSpeed(paths.Path2, paths.speedPath2);
                    setPathState(2);
                }
                break;

            // ========== STATE 2: Wait for Path 2 Completion ==========
            case 2:
                /* Waiting for Path 2 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 2 Complete - Move to Path 9 */
                    followPathWithSpeed(paths.Path9, paths.speedPath9);
                    setPathState(3);
                }
                break;

            // ========== STATE 3: Wait for Path 9 Completion ==========
            case 3:
                /* Waiting for Path 9 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 9 Complete - Move to Path 3 */
                    followPathWithSpeed(paths.Path3, paths.speedPath3);
                    setPathState(4);
                }
                break;

            // ========== STATE 4: Wait for Path 3 Completion ==========
            case 4:
                /* Waiting for Path 3 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 3 Complete - Move to Path 4 */
                    followPathWithSpeed(paths.Path4, paths.speedPath4);
                    setPathState(5);
                }
                break;

            // ========== STATE 5: Wait for Path 4 Completion ==========
            case 5:
                /* Waiting for Path 4 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 4 Complete - Move to Path 10 */
                    followPathWithSpeed(paths.Path10, paths.speedPath10);
                    setPathState(6);
                }
                break;

            // ========== STATE 6: Wait for Path 10 Completion ==========
            case 6:
                /* Waiting for Path 10 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 10 Complete - Move to Path 5 */
                    followPathWithSpeed(paths.Path5, paths.speedPath5);
                    setPathState(7);
                }
                break;

            // ========== STATE 7: Wait for Path 5 Completion ==========
            case 7:
                /* Waiting for Path 5 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 5 Complete - Move to Path 6 */
                    followPathWithSpeed(paths.Path6, paths.speedPath6);
                    setPathState(8);
                }
                break;

            // ========== STATE 8: Wait for Path 6 Completion ==========
            case 8:
                /* Waiting for Path 6 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 6 Complete - Move to Path 11 */
                    followPathWithSpeed(paths.Path11, paths.speedPath11);
                    setPathState(9);
                }
                break;

            // ========== STATE 9: Wait for Path 11 Completion ==========
            case 9:
                /* Waiting for Path 11 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 11 Complete - Move to Path 7 */
                    followPathWithSpeed(paths.Path7, paths.speedPath7);
                    setPathState(10);
                }
                break;

            // ========== STATE 10: Wait for Path 7 Completion ==========
            case 10:
                /* Waiting for Path 7 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Actions Here ===== */

                    /* Path 7 Complete - Move to Path 8 */
                    followPathWithSpeed(paths.Path8, paths.speedPath8);
                    setPathState(11);
                }
                break;

            // ========== STATE 11: Wait for Path 8 Completion (Final Path) ==========
            case 11:
                /* Waiting for Path 8 to complete */
                if (!follower.isBusy()) {
                    /* ===== EXTENSION POINT: Add Final Actions Here =====
                     * Example: Park, final scoring, etc.
                     */

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
}
    
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Panels field visualization
    private FieldManager panelsField;
    private static final Style robotStyle = new Style("", "#00FF00", 0.75); // Green robot
    private static final Style targetStyle = new Style("", "#FF0000", 0.75); // Red target
    private static final Style pathStyle = new Style("", "#3F51B5", 0.75); // Blue path
    private static final double ROBOT_RADIUS = 9.0; // Robot radius in inches

    // State machine constants
    private static final int STATE_IDLE = 0;
    private static final int STATE_FOLLOWING_PATH_1 = 1;
    private static final int STATE_FOLLOWING_PATH_2 = 2;
    private static final int STATE_FOLLOWING_PATH_3 = 3;
    private static final int STATE_FOLLOWING_PATH_4 = 4;
    private static final int STATE_FOLLOWING_PATH_5 = 5;
    private static final int STATE_FOLLOWING_PATH_6 = 6;
    private static final int STATE_FOLLOWING_PATH_7 = 7;
    private static final int STATE_FOLLOWING_PATH_8 = 8;
    private static final int STATE_FINISHED = 9;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Panels field visualization
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.862, 9.077, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths

        pathState = STATE_IDLE; // Start in idle state

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Called when autonomous starts (when PLAY is pressed)
        pathState = STATE_FOLLOWING_PATH_1;
        follower.followPath(paths.Path1);

        panelsTelemetry.debug("Status", "Started - Following Path 1");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Get current robot pose
        Pose currentPose = follower.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading = currentPose.getHeading();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", getStateName(pathState));
        panelsTelemetry.debug("Path Busy", follower.isBusy());
        panelsTelemetry.debug("X", String.format(java.util.Locale.US, "%.2f", x));
        panelsTelemetry.debug("Y", String.format(java.util.Locale.US, "%.2f", y));
        panelsTelemetry.debug("Heading", String.format(java.util.Locale.US, "%.1f°", Math.toDegrees(heading)));

        // Panels Field Visualization
        drawRobot(currentPose);

        if (follower.isBusy()) {
            // Draw target position
            Pose targetPose = getTargetPose();
            if (targetPose != null) {
                drawTarget(targetPose);
            }

            // Draw current path if available
            if (follower.getCurrentPath() != null) {
                drawPath(follower.getCurrentPath());
            }
        }

        panelsField.update();
        panelsTelemetry.update(telemetry);
    }

    /**
     * Get the target pose of the current path
     */
    private Pose getTargetPose() {
        switch (pathState) {
            case STATE_FOLLOWING_PATH_1: return new Pose(18.567, 35.484);
            case STATE_FOLLOWING_PATH_2: return new Pose(47.656, 12.791);
            case STATE_FOLLOWING_PATH_3: return new Pose(9.077, 9.077);
            case STATE_FOLLOWING_PATH_4: return new Pose(56.940, 22.074);
            case STATE_FOLLOWING_PATH_5: return new Pose(18.774, 59.415);
            case STATE_FOLLOWING_PATH_6: return new Pose(59.622, 25.582);
            case STATE_FOLLOWING_PATH_7: return new Pose(18.980, 84.172);
            case STATE_FOLLOWING_PATH_8: return new Pose(60.034, 25.582);
            default: return null;
        }
    }

    /**
     * Draw robot on Panels field with heading indicator
     */
    private void drawRobot(Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(robotStyle);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        // Draw heading line
        double headingLength = ROBOT_RADIUS;
        double x2 = pose.getX() + Math.cos(pose.getHeading()) * headingLength;
        double y2 = pose.getY() + Math.sin(pose.getHeading()) * headingLength;

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.line(x2, y2);
    }

    /**
     * Draw target position on Panels field
     */
    private void drawTarget(Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())) {
            return;
        }

        panelsField.setStyle(targetStyle);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS / 2); // Smaller circle for target
    }

    /**
     * Draw the current path on Panels field
     */
    private void drawPath(com.pedropathing.paths.Path path) {
        if (path == null) {
            return;
        }

        double[][] points = path.getPanelsDrawingPoints();

        if (points == null || points.length < 2 || points[0].length == 0) {
            return;
        }

        // Clean up NaN values
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(pathStyle);
        panelsField.moveCursor(points[0][0], points[1][0]);

        // Draw lines connecting each point
        for (int i = 1; i < points[0].length; i++) {
            panelsField.line(points[0][i], points[1][i]);
        }
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.862, 9.077),
                                    new Pose(44.974, 30.120),
                                    new Pose(18.567, 35.484)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.567, 35.484),
                                    new Pose(41.261, 22.487),
                                    new Pose(47.656, 12.791)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(77))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.656, 12.791),
                                    new Pose(34.659, 13.203),
                                    new Pose(16.298, 9.903),
                                    new Pose(9.077, 9.077)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(77), Math.toRadians(92))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.077, 9.077),
                                    new Pose(55.289, 17.330),
                                    new Pose(56.940, 22.074)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(92), Math.toRadians(75))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.940, 22.074),
                                    new Pose(49.513, 43.117),
                                    new Pose(39.610, 70.143),
                                    new Pose(34.659, 56.321),
                                    new Pose(18.774, 59.415)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.774, 59.415),
                                    new Pose(45.799, 32.802),
                                    new Pose(59.622, 25.582)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(82))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.622, 25.582),
                                    new Pose(48.894, 93.249),
                                    new Pose(33.009, 83.966),
                                    new Pose(18.980, 84.172)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(82), Math.toRadians(90))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.980, 84.172),
                                    new Pose(47.000, 79.000),
                                    new Pose(60.034, 25.582)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(76))
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_IDLE:
                // Waiting for start
                break;

            case STATE_FOLLOWING_PATH_1:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_2;
                    follower.followPath(paths.Path2);
                    panelsTelemetry.debug("Status", "Path 1 Complete - Starting Path 2");
                }
                break;

            case STATE_FOLLOWING_PATH_2:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_3;
                    follower.followPath(paths.Path3);
                    panelsTelemetry.debug("Status", "Path 2 Complete - Starting Path 3");
                }
                break;

            case STATE_FOLLOWING_PATH_3:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_4;
                    follower.followPath(paths.Path4);
                    panelsTelemetry.debug("Status", "Path 3 Complete - Starting Path 4");
                }
                break;

            case STATE_FOLLOWING_PATH_4:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_5;
                    follower.followPath(paths.Path5);
                    panelsTelemetry.debug("Status", "Path 4 Complete - Starting Path 5");
                }
                break;

            case STATE_FOLLOWING_PATH_5:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_6;
                    follower.followPath(paths.Path6);
                    panelsTelemetry.debug("Status", "Path 5 Complete - Starting Path 6");
                }
                break;

            case STATE_FOLLOWING_PATH_6:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_7;
                    follower.followPath(paths.Path7);
                    panelsTelemetry.debug("Status", "Path 6 Complete - Starting Path 7");
                }
                break;

            case STATE_FOLLOWING_PATH_7:
                if (!follower.isBusy()) {
                    pathState = STATE_FOLLOWING_PATH_8;
                    follower.followPath(paths.Path8);
                    panelsTelemetry.debug("Status", "Path 7 Complete - Starting Path 8");
                }
                break;

            case STATE_FOLLOWING_PATH_8:
                if (!follower.isBusy()) {
                    pathState = STATE_FINISHED;
                    panelsTelemetry.debug("Status", "Autonomous Complete");
                }
                break;

            case STATE_FINISHED:
                // Autonomous complete - do nothing
                break;
        }

        return pathState;
    }

    /**
     * Helper method to get human-readable state names
     */
    private String getStateName(int state) {
        switch (state) {
            case STATE_IDLE: return "IDLE";
            case STATE_FOLLOWING_PATH_1: return "FOLLOWING_PATH_1";
            case STATE_FOLLOWING_PATH_2: return "FOLLOWING_PATH_2";
            case STATE_FOLLOWING_PATH_3: return "FOLLOWING_PATH_3";
            case STATE_FOLLOWING_PATH_4: return "FOLLOWING_PATH_4";
            case STATE_FOLLOWING_PATH_5: return "FOLLOWING_PATH_5";
            case STATE_FOLLOWING_PATH_6: return "FOLLOWING_PATH_6";
            case STATE_FOLLOWING_PATH_7: return "FOLLOWING_PATH_7";
            case STATE_FOLLOWING_PATH_8: return "FOLLOWING_PATH_8";
            case STATE_FINISHED: return "FINISHED";
            default: return "UNKNOWN";
        }
    }
}


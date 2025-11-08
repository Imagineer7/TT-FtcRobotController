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
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAutonomousBuilder;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

@Autonomous(name = "Pedro Builder Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroBuilderAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private EnhancedDecodeHelper shooter;
    private PedroAutonomousBuilder autoBuilder;
    private Paths paths;

    // Panels field visualization
    private FieldManager panelsField;
    private static final Style robotStyle = new Style("", "#00FF00", 0.75);
    private static final Style targetStyle = new Style("", "#FF0000", 0.75);
    private static final Style pathStyle = new Style("", "#3F51B5", 0.75);
    private static final double ROBOT_RADIUS = 9.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Panels field visualization
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.862, 9.077, Math.toRadians(0)));

        // Initialize shooter
        shooter = new EnhancedDecodeHelper(hardwareMap);

        // Build paths
        paths = new Paths(follower);

        // Build autonomous sequence using the builder
        buildAutonomousSequence();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Total Steps", autoBuilder.getTotalSteps());
        panelsTelemetry.update(telemetry);
    }

    /**
     * Build the autonomous sequence here!
     * This is where you define your paths and actions in order.
     */
    private void buildAutonomousSequence() {
        autoBuilder = new PedroAutonomousBuilder(follower)
                .withShooter(shooter)

                // Example sequence - customize this for your autonomous!
                .addPath(paths.Path1)
                .addPath(paths.Path2)

                // After Path 2, turn to goal and shoot
                .addTurnToHeading(Math.toRadians(90))  // Turn to face goal
                .addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)  // Fire 3 shots

                .addPath(paths.Path3)
                .addPath(paths.Path4)
                .addPath(paths.Path5)
                .addPath(paths.Path6)
                .addPath(paths.Path7)
                .addPath(paths.Path8);

        // More complex example (commented out):
        /*
        autoBuilder = new PedroAutonomousBuilder(follower)
                .withShooter(shooter)

                // Score preload
                .addPath(paths.Path1)
                .addTurnToHeading(Math.toRadians(45))
                .addCustomAction("Deploy Specimen", (f, t) -> {
                    // Custom mechanism code
                    return t > 0.5; // Complete after 0.5s
                })

                // First sample cycle
                .addPath(paths.Path2)
                .addWait(0.3)  // Wait for sample to settle
                .addCustomAction("Grab Sample", (f, t) -> {
                    // Intake control
                    return t > 0.8;
                })

                .addPath(paths.Path3)  // Go to basket
                .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)

                // Second sample cycle
                .addPath(paths.Path4)
                .addWait(0.3)
                .addCustomAction("Grab Sample", (f, t) -> {
                    return t > 0.8;
                })

                .addPath(paths.Path5)  // Go to basket
                .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)

                // Third sample cycle
                .addPath(paths.Path6)
                .addWait(0.3)
                .addCustomAction("Grab Sample", (f, t) -> {
                    return t > 0.8;
                })

                .addPath(paths.Path7)  // Go to basket
                .addShootAction(1, ShooterConfig.ShooterPreset.SHORT_RANGE)

                // Park
                .addPath(paths.Path8);
        */
    }

    @Override
    public void start() {
        autoBuilder.start();
        panelsTelemetry.debug("Status", "Started");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        // Update follower (required before builder update)
        follower.update();

        // Update autonomous sequence
        String currentStep = autoBuilder.update();

        // Get current robot pose
        Pose currentPose = follower.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading = currentPose.getHeading();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Current Step", currentStep);
        panelsTelemetry.debug("Step Progress",
            (autoBuilder.getCurrentStepIndex() + 1) + " / " + autoBuilder.getTotalSteps());
        panelsTelemetry.debug("X", String.format(java.util.Locale.US, "%.2f", x));
        panelsTelemetry.debug("Y", String.format(java.util.Locale.US, "%.2f", y));
        panelsTelemetry.debug("Heading", String.format(java.util.Locale.US, "%.1f°", Math.toDegrees(heading)));
        panelsTelemetry.debug("Finished", autoBuilder.isFinished());

        // Panels Field Visualization
        drawRobot(currentPose);

        if (follower.isBusy() && follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath());
        }

        panelsField.update();
        panelsTelemetry.update(telemetry);
    }

    /**
     * Define your paths here
     */
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
}


package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroAutonomousBuilder;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

@Autonomous(name = "Blue Long Range", group = "Pedro Autonomous")
@Configurable // Panels
public class BlueLongRange extends OpMode {

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
        follower.setStartingPose(new Pose(56.564, 8.495, Math.toRadians(90)));

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
     * Main method to build the autonomous sequence
     */
    private void buildAutonomousSequence() {
        autoBuilder = new PedroAutonomousBuilder(follower)
                .withShooter(shooter)

                // Example sequence - customize this for your autonomous!
                .addPath(paths.Path1)
                .addTurnToHeading(Math.toRadians(114))  // Convert degrees to radians
                .addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)  // Fire 3 shots
                .addPath(paths.Path2);
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.564, 8.495), new Pose(59.050, 18.855))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.050, 18.855), new Pose(61.329, 33.358))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
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

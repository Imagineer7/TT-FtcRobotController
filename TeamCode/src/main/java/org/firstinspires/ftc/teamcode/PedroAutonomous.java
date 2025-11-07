package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.aurora.EnhancedDecodeHelper;
import org.firstinspires.ftc.teamcode.util.aurora.ShooterConfig;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private EnhancedDecodeHelper shooter; // Shooter control system
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // State machine timing
    private com.qualcomm.robotcore.util.ElapsedTime pathTimer; // Timer for action delays

    // Shooter configuration
    private static final ShooterConfig.ShooterPreset SHOOTER_PRESET = ShooterConfig.ShooterPreset.SHORT_RANGE;
    private static final int SHOTS_TO_FIRE = 3;
    private int shotsFired = 0; // Track shots fired during current action
    private double lastShotTime = 0; // Track time of last shot

    // State machine constants
    private static final int STATE_IDLE = 0;
    private static final int STATE_FOLLOWING_PATH_1 = 1;
    private static final int STATE_ACTION_AT_PATH_1_END = 2;
    private static final int STATE_FOLLOWING_PATH_2 = 3;
    private static final int STATE_ACTION_AT_PATH_2_END = 4;
    private static final int STATE_FINISHED = 5;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Initialize shooter system
        shooter = new EnhancedDecodeHelper(hardwareMap);
        shooter.getConfig().setPreset(SHOOTER_PRESET);

        paths = new Paths(follower); // Build paths

        pathTimer = new com.qualcomm.robotcore.util.ElapsedTime(); // Initialize timer
        pathState = STATE_IDLE; // Start in idle state

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Shooter Preset", SHOOTER_PRESET.getName());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Called when autonomous starts (when PLAY is pressed)
        pathState = STATE_FOLLOWING_PATH_1;
        follower.followPath(paths.Path1);
        pathTimer.reset();

        panelsTelemetry.debug("Status", "Started - Following Path 1");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", getStateName(pathState));
        panelsTelemetry.debug("Path Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        // Shooter status
        panelsTelemetry.debug("Shooter Running", shooter.isShooterRunning());
        panelsTelemetry.debug("Shooter RPM", String.format(java.util.Locale.US, "%.0f", shooter.getCurrentRPM()));
        panelsTelemetry.debug("Target RPM", String.format(java.util.Locale.US, "%.0f", shooter.getTargetRPM()));
        panelsTelemetry.debug("Shooting", shooter.isShooting());

        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.719, 123.073), new Pose(40.403, 107.327))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(138))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.403, 107.327), new Pose(59.258, 59.258))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(270))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_IDLE:
                // Waiting for start
                break;

            case STATE_FOLLOWING_PATH_1:
                // Following Path 1 - wait until path is complete
                if (!follower.isBusy()) {
                    // Path 1 complete - transition to action state
                    pathState = STATE_ACTION_AT_PATH_1_END;
                    pathTimer.reset();
                    panelsTelemetry.debug("Status", "Path 1 Complete - Performing Action");
                }
                break;

            case STATE_ACTION_AT_PATH_1_END:
                // Perform action at end of Path 1 - Fire three shots
                // Reset shot counter when first entering this state
                if (pathTimer.seconds() < 0.05) {
                    shotsFired = 0;
                }

                performActionAtPath1End();

                // Wait for shooting sequence to complete
                // Check if all shots have been fired and shooter is not currently shooting
                if (shotsFired >= SHOTS_TO_FIRE && !shooter.isShooting() && pathTimer.seconds() > 1.5) {
                    // Shooting complete - stop shooter and start Path 2
                    shooter.stopShooter();
                    pathState = STATE_FOLLOWING_PATH_2;
                    follower.followPath(paths.Path2);
                    panelsTelemetry.debug("Status", "Shooting complete - Starting Path 2");
                }
                break;

            case STATE_FOLLOWING_PATH_2:
                // Following Path 2 - wait until path is complete
                if (!follower.isBusy()) {
                    // Path 2 complete - transition to action state
                    pathState = STATE_ACTION_AT_PATH_2_END;
                    pathTimer.reset();
                    panelsTelemetry.debug("Status", "Path 2 Complete - Performing Action");
                }
                break;

            case STATE_ACTION_AT_PATH_2_END:
                // Perform action at end of Path 2
                // Example: Park, final score, etc.
                performActionAtPath2End();

                // Wait for action to complete (0.5 seconds example)
                if (pathTimer.seconds() > 0.5) {
                    // Action complete - finish autonomous
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
     * Action to perform at the end of Path 1
     * Fire three shots using the EnhancedDecodeHelper
     */
    private void performActionAtPath1End() {
        double elapsed = pathTimer.seconds();
        double shotInterval = shooter.getConfig().getPreset().getShotInterval();

        // Start shooter spinup immediately when entering this state
        if (elapsed < 0.1) {
            shooter.startShooter();
            panelsTelemetry.debug("Action 1", "Starting shooter spinup");
            return;
        }

        // Wait for shooter to reach target RPM and stabilize
        if (!shooter.isShooterReady()) {
            panelsTelemetry.debug("Action 1", "Waiting for shooter spinup...");
            panelsTelemetry.debug("RPM Status",
                String.format(java.util.Locale.US, "%.0f / %.0f",
                    shooter.getCurrentRPM(), shooter.getTargetRPM()));
            return;
        }

        // Shooter is ready - fire shots with timing between each
        if (shotsFired < SHOTS_TO_FIRE) {
            // Check if enough time has passed since last shot
            double timeSinceLastShot = elapsed - lastShotTime;

            if (shotsFired == 0 || timeSinceLastShot >= shotInterval) {
                // Try to fire a shot
                if (shooter.fireSingleShot()) {
                    shotsFired++;
                    lastShotTime = elapsed;
                    panelsTelemetry.debug("Action 1", "Shot " + shotsFired + " fired!");
                }
            } else {
                // Waiting for next shot interval
                double waitTime = shotInterval - timeSinceLastShot;
                panelsTelemetry.debug("Action 1",
                    String.format(java.util.Locale.US, "Next shot in %.1fs", waitTime));
            }

            panelsTelemetry.debug("Shot Progress", shotsFired + " / " + SHOTS_TO_FIRE);
        } else {
            // All shots fired - waiting for completion
            panelsTelemetry.debug("Action 1", "All shots fired - completing");
        }
    }

    /**
     * Action to perform at the end of Path 2
     * Add your custom mechanism control here
     */
    private void performActionAtPath2End() {
        // Example: Park position, stow mechanisms, etc.
        // Replace with your actual mechanism code

        panelsTelemetry.debug("Action 2", "Running for " + pathTimer.seconds() + "s");
    }

    /**
     * Helper method to get human-readable state names
     */
    private String getStateName(int state) {
        switch (state) {
            case STATE_IDLE: return "IDLE";
            case STATE_FOLLOWING_PATH_1: return "FOLLOWING_PATH_1";
            case STATE_ACTION_AT_PATH_1_END: return "ACTION_AT_PATH_1_END";
            case STATE_FOLLOWING_PATH_2: return "FOLLOWING_PATH_2";
            case STATE_ACTION_AT_PATH_2_END: return "ACTION_AT_PATH_2_END";
            case STATE_FINISHED: return "FINISHED";
            default: return "UNKNOWN";
        }
    }
}


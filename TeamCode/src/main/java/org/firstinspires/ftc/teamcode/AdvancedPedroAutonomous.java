// Complete Example: Pedro Autonomous with Connector Path Helper
// This example shows how to use the automatic connector path feature

package org.firstinspires.ftc.teamcode;

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

/**
 * Advanced Autonomous using PedroAutonomousBuilder with Connector Paths
 *
 * Features:
 * - Automatic connector path insertion for smooth transitions
 * - State machine management
 * - Action support (shooting, turning, custom)
 * - Panels telemetry integration
 * - Continuous pose tracking during all actions
 *
 * Key Points:
 * - Always provide the starting pose when calling addPath()
 * - Connector paths are automatically inserted when needed
 * - Thresholds are configurable for fine-tuning
 * - Robot pose is continuously tracked even during actions
 */
@Autonomous(name = "Advanced Pedro with Connectors", group = "Pedro Autonomous")
public class AdvancedPedroAutonomous extends OpMode {

    // Telemetry
    private TelemetryManager panelsTelemetry;

    // Pedro Pathing components
    private Follower follower;
    private PedroAutonomousBuilder builder;

    // Other systems
    private EnhancedDecodeHelper shooterHelper;

    // Paths container
    private Paths paths;

    @Override
    public void init() {
        // Initialize telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose (where the robot physically is at the start)
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));

        // Initialize shooter
        shooterHelper = new EnhancedDecodeHelper(hardwareMap);

        // Create path definitions
        paths = new Paths(follower);

        // Build the autonomous routine with automatic connector paths
        builder = new PedroAutonomousBuilder(follower)
            .withShooter(shooterHelper)

            // Optional: Configure connector thresholds
            // (defaults shown here)
            .setReconnectDistanceThreshold(6.0)      // inches
            .setReconnectHeadingThreshold(Math.toRadians(20)) // degrees
            .setAutoConnectEnabled(true)             // enabled by default

            // Sequence your paths with their starting poses
            // Connectors will automatically insert if needed

            // Segment 1: Travel to first position
            .addPath(paths.Path1, new Pose(0, 0, Math.toRadians(90)))

            // Segment 2: Shoot at first position
            .addWait(0.5)
            .addShootAction(3, ShooterConfig.ShooterPreset.LONG_RANGE)

            // Segment 3: Travel to second position
            // If robot pose differs from Path2's start, a connector will auto-insert
            .addPath(paths.Path2, new Pose(40, 100, Math.toRadians(180)))

            // Segment 4: Custom action
            .addCustomAction("AlignIntake", (follower, elapsedTime) -> {
                // This runs while path is paused
                // Keep returning false until action completes
                // Robot pose continues to be tracked
                if (elapsedTime < 0.5) {
                    // Perform intake alignment
                    return false;
                }
                return true; // Action complete, resume path
            })

            // Segment 5: Travel to third position
            .addPath(paths.Path3, new Pose(70, 50, Math.toRadians(270)))

            // Segment 6: Final turn
            .addTurnToHeading(Math.toRadians(0))

            // Segment 7: Shoot from final position
            .addShootAction(5, ShooterConfig.ShooterPreset.SHORT_RANGE);

        // Start the autonomous routine
        builder.start();

        panelsTelemetry.debug("Status", "Autonomous Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        // Update the autonomous state machine
        // This also calls follower.update() internally
        String currentStep = builder.update();

        // Log telemetry
        panelsTelemetry.debug("Current Step", currentStep);
        panelsTelemetry.debug("Step Index", builder.getCurrentStepIndex() + "/" + builder.getTotalSteps());
        panelsTelemetry.debug("Robot X", String.format("%.2f", follower.getPose().getX()));
        panelsTelemetry.debug("Robot Y", String.format("%.2f", follower.getPose().getY()));
        panelsTelemetry.debug("Robot Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.debug("Is Finished", builder.isFinished());

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (builder != null && !builder.isFinished()) {
            panelsTelemetry.debug("Status", "Autonomous stopped manually");
            panelsTelemetry.update(telemetry);
        }
    }

    /**
     * Paths class - Define all your path chains here
     * Each path should start at specific world coordinates
     */
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            // Path 1: Starting position to first target
            Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                    new Pose(0, 0, Math.toRadians(90)),
                    new Pose(20, 50, Math.toRadians(90))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

            // Path 2: First target to second target
            // This path starts at (40, 100) with heading 180°
            // If the robot's actual pose differs significantly, a connector will auto-insert
            Path2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(40, 100, Math.toRadians(180)),
                    new Pose(50, 120, Math.toRadians(180)),
                    new Pose(70, 100, Math.toRadians(270))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

            // Path 3: Second target to final position
            Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                    new Pose(70, 50, Math.toRadians(270)),
                    new Pose(50, 20, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();
        }
    }
}

/**
 * How the Connector Path Feature Works
 *
 * 1. DETECTION:
 *    When addPath(path, startPose) is called, the builder checks:
 *    - Distance between robot's current pose and path's start pose
 *    - Heading difference between robot and path
 *
 * 2. COMPARISON:
 *    if (distance > 6.0 inches OR heading > 20 degrees) {
 *        // Insert connector
 *    }
 *
 * 3. CONNECTOR CREATION:
 *    A smooth Bezier curve is created:
 *    - Start: Robot's current position
 *    - Control Point: Midpoint (for smooth curvature)
 *    - End: Path's starting position
 *
 * 4. EXECUTION:
 *    The connector is added to the step list BEFORE the main path
 *    When executed:
 *    - Robot follows connector path
 *    - Robot arrives at path start position
 *    - Robot then follows main path
 *    - All during this time, pose tracking continues
 *
 * Benefits:
 * ✅ No jerkiness from position mismatches
 * ✅ Works with paths called out of sequence
 * ✅ Automatic - no manual intervention needed
 * ✅ Configurable thresholds for your specific needs
 */

/**
 * Common Configuration Patterns
 *
 * Pattern 1: Conservative (lots of connectors)
 *   .setReconnectDistanceThreshold(3.0)      // Lower = more connectors
 *   .setReconnectHeadingThreshold(Math.toRadians(10))
 *
 * Pattern 2: Aggressive (fewer connectors)
 *   .setReconnectDistanceThreshold(15.0)     // Higher = fewer connectors
 *   .setReconnectHeadingThreshold(Math.toRadians(45))
 *
 * Pattern 3: Disabled (manual only)
 *   .setAutoConnectEnabled(false)
 *
 * Pattern 4: Distance only (ignore heading)
 *   .setReconnectHeadingThreshold(Math.PI)   // Very high = always passes heading check
 */


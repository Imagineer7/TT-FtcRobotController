package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;
import org.firstinspires.ftc.teamcode.util.aurora.lightning.AuroraLightningCore;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanner;
import org.firstinspires.ftc.teamcode.util.tool.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * LightningForward24Auto - Simple test autonomous
 *
 * This OpMode:
 * - Starts at (0, 0, 0)
 * - Drives forward 24 inches
 * - Stops
 *
 * Use this to test basic Lightning Core functionality
 */
@Autonomous(name="Lightning Forward 24", group="Lightning Demo")
public class LightningForward24Auto extends LinearOpMode {

    // Robot systems
    private AuroraManager robotManager;
    private AuroraLightningCore lightning;
    private ElapsedTime runtime = new ElapsedTime();

    // Distance to travel
    private static final double FORWARD_DISTANCE = 24.0;

    @Override
    public void runOpMode() {
        // ========== INITIALIZATION ==========
        telemetry.addLine("Initializing Lightning Forward 24...");
        telemetry.update();

        // Initialize AuroraManager first (provides drive system)
        robotManager = new AuroraManager(hardwareMap, telemetry);

        // Initialize AuroraLightningCore with the robotManager
        lightning = new AuroraLightningCore(
            robotManager,
            FieldMap.Alliance.BLUE  // Change to RED if needed
        );

        // Initialize all subsystems
        if (!lightning.initialize()) {
            telemetry.addLine("⚠️ ERROR: Lightning Core failed to initialize!");
            telemetry.update();
            sleep(3000);
            return;
        }

        // Set starting position to (0, 0, 0)
        lightning.getPositionManager().updatePosition(0, 0, 0);
        lightning.getPositionManager().setStartPositionToCurrent();

        // Configure for movement
        lightning.setPathMode(PathPlanner.PathMode.STRAIGHT);  // Use straight lines for simple paths
        lightning.setSmoothingFactor(0.5);  // Less smoothing for simple path

        // Set looser tolerances to prevent jerky movement at intermediate waypoints
        lightning.setPositionTolerance(4.0);  // Within 4 inches (looser for smooth flow)
        lightning.setHeadingTolerance(5.0);   // Within 10 degrees (looser for smooth flow)

        telemetry.clear();
        telemetry.addLine("✅ Lightning Forward 24 Ready!");
        telemetry.addLine("");
        telemetry.addData("Start Position", "0, 0, 0°");
        telemetry.addData("Target", "24, 0, 0°");
        telemetry.addLine("");
        telemetry.addLine("Press START to drive forward 24 inches");
        telemetry.update();

        // ========== WAIT FOR START ==========
        waitForStart();
        runtime.reset();

        // ========== RUN AUTONOMOUS SEQUENCE ==========
        while (opModeIsActive()) {
            runForwardSequence();

            // Break after sequence completes once
            break;
        }

        // ========== CLEANUP ==========
        lightning.stop();

        telemetry.addLine("✅ Forward 24 autonomous complete!");
        telemetry.update();
    }

    /**
     * Run the forward movement sequence
     */
    private void runForwardSequence() {
        telemetry.addLine("Starting forward movement...");
        telemetry.update();

        // Create simple forward path with just 2 waypoints
        List<Pose> forwardPath = new ArrayList<>();
        forwardPath.add(new Pose(0, 0, 0));                    // Start at origin
        forwardPath.add(new Pose(FORWARD_DISTANCE, 0, 0));
        forwardPath.add(new Pose(24, FORWARD_DISTANCE, 0));// Move sideways 24 inches
        forwardPath.add(new Pose(24, FORWARD_DISTANCE, 90));// Turn 90

        // Load path into Lightning Core - use STRAIGHT mode to avoid intermediate waypoints
        lightning.followPath(forwardPath, PathPlanner.PathMode.STRAIGHT);

        telemetry.addLine("Path loaded, executing...");
        telemetry.update();

        // ========== MAIN LOOP ==========
        // Monitor path execution while OpMode is active
        int loopCount = 0;
        double startTime = runtime.seconds();

        while (opModeIsActive() && !lightning.isPathComplete()) {
            loopCount++;

            // Safety timeout after 10 seconds
            if (runtime.seconds() - startTime > 10.0) {
                telemetry.addLine("⚠️ TIMEOUT: Movement took too long!");
                break;
            }

            // Update all subsystems (MUST call every loop!)
            lightning.update();

            // Display telemetry
            displayTelemetry(loopCount);
            telemetry.update();

            // Small sleep to prevent loop from running too fast
            sleep(10);
        }

        // ========== MOVEMENT COMPLETE ==========
        if (opModeIsActive()) {
            telemetry.addLine("✓ Movement complete!");
            telemetry.update();

            // Stop all movement
            lightning.stop();

            // Hold final position for 2 seconds
            double endTime = runtime.seconds() + 2.0;
            while (opModeIsActive() && runtime.seconds() < endTime) {
                displayFinalStatus();
                telemetry.update();
                sleep(50);
            }
        }
    }

    /**
     * Display telemetry during movement
     */
    private void displayTelemetry(int loopCount) {
        // Get current position
        double[] pos = lightning.getCurrentPosition();

        telemetry.addData("=== LIGHTNING PATH TEST ===", "");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Loop Count", loopCount);
        telemetry.addData("", "");

        telemetry.addData("Current Pos", "X:%.1f Y:%.1f H:%.1f°",
            pos[0], pos[1], pos[2]);

        // Show actual target from PoseController
        if (lightning.isFollowingPath()) {
            int remaining = lightning.getPoseController().getRemainingWaypoints();
            int total = lightning.getPoseController().getTotalWaypoints();
            int current = total - remaining;
            telemetry.addData("Waypoint", "%d / %d", current, total);

            // Show which waypoint we're CURRENTLY HEADING TO (next in queue)
            // current tells us how many we've completed, so we're heading to waypoint index 'current'
            if (remaining > 0) {
                if (current == 0) {
                    telemetry.addData("Target", "Start (0, 0, 0°)");
                } else if (current == 1) {
                    telemetry.addData("Target", "Forward (24, 0, 0°)");
                } else if (current == 2) {
                    telemetry.addData("Target", "Sideways (24, 24, 0°)");
                } else if (current == 3) {
                    telemetry.addData("Target", "Turn (24, 24, 90°)");
                }
            } else {
                telemetry.addData("Target", "Path Complete!");
            }
        }
        telemetry.addData("", "");

        telemetry.addData("At Target", lightning.atTarget() ? "YES ✓" : "NO");
        telemetry.addData("Path Complete", lightning.isPathComplete() ? "YES ✓" : "NO");

        // Add detailed heading debug info - show ACTUAL target from PoseController
        telemetry.addData("", "");
        telemetry.addData("=== DEBUG ===", "");
        telemetry.addData("Current Heading", "%.1f°", pos[2]);

        // Get actual target from PoseController
        Double targetHeading = lightning.getPoseController().getTargetHeading();
        double[] targetPos = lightning.getPoseController().getTargetPosition();

        if (targetHeading != null && targetPos != null) {
            telemetry.addData("Target Pos", "X:%.1f Y:%.1f", targetPos[0], targetPos[1]);
            telemetry.addData("Target Heading", "%.1f°", targetHeading);

            // Calculate position error
            double posError = Math.hypot(pos[0] - targetPos[0], pos[1] - targetPos[1]);
            telemetry.addData("Position Error", "%.2f in", posError);

            double headingError = Math.abs(pos[2] - targetHeading);
            // Normalize heading error to 0-180 range
            if (headingError > 180) {
                headingError = 360 - headingError;
            }
            telemetry.addData("Heading Error", "%.1f°", headingError);

            // Show if in-place rotation should be active
            // Position is locked once robot gets within tolerance
            boolean posWithinTol = posError <= 2.0;  // posTolerance
            boolean headingNeedsCorrection = headingError > 5.0;  // > angTolerance in degrees

            if (posWithinTol && headingNeedsCorrection) {
                telemetry.addData("Mode", "POSITION LOCKED - ROTATING ✓");
                telemetry.addData("", "Pos locked, only correcting heading");
            } else if (posWithinTol) {
                telemetry.addData("Mode", "AT TARGET ✓");
            } else {
                telemetry.addData("Mode", "DRIVING (pos error %.1f\")", posError);
            }
        } else {
            telemetry.addData("Target", "NONE");
        }
    }

    /**
     * Display final status after movement
     */
    private void displayFinalStatus() {
        double[] finalPos = lightning.getCurrentPosition();

        telemetry.clear();
        telemetry.addData("=== MOVEMENT COMPLETE ===", "");
        telemetry.addData("", "");
        telemetry.addData("Final Position", "X:%.2f Y:%.2f H:%.1f°",
            finalPos[0], finalPos[1], finalPos[2]);
        telemetry.addData("Target Position", "X:%.1f Y:0.0 H:0.0°", FORWARD_DISTANCE);
        telemetry.addData("", "");

        double distanceError = Math.abs(finalPos[0] - FORWARD_DISTANCE);
        telemetry.addData("Distance Error", "%.2f inches", distanceError);

        if (distanceError <= 2.0) {
            telemetry.addLine("✓ Within tolerance!");
        } else {
            telemetry.addLine("⚠️ Outside tolerance");
        }

        telemetry.addData("", "");
        telemetry.addLine("Holding position...");
    }
}


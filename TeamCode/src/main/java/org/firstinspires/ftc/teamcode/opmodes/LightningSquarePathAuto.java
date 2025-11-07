package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.aurora.AuroraManager;
import org.firstinspires.ftc.teamcode.util.aurora.lightning.AuroraLightningCore;
import org.firstinspires.ftc.teamcode.util.tool.FieldMap;
import org.firstinspires.ftc.teamcode.util.tool.PathPlanner;
import org.firstinspires.ftc.teamcode.util.tool.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * LightningSquarePathAuto - Autonomous OpMode demonstrating AuroraLightningCore
 *
 * This OpMode:
 * - Sets starting position to (0, 0, 0)
 * - Moves forward to start position
 * - Follows a square path using waypoint navigation
 * - Returns to starting position
 *
 * Uses smooth curve path mode for natural movement
 */
@Autonomous(name="Lightning Square Path", group="Lightning Demo")
public class LightningSquarePathAuto extends LinearOpMode {

    // Robot systems
    private AuroraManager robotManager;
    private AuroraLightningCore lightning;
    private ElapsedTime runtime = new ElapsedTime();

    // Square dimensions (inches)
    private static final double SQUARE_SIZE = 24.0;

    @Override
    public void runOpMode() {
        // ========== INITIALIZATION ==========
        telemetry.addLine("Initializing Lightning Square Path...");
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
        // This tells the robot "you are at origin, facing 0 degrees"
        lightning.getPositionManager().updatePosition(0, 0, 0);
        lightning.getPositionManager().setStartPositionToCurrent();

        // Configure for smooth movement
        lightning.setPathMode(PathPlanner.PathMode.STRAIGHT);
        lightning.setSmoothingFactor(0.7);  // 0.0 = sharp corners, 1.0 = very smooth

        // Set tolerances for path following
        lightning.setPositionTolerance(2.0);  // Within 2 inches of waypoint
        lightning.setHeadingTolerance(5.0);   // Within 5 degrees of heading

        // Enable overshoot detection for smoother paths
        lightning.setOvershootDetection(true);

        telemetry.clear();
        telemetry.addLine("✅ Lightning Square Path Ready!");
        telemetry.addLine("");
        telemetry.addData("Start Position", "0, 0, 0°");
        telemetry.addData("Square Size", "%.0f inches", SQUARE_SIZE);
        telemetry.addLine("");
        telemetry.addLine("Press START to run square path");
        telemetry.update();

        // ========== WAIT FOR START ==========
        waitForStart();
        runtime.reset();

        // ========== RUN AUTONOMOUS SEQUENCE IN CONTINUOUS LOOP ==========
        while (opModeIsActive()) {
            runSquarePathSequence();

            // Break after sequence completes once
            break;
        }

        // ========== CLEANUP ==========
        lightning.stop();

        telemetry.addLine("✅ Square path autonomous complete!");
        telemetry.update();
    }

    /**
     * Run the square path sequence
     */
    private void runSquarePathSequence() {
        telemetry.addLine("Starting square path sequence...");
        telemetry.update();

        // Create square path waypoints
        List<Pose> squarePath = createSquarePath();

        // Load path into Lightning Core
        lightning.followPath(squarePath, PathPlanner.PathMode.STRAIGHT);

        telemetry.addLine("Path loaded, executing...");
        telemetry.update();

        // ========== MAIN LOOP ==========
        // Monitor path execution while OpMode is active
        while (opModeIsActive() && !lightning.isPathComplete()) {
            // Update all subsystems (MUST call every loop!)
            lightning.update();

            // Display telemetry
            displayTelemetry();
            telemetry.update();

            // Small sleep to prevent loop from running too fast
            sleep(10);
        }

        // ========== PATH COMPLETE ==========
        if (opModeIsActive()) {
            telemetry.addLine("✓ Square path complete!");
            telemetry.update();

            // Hold final position for 2 seconds
            lightning.holdPosition();

            double endTime = runtime.seconds() + 2.0;
            while (opModeIsActive() && runtime.seconds() < endTime) {
                lightning.update();
                displayTelemetry();
                telemetry.addLine("Holding position...");
                telemetry.update();
                sleep(10);
            }
        }
    }

    /**
     * Create waypoints for a square path
     * Square starts at origin, moves clockwise
     *
     * @return List of waypoints defining the square path
     */
    private List<Pose> createSquarePath() {
        List<Pose> path = new ArrayList<>();

        // Start at origin
        path.add(new Pose(0, 0, 0));

        // Corner 1: Move forward (positive X)
        path.add(new Pose(SQUARE_SIZE, 0, 0));

        // Corner 2: Turn and move left (positive Y)
        path.add(new Pose(SQUARE_SIZE, SQUARE_SIZE, 90));

        // Corner 3: Turn and move backward (negative X)
        path.add(new Pose(0, SQUARE_SIZE, 180));

        // Corner 4: Turn and return to origin (negative Y)
        path.add(new Pose(0, 0, 270));

        // Final: Face forward again
        path.add(new Pose(0, 0, 0));

        telemetry.addData("Path", "Created %d waypoints", path.size());

        return path;
    }

    /**
     * Display comprehensive telemetry information
     */
    private void displayTelemetry() {
        // Use Lightning Core's built-in telemetry
        lightning.addTelemetry();

        // Add custom info
        telemetry.addData("", "");
        telemetry.addData("=== SQUARE PATH ===", "");
        telemetry.addData("OpMode Active", opModeIsActive() ? "YES" : "NO");

        if (lightning.isFollowingPath()) {
            int remaining = lightning.getPoseController().getRemainingWaypoints();
            int total = lightning.getPoseController().getTotalWaypoints();
            int current = total - remaining;

            telemetry.addData("Current Waypoint", "%d / %d", current, total);

            // Show which corner we're heading to
            String corner = "Unknown";
            if (current == 0) corner = "Origin";
            else if (current == 1) corner = "Corner 1 (Forward)";
            else if (current == 2) corner = "Corner 2 (Left)";
            else if (current == 3) corner = "Corner 3 (Back)";
            else if (current == 4) corner = "Corner 4 (Return)";
            else if (current == 5) corner = "Final Position";

            telemetry.addData("Heading To", corner);
        } else if (lightning.isPathComplete()) {
            telemetry.addData("Status", "✓ PATH COMPLETE");
        }
    }
}


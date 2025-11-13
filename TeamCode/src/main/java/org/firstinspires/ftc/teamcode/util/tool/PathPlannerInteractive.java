package org.firstinspires.ftc.teamcode.util.tool;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/**
 * Interactive PathPlanner tool for terminal use
 * Input waypoints and get full path output for external plotting
 */
public class PathPlannerInteractive {

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        PathPlanner planner = new PathPlanner();

        System.out.println("=== Interactive PathPlanner ===\n");

        // Configure planner
        System.out.print("Waypoint spacing (inches) [default: 3.0]: ");
        String spacingInput = scanner.nextLine().trim();
        if (!spacingInput.isEmpty()) {
            planner.waypointSpacing = Double.parseDouble(spacingInput);
        }

        System.out.print("Smoothing factor (0.0-1.0) [default: 0.5]: ");
        String smoothingInput = scanner.nextLine().trim();
        if (!smoothingInput.isEmpty()) {
            planner.smoothingFactor = Double.parseDouble(smoothingInput);
        }

        System.out.print("Interpolate heading? (y/n) [default: y]: ");
        String headingInput = scanner.nextLine().trim().toLowerCase();
        if (!headingInput.isEmpty()) {
            planner.interpolateHeading = headingInput.equals("y");
        }

        System.out.print("Path mode (STRAIGHT/SMOOTH_CURVE/CUBIC_SPLINE) [default: SMOOTH_CURVE]: ");
        String modeInput = scanner.nextLine().trim().toUpperCase();
        PathPlanner.PathMode mode = PathPlanner.PathMode.SMOOTH_CURVE;
        if (!modeInput.isEmpty()) {
            mode = PathPlanner.PathMode.valueOf(modeInput);
        }

        // Input waypoints
        System.out.println("\nEnter waypoints (format: x,y,heading)");
        System.out.println("Example: 0,0,0 or 24.5,12.3,90");
        System.out.println("Enter empty line when done\n");

        List<Pose> waypoints = new ArrayList<>();
        int count = 1;

        while (true) {
            System.out.print("Waypoint " + count + ": ");
            String line = scanner.nextLine().trim();

            if (line.isEmpty()) {
                break;
            }

            try {
                String[] parts = line.split(",");
                double x = Double.parseDouble(parts[0].trim());
                double y = Double.parseDouble(parts[1].trim());
                double heading = Double.parseDouble(parts[2].trim());

                waypoints.add(new Pose(x, y, heading));
                count++;
            } catch (Exception e) {
                System.out.println("Invalid format. Try again (x,y,heading)");
            }
        }

        if (waypoints.size() < 2) {
            System.out.println("\nError: Need at least 2 waypoints");
            return;
        }

        // Generate path
        System.out.println("\nGenerating path...");
        List<Pose> path = planner.generatePath(waypoints, mode);

        // Output results
        System.out.println("\n=== Path Generated ===");
        System.out.println("Input waypoints: " + waypoints.size());
        System.out.println("Output waypoints: " + path.size());
        System.out.printf("Path length: %.2f inches%n", PathPlanner.calculatePathLength(path));
        System.out.println("Mode: " + mode);
        System.out.println();

        // Ask for output format
        System.out.println("Output format:");
        System.out.println("1. CSV (x,y,heading)");
        System.out.println("2. Java array initialization");
        System.out.println("3. Python list");
        System.out.println("4. Tab-separated (for Excel/Google Sheets)");
        System.out.print("Choose format [1]: ");

        String formatInput = scanner.nextLine().trim();
        int format = formatInput.isEmpty() ? 1 : Integer.parseInt(formatInput);

        System.out.println("\n=== COPY BELOW THIS LINE ===\n");

        switch (format) {
            case 1: // CSV
                System.out.println("x,y,heading");
                for (Pose p : path) {
                    System.out.printf("%.3f,%.3f,%.3f%n", p.x, p.y, p.heading);
                }
                break;

            case 2: // Java
                System.out.println("Pose[] path = new Pose[] {");
                for (int i = 0; i < path.size(); i++) {
                    Pose p = path.get(i);
                    System.out.printf("    new Pose(%.3f, %.3f, %.3f)%s%n",
                            p.x, p.y, p.heading, i < path.size() - 1 ? "," : "");
                }
                System.out.println("};");
                break;

            case 3: // Python
                System.out.println("path = [");
                for (int i = 0; i < path.size(); i++) {
                    Pose p = path.get(i);
                    System.out.printf("    (%.3f, %.3f, %.3f)%s%n",
                            p.x, p.y, p.heading, i < path.size() - 1 ? "," : "");
                }
                System.out.println("]");
                break;

            case 4: // Tab-separated
                System.out.println("x\ty\theading");
                for (Pose p : path) {
                    System.out.printf("%.3f\t%.3f\t%.3f%n", p.x, p.y, p.heading);
                }
                break;
        }

        System.out.println("\n=== END OF OUTPUT ===");

        scanner.close();
    }
}

package org.firstinspires.ftc.teamcode.util.tool;

import java.util.ArrayList;
import java.util.List;

/**
 * PathPlanner - Generates detailed paths from sparse waypoints
 *
 * Takes a list of waypoints and generates a smooth, detailed path based on the selected mode.
 * Supports straight-line segments, smooth curves, and spline interpolation.
 *
 * Usage:
 *   PathPlanner planner = new PathPlanner();
 *   List<Pose> waypoints = new ArrayList<>();
 *   waypoints.add(new Pose(0, 0, 0));
 *   waypoints.add(new Pose(24, 24, 90));
 *   waypoints.add(new Pose(48, 24, 0));
 *
 *   List<Pose> path = planner.generatePath(waypoints, PathMode.SMOOTH_CURVE);
 */
public class PathPlanner {

    /**
     * Path generation modes
     */
    public enum PathMode {
        /**
         * Straight line segments between waypoints with sharp corners
         * Fast to compute, simple motion
         */
        STRAIGHT,

        /**
         * Smooth curves between waypoints using Bezier-like smoothing
         * Good balance of smoothness and simplicity
         */
        SMOOTH_CURVE,

        /**
         * Cubic spline interpolation for very smooth paths
         * Smoothest motion, best for high-speed navigation
         */
        CUBIC_SPLINE
    }

    // ======== Configuration Parameters ========

    /**
     * Distance between generated waypoints (inches)
     * Smaller = more detail, more computation
     * Larger = less detail, faster
     */
    public double waypointSpacing = 3.0;  // inches

    /**
     * Smoothing factor for SMOOTH_CURVE mode (0.0 to 1.0)
     * 0.0 = sharp corners (like STRAIGHT)
     * 1.0 = maximum smoothing
     */
    public double smoothingFactor = 0.5;

    /**
     * Minimum number of points to generate between each waypoint pair
     * Ensures smooth motion even for short segments
     */
    public int minPointsPerSegment = 5;

    /**
     * Whether to interpolate heading smoothly along the path
     * true = gradual heading changes
     * false = heading changes only at waypoints
     */
    public boolean interpolateHeading = true;

    /**
     * Default constructor with standard settings
     */
    public PathPlanner() {
        // Use defaults
    }

    /**
     * Constructor with custom waypoint spacing
     * @param waypointSpacing Distance between generated waypoints (inches)
     */
    public PathPlanner(double waypointSpacing) {
        this.waypointSpacing = waypointSpacing;
    }

    // ======== Main Path Generation ========

    /**
     * Generate a detailed path from sparse waypoints
     *
     * @param waypoints List of waypoints to follow (at least 2 required)
     * @param mode Path generation mode (STRAIGHT, SMOOTH_CURVE, or CUBIC_SPLINE)
     * @return List of detailed waypoints forming the complete path
     */
    public List<Pose> generatePath(List<Pose> waypoints, PathMode mode) {
        if (waypoints == null || waypoints.size() < 2) {
            throw new IllegalArgumentException("At least 2 waypoints required");
        }

        switch (mode) {
            case STRAIGHT:
                return generateStraightPath(waypoints);
            case SMOOTH_CURVE:
                return generateSmoothCurvePath(waypoints);
            case CUBIC_SPLINE:
                return generateSplinePath(waypoints);
            default:
                return generateStraightPath(waypoints);
        }
    }

    /**
     * Convenience method - generate path starting from current position
     *
     * @param startPose Current robot pose
     * @param targetWaypoints Waypoints to navigate to
     * @param mode Path generation mode
     * @return Complete path including start position
     */
    public List<Pose> generatePathFromStart(Pose startPose, List<Pose> targetWaypoints, PathMode mode) {
        List<Pose> allWaypoints = new ArrayList<>();
        allWaypoints.add(startPose);
        allWaypoints.addAll(targetWaypoints);
        return generatePath(allWaypoints, mode);
    }

    // ======== Path Generation Algorithms ========

    /**
     * STRAIGHT mode - Simple linear interpolation between waypoints
     * Creates straight line segments with sharp corners
     */
    private List<Pose> generateStraightPath(List<Pose> waypoints) {
        List<Pose> path = new ArrayList<>();

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Pose start = waypoints.get(i);
            Pose end = waypoints.get(i + 1);

            // Calculate segment length
            double segmentLength = start.distanceTo(end);

            // Check if this is an in-place rotation (position doesn't change)
            boolean isInPlaceRotation = segmentLength < 0.1; // Less than 0.1 inches

            // Determine number of points for this segment
            int numPoints;
            if (isInPlaceRotation) {
                // For in-place rotations, just add the end point (no interpolation)
                numPoints = 1;
            } else {
                numPoints = Math.max(minPointsPerSegment,
                                    (int) Math.ceil(segmentLength / waypointSpacing));
            }

            // Generate points along the segment
            for (int j = 0; j < numPoints; j++) {
                double t = (double) j / numPoints;

                // Linear interpolation
                double x = start.x + (end.x - start.x) * t;
                double y = start.y + (end.y - start.y) * t;

                // Heading: don't interpolate for in-place rotations or if disabled
                double heading;
                if (isInPlaceRotation || !interpolateHeading) {
                    heading = start.heading; // Keep start heading until we reach the waypoint
                } else {
                    heading = start.heading + start.headingDifference(end) * t;
                }

                path.add(new Pose(x, y, heading));
            }
        }

        // Add final waypoint
        Pose lastWaypoint = waypoints.get(waypoints.size() - 1);
        path.add(lastWaypoint.copy());

        return path;
    }

    /**
     * SMOOTH_CURVE mode - Smoothed path using Catmull-Rom-like interpolation
     * Creates smooth curves through waypoints with adjustable corner rounding
     */
    private List<Pose> generateSmoothCurvePath(List<Pose> waypoints) {
        List<Pose> path = new ArrayList<>();

        // For smooth curves, we need at least 2 waypoints
        if (waypoints.size() == 2) {
            // Just use straight path for 2 points
            return generateStraightPath(waypoints);
        }

        // Process each segment with smoothing
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Pose current = waypoints.get(i);
            Pose next = waypoints.get(i + 1);

            // Get control points for smoothing
            Pose prev = (i > 0) ? waypoints.get(i - 1) : current;
            Pose afterNext = (i < waypoints.size() - 2) ? waypoints.get(i + 2) : next;

            // Calculate segment length
            double segmentLength = current.distanceTo(next);
            int numPoints = Math.max(minPointsPerSegment,
                                    (int) Math.ceil(segmentLength / waypointSpacing));

            // Generate smoothed points
            for (int j = 0; j < numPoints; j++) {
                double t = (double) j / numPoints;

                // Catmull-Rom spline for position
                Pose interpolated = catmullRomInterpolate(prev, current, next, afterNext, t);

                // Apply smoothing factor (blend with straight path)
                double blendedX = current.x + (next.x - current.x) * t;
                double blendedY = current.y + (next.y - current.y) * t;

                double finalX = blendedX + (interpolated.x - blendedX) * smoothingFactor;
                double finalY = blendedY + (interpolated.y - blendedY) * smoothingFactor;

                // Heading interpolation
                double heading = interpolateHeading ?
                    current.heading + current.headingDifference(next) * t : current.heading;

                path.add(new Pose(finalX, finalY, heading));
            }
        }

        // Add final waypoint
        path.add(waypoints.get(waypoints.size() - 1).copy());

        return path;
    }

    /**
     * CUBIC_SPLINE mode - Smooth cubic spline interpolation
     * Creates the smoothest paths, ideal for high-speed navigation
     */
    private List<Pose> generateSplinePath(List<Pose> waypoints) {
        List<Pose> path = new ArrayList<>();

        // For spline, we need at least 3 waypoints for proper curvature
        if (waypoints.size() < 3) {
            return generateSmoothCurvePath(waypoints);
        }

        // Calculate total path length for uniform parameterization
        double totalLength = 0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            totalLength += waypoints.get(i).distanceTo(waypoints.get(i + 1));
        }

        int totalPoints = Math.max(minPointsPerSegment * (waypoints.size() - 1),
                                   (int) Math.ceil(totalLength / waypointSpacing));

        // Generate uniformly spaced points along the spline
        for (int i = 0; i < totalPoints; i++) {
            double globalT = (double) i / totalPoints;

            // Find which segment this t belongs to
            int segmentIndex = (int) (globalT * (waypoints.size() - 1));
            segmentIndex = Math.min(segmentIndex, waypoints.size() - 2);

            // Local t within the segment
            double localT = globalT * (waypoints.size() - 1) - segmentIndex;

            // Get control points
            Pose p0 = (segmentIndex > 0) ? waypoints.get(segmentIndex - 1) : waypoints.get(segmentIndex);
            Pose p1 = waypoints.get(segmentIndex);
            Pose p2 = waypoints.get(segmentIndex + 1);
            Pose p3 = (segmentIndex < waypoints.size() - 2) ? waypoints.get(segmentIndex + 2) : waypoints.get(segmentIndex + 1);

            // Cubic spline interpolation
            Pose interpolated = cubicSplineInterpolate(p0, p1, p2, p3, localT);
            path.add(interpolated);
        }

        // Add final waypoint
        path.add(waypoints.get(waypoints.size() - 1).copy());

        return path;
    }

    // ======== Interpolation Helper Methods ========

    /**
     * Catmull-Rom spline interpolation
     * Creates smooth curves that pass through control points
     */
    private Pose catmullRomInterpolate(Pose p0, Pose p1, Pose p2, Pose p3, double t) {
        double t2 = t * t;
        double t3 = t2 * t;

        // Catmull-Rom matrix coefficients
        double x = 0.5 * ((2 * p1.x) +
                         (-p0.x + p2.x) * t +
                         (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                         (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);

        double y = 0.5 * ((2 * p1.y) +
                         (-p0.y + p2.y) * t +
                         (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                         (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);

        // Interpolate heading
        double heading = interpolateHeading ?
            p1.heading + p1.headingDifference(p2) * t : p1.heading;

        return new Pose(x, y, heading);
    }

    /**
     * Cubic spline interpolation using Hermite basis
     */
    private Pose cubicSplineInterpolate(Pose p0, Pose p1, Pose p2, Pose p3, double t) {
        // Calculate tangents at p1 and p2
        double m1x = (p2.x - p0.x) / 2.0;
        double m1y = (p2.y - p0.y) / 2.0;
        double m2x = (p3.x - p1.x) / 2.0;
        double m2y = (p3.y - p1.y) / 2.0;

        double t2 = t * t;
        double t3 = t2 * t;

        // Hermite basis functions
        double h00 = 2*t3 - 3*t2 + 1;
        double h10 = t3 - 2*t2 + t;
        double h01 = -2*t3 + 3*t2;
        double h11 = t3 - t2;

        // Calculate position
        double x = h00 * p1.x + h10 * m1x + h01 * p2.x + h11 * m2x;
        double y = h00 * p1.y + h10 * m1y + h01 * p2.y + h11 * m2y;

        // Interpolate heading
        double heading = interpolateHeading ?
            p1.heading + p1.headingDifference(p2) * t : p1.heading;

        return new Pose(x, y, heading);
    }

    // ======== Utility Methods ========

    /**
     * Calculate the total path length
     * @param path List of poses
     * @return Total length in inches
     */
    public static double calculatePathLength(List<Pose> path) {
        if (path.size() < 2) return 0;

        double length = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            length += path.get(i).distanceTo(path.get(i + 1));
        }
        return length;
    }

    /**
     * Get a pose at a specific distance along the path
     * @param path The path
     * @param distance Distance from start (inches)
     * @return Pose at that distance, or last pose if distance exceeds path length
     */
    public static Pose getPoseAtDistance(List<Pose> path, double distance) {
        if (path.isEmpty()) return null;
        if (distance <= 0) return path.get(0).copy();

        double accumulatedDistance = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            Pose current = path.get(i);
            Pose next = path.get(i + 1);
            double segmentLength = current.distanceTo(next);

            if (accumulatedDistance + segmentLength >= distance) {
                // The target distance is within this segment
                double t = (distance - accumulatedDistance) / segmentLength;
                return current.interpolate(next, t);
            }

            accumulatedDistance += segmentLength;
        }

        // Distance exceeds path length, return last pose
        return path.get(path.size() - 1).copy();
    }

    /**
     * Simplify a path by removing redundant waypoints
     * Useful for reducing path size while maintaining overall shape
     *
     * @param path Original path
     * @param tolerance Maximum deviation allowed (inches)
     * @return Simplified path
     */
    public static List<Pose> simplifyPath(List<Pose> path, double tolerance) {
        if (path.size() <= 2) return new ArrayList<>(path);

        List<Pose> simplified = new ArrayList<>();
        simplified.add(path.get(0));

        int i = 0;
        while (i < path.size() - 1) {
            int farthest = i + 1;

            // Find the farthest point we can skip to
            for (int j = i + 2; j < path.size(); j++) {
                if (perpendicularDistance(path.get(j), path.get(i), path.get(farthest)) <= tolerance) {
                    farthest = j;
                } else {
                    break;
                }
            }

            simplified.add(path.get(farthest));
            i = farthest;
        }

        return simplified;
    }

    /**
     * Calculate perpendicular distance from a point to a line segment
     */
    private static double perpendicularDistance(Pose point, Pose lineStart, Pose lineEnd) {
        double dx = lineEnd.x - lineStart.x;
        double dy = lineEnd.y - lineStart.y;

        if (dx == 0 && dy == 0) {
            return point.distanceTo(lineStart);
        }

        double t = ((point.x - lineStart.x) * dx + (point.y - lineStart.y) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));

        double closestX = lineStart.x + t * dx;
        double closestY = lineStart.y + t * dy;

        return Math.hypot(point.x - closestX, point.y - closestY);
    }

    /**
     * Reverse a path
     * @param path Original path
     * @return Reversed path
     */
    public static List<Pose> reversePath(List<Pose> path) {
        List<Pose> reversed = new ArrayList<>();
        for (int i = path.size() - 1; i >= 0; i--) {
            Pose p = path.get(i);
            // Reverse heading by 180 degrees
            reversed.add(new Pose(p.x, p.y, p.heading + 180));
        }
        return reversed;
    }
}


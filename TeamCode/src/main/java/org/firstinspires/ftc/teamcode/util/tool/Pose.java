package org.firstinspires.ftc.teamcode.util.tool;

/**
 * Represents a 2D pose (position + heading) in field coordinates
 */
public class Pose {
    public double x;        // X position in field frame (inches)
    public double y;        // Y position in field frame (inches)
    public double heading;  // Heading in degrees (0 = +X axis, CCW positive)

    /**
     * Create a new pose
     * @param x X position (inches)
     * @param y Y position (inches)
     * @param heading Heading (degrees)
     */
    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Create a copy of this pose
     */
    public Pose copy() {
        return new Pose(x, y, heading);
    }

    /**
     * Calculate distance to another pose (ignoring heading)
     * @param other The other pose
     * @return Distance in inches
     */
    public double distanceTo(Pose other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }

    /**
     * Calculate heading difference to another pose (ignoring positions)
     * @param other The other pose
     * @return Heading difference in degrees (-180 to 180)
     */
    public double headingDifference(Pose other) {
        double diff = other.heading - this.heading;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    /**
     * Interpolate between this pose and another
     * @param other The other pose
     * @param t Interpolation parameter (0 = this pose, 1 = other pose)
     * @return Interpolated pose
     */
    public Pose interpolate(Pose other, double t) {
        double newX = this.x + (other.x - this.x) * t;
        double newY = this.y + (other.y - this.y) * t;

        // Interpolate heading with proper wrapping
        double headingDiff = headingDifference(other);
        double newHeading = this.heading + headingDiff * t;

        return new Pose(newX, newY, newHeading);
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "Pose(x=%.2f, y=%.2f, θ=%.1f°)", x, y, heading);
    }
}


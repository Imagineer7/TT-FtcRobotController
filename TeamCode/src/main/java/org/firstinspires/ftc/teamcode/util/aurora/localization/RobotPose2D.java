package org.firstinspires.ftc.teamcode.util.aurora.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Robot pose representation with uncertainty and velocity.
 * 
 * This class extends the basic 2D pose with:
 * - Covariance matrix (3x3) for uncertainty quantification
 * - Velocity components (vx, vy, vheading)
 * - Timestamp for latency compensation
 * 
 * State vector: [x, y, heading]^T
 * - x: position in field frame (mm)
 * - y: position in field frame (mm)
 * - heading: orientation (radians, CCW from +X axis)
 */
public class RobotPose2D {
    
    // Pose state
    public double x;            // mm, field frame
    public double y;            // mm, field frame
    public double heading;      // radians, CCW from +X
    
    // Uncertainty (3x3 covariance matrix)
    public Matrix3x3 covariance;
    
    // Velocity
    public double vx;           // mm/s, field frame
    public double vy;           // mm/s, field frame
    public double vheading;     // rad/s
    
    // Timestamp
    public long timestamp;      // milliseconds since epoch
    
    /**
     * Create pose at origin with zero uncertainty
     */
    public RobotPose2D() {
        this(0, 0, 0);
    }
    
    /**
     * Create pose with specified position and heading
     */
    public RobotPose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.covariance = Matrix3x3.identity();
        this.timestamp = System.currentTimeMillis();
    }
    
    /**
     * Create pose with specified position, heading, and covariance
     */
    public RobotPose2D(double x, double y, double heading, Matrix3x3 covariance) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.covariance = covariance.copy();
        this.timestamp = System.currentTimeMillis();
    }
    
    /**
     * Create from FTC SDK Pose2D
     */
    public static RobotPose2D fromPose2D(Pose2D pose) {
        return new RobotPose2D(
            pose.getX(DistanceUnit.MM),
            pose.getY(DistanceUnit.MM),
            pose.getHeading(AngleUnit.RADIANS)
        );
    }
    
    /**
     * Convert to FTC SDK Pose2D
     */
    public Pose2D toPose2D() {
        return new Pose2D(
            DistanceUnit.MM, x, y,
            AngleUnit.RADIANS, heading
        );
    }
    
    /**
     * Get X position in specified unit
     */
    public double getX(DistanceUnit unit) {
        return unit.fromMm(x);
    }
    
    /**
     * Get Y position in specified unit
     */
    public double getY(DistanceUnit unit) {
        return unit.fromMm(y);
    }
    
    /**
     * Get heading in specified unit
     */
    public double getHeading(AngleUnit unit) {
        return unit.fromRadians(heading);
    }
    
    /**
     * Get velocity magnitude in specified unit
     */
    public double getVelocityMagnitude(DistanceUnit unit) {
        double vmag = Math.sqrt(vx*vx + vy*vy);
        return unit.fromMm(vmag);
    }
    
    /**
     * Get heading velocity in specified unit
     */
    public double getHeadingVelocity(AngleUnit unit) {
        return unit.fromRadians(vheading);
    }
    
    /**
     * Compute distance to another pose
     */
    public double distanceTo(RobotPose2D other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.sqrt(dx*dx + dy*dy);
    }
    
    /**
     * Compute angle to another pose
     */
    public double angleTo(RobotPose2D other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.atan2(dy, dx);
    }
    
    /**
     * Get pose uncertainty (trace of covariance matrix)
     * Returns scalar measure of total uncertainty in mm
     */
    public double getUncertainty() {
        return Math.sqrt(covariance.trace());
    }
    
    /**
     * Get position uncertainty (x, y covariance only)
     */
    public double getPositionUncertainty() {
        double var_x = covariance.get(0, 0);
        double var_y = covariance.get(1, 1);
        return Math.sqrt(var_x + var_y);
    }
    
    /**
     * Get heading uncertainty in radians
     */
    public double getHeadingUncertainty() {
        return Math.sqrt(covariance.get(2, 2));
    }
    
    /**
     * Create a copy of this pose
     */
    public RobotPose2D copy() {
        RobotPose2D copy = new RobotPose2D(x, y, heading, covariance);
        copy.vx = this.vx;
        copy.vy = this.vy;
        copy.vheading = this.vheading;
        copy.timestamp = this.timestamp;
        return copy;
    }
    
    /**
     * Interpolate between this pose and another
     * @param other Target pose
     * @param alpha Interpolation factor (0 = this, 1 = other)
     * @return Interpolated pose
     */
    public RobotPose2D interpolate(RobotPose2D other, double alpha) {
        double newX = (1 - alpha) * this.x + alpha * other.x;
        double newY = (1 - alpha) * this.y + alpha * other.y;
        
        // Angular interpolation (handle wraparound)
        double dHeading = angleWrap(other.heading - this.heading);
        double newHeading = angleWrap(this.heading + alpha * dHeading);
        
        // Interpolate covariance
        Matrix3x3 newCov = this.covariance.multiply(1 - alpha)
                                           .add(other.covariance.multiply(alpha));
        
        RobotPose2D result = new RobotPose2D(newX, newY, newHeading, newCov);
        result.timestamp = (long)((1 - alpha) * this.timestamp + alpha * other.timestamp);
        
        return result;
    }
    
    /**
     * Wrap angle to [-pi, pi]
     */
    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Normalize heading to [-pi, pi]
     */
    public void normalizeHeading() {
        heading = angleWrap(heading);
    }
    
    @Override
    public String toString() {
        return String.format("Pose2D(x=%.1f mm, y=%.1f mm, heading=%.1f°, uncertainty=%.1f mm)",
            x, y, Math.toDegrees(heading), getUncertainty());
    }
    
    /**
     * Detailed string with velocity and covariance
     */
    public String toDetailedString() {
        return String.format(
            "Pose2D:\n" +
            "  Position: (%.1f, %.1f) mm\n" +
            "  Heading: %.1f°\n" +
            "  Velocity: (%.1f, %.1f) mm/s, %.1f°/s\n" +
            "  Uncertainty: pos=%.1f mm, heading=%.1f°\n" +
            "  Timestamp: %d ms",
            x, y, Math.toDegrees(heading),
            vx, vy, Math.toDegrees(vheading),
            getPositionUncertainty(), Math.toDegrees(getHeadingUncertainty()),
            timestamp
        );
    }
}

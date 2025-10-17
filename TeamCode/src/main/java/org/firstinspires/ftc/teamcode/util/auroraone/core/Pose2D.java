package org.firstinspires.ftc.teamcode.util.auroraone.core;

import org.firstinspires.ftc.teamcode.util.auroraone.config.Tunables;

/**
 * AURORA ONE - Pose2D Class
 *
 * This class represents a 2D pose, typically consisting of position (x, y) and orientation (theta).
 * It is commonly used in robotics for navigation and localization purposes.
 * It can be expanded with methods for transformations, distance calculations, and other pose-related operations.
 *
 * Features:
 * - Immutable pose representation for thread safety
 * - Comprehensive transformation operations
 * - Distance and angle calculations
 * - Integration with Blackboard for real-time data
 * - Path planning and navigation utilities
 * - Field coordinate system support
 * - State machine integration
 */
public class Pose2D {

    // Core pose components
    private final double x;      // X coordinate in inches
    private final double y;      // Y coordinate in inches
    private final double theta;  // Heading in degrees (-180 to 180)

    // Metadata
    private final long timestamp;
    private final double confidence;

    // Constants
    public static final Pose2D ORIGIN = new Pose2D(0, 0, 0);
    public static final double TOLERANCE_POSITION = 2.0;  // 2 inch position tolerance
    public static final double TOLERANCE_HEADING = 5.0;   // 5 degree heading tolerance

    /**
     * Primary constructor
     */
    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = normalizeAngle(theta);
        this.timestamp = System.currentTimeMillis();
        this.confidence = 1.0;
    }

    /**
     * Constructor with confidence
     */
    public Pose2D(double x, double y, double theta, double confidence) {
        this.x = x;
        this.y = y;
        this.theta = normalizeAngle(theta);
        this.timestamp = System.currentTimeMillis();
        this.confidence = Math.max(0.0, Math.min(1.0, confidence));
    }

    /**
     * Constructor with timestamp and confidence
     */
    public Pose2D(double x, double y, double theta, long timestamp, double confidence) {
        this.x = x;
        this.y = y;
        this.theta = normalizeAngle(theta);
        this.timestamp = timestamp;
        this.confidence = Math.max(0.0, Math.min(1.0, confidence));
    }

    /**
     * Copy constructor
     */
    public Pose2D(Pose2D other) {
        this.x = other.x;
        this.y = other.y;
        this.theta = other.theta;
        this.timestamp = other.timestamp;
        this.confidence = other.confidence;
    }

    // === GETTERS ===

    public double getX() { return x; }
    public double getY() { return y; }
    public double getTheta() { return theta; }
    public double getHeading() { return theta; }
    public long getTimestamp() { return timestamp; }
    public double getConfidence() { return confidence; }

    /**
     * Get pose as array [x, y, theta]
     */
    public double[] toArray() {
        return new double[]{x, y, theta};
    }

    /**
     * Get position as array [x, y]
     */
    public double[] getPosition() {
        return new double[]{x, y};
    }

    // === TRANSFORMATION OPERATIONS ===

    /**
     * Translate pose by given offsets
     */
    public Pose2D translate(double dx, double dy) {
        return new Pose2D(x + dx, y + dy, theta, timestamp, confidence);
    }

    /**
     * Rotate pose by given angle
     */
    public Pose2D rotate(double deltaTheta) {
        return new Pose2D(x, y, theta + deltaTheta, timestamp, confidence);
    }

    /**
     * Transform this pose by another pose (composition)
     */
    public Pose2D transformBy(Pose2D transform) {
        double cos = Math.cos(Math.toRadians(theta));
        double sin = Math.sin(Math.toRadians(theta));

        double newX = x + transform.x * cos - transform.y * sin;
        double newY = y + transform.x * sin + transform.y * cos;
        double newTheta = theta + transform.theta;

        return new Pose2D(newX, newY, newTheta, timestamp, confidence);
    }

    /**
     * Get the inverse of this pose
     */
    public Pose2D inverse() {
        double cos = Math.cos(Math.toRadians(-theta));
        double sin = Math.sin(Math.toRadians(-theta));

        double newX = -x * cos + y * sin;
        double newY = -x * sin - y * cos;
        double newTheta = -theta;

        return new Pose2D(newX, newY, newTheta, timestamp, confidence);
    }

    /**
     * Transform from this pose to another pose (relative pose)
     */
    public Pose2D relativeTo(Pose2D other) {
        return this.inverse().transformBy(other);
    }

    // === DISTANCE AND ANGLE CALCULATIONS ===

    /**
     * Calculate Euclidean distance to another pose
     */
    public double distanceTo(Pose2D other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate angle to another pose (in degrees)
     */
    public double angleTo(Pose2D other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return normalizeAngle(Math.toDegrees(Math.atan2(dy, dx)));
    }

    /**
     * Calculate heading error to face another pose
     */
    public double headingErrorTo(Pose2D other) {
        double targetAngle = angleTo(other);
        return normalizeAngle(targetAngle - this.theta);
    }

    /**
     * Calculate Manhattan distance (L1 norm)
     */
    public double manhattanDistanceTo(Pose2D other) {
        return Math.abs(other.x - this.x) + Math.abs(other.y - this.y);
    }

    // === GEOMETRIC OPERATIONS ===

    /**
     * Linear interpolation between this pose and another
     */
    public Pose2D interpolate(Pose2D other, double t) {
        t = Math.max(0.0, Math.min(1.0, t)); // Clamp t to [0, 1]

        double newX = x + t * (other.x - x);
        double newY = y + t * (other.y - y);
        double newTheta = interpolateAngle(theta, other.theta, t);
        double newConfidence = confidence + t * (other.confidence - confidence);

        return new Pose2D(newX, newY, newTheta, timestamp, newConfidence);
    }

    /**
     * Get unit vector in the direction of the pose heading
     */
    public double[] getHeadingVector() {
        double radians = Math.toRadians(theta);
        return new double[]{Math.cos(radians), Math.sin(radians)};
    }

    /**
     * Project this pose forward by a given distance
     */
    public Pose2D projectForward(double distance) {
        double[] headingVec = getHeadingVector();
        return new Pose2D(x + distance * headingVec[0], y + distance * headingVec[1], theta, timestamp, confidence);
    }

    /**
     * Check if this pose is within tolerance of another pose
     */
    public boolean isNear(Pose2D other, double positionTolerance, double headingTolerance) {
        double positionError = distanceTo(other);
        double headingError = Math.abs(normalizeAngle(other.theta - this.theta));

        return positionError <= positionTolerance && headingError <= headingTolerance;
    }

    /**
     * Check if this pose is near another using default tolerances
     */
    public boolean isNear(Pose2D other) {
        return isNear(other, TOLERANCE_POSITION, TOLERANCE_HEADING);
    }

    // === FIELD COORDINATE UTILITIES ===

    /**
     * Check if pose is within field boundaries
     */
    public boolean isWithinField() {
        return isWithinField(6.0); // 6 inch margin
    }

    /**
     * Check if pose is within field boundaries with margin
     */
    public boolean isWithinField(double margin) {
        return x >= -margin && x <= (Tunables.FIELD_WIDTH + margin) &&
               y >= -margin && y <= (Tunables.FIELD_LENGTH + margin);
    }

    /**
     * Constrain pose to field boundaries
     */
    public Pose2D constrainToField() {
        return constrainToField(6.0);
    }

    /**
     * Constrain pose to field boundaries with margin
     */
    public Pose2D constrainToField(double margin) {
        double constrainedX = Math.max(-margin, Math.min(Tunables.FIELD_WIDTH + margin, x));
        double constrainedY = Math.max(-margin, Math.min(Tunables.FIELD_LENGTH + margin, y));

        return new Pose2D(constrainedX, constrainedY, theta, timestamp, confidence);
    }

    /**
     * Mirror pose across the field (for alliance switching)
     */
    public Pose2D mirrorAcrossField() {
        return new Pose2D(Tunables.FIELD_WIDTH - x, y, normalizeAngle(180 - theta), timestamp, confidence);
    }

    // === BLACKBOARD INTEGRATION ===

    /**
     * Get current robot pose from Blackboard
     */
    public static Pose2D fromBlackboard() {
        Blackboard blackboard = Blackboard.getInstance();

        double x = blackboard.get("robot.position.x", 0.0);
        double y = blackboard.get("robot.position.y", 0.0);
        double theta = blackboard.get("robot.position.heading", 0.0);
        double confidence = blackboard.get("robot.position.confidence", 0.0);
        long timestamp = blackboard.get("robot.position.last_update", 0L);

        return new Pose2D(x, y, theta, timestamp, confidence);
    }

    /**
     * Update Blackboard with this pose
     */
    public void toBlackboard() {
        Blackboard blackboard = Blackboard.getInstance();

        blackboard.put("robot.position.x", x);
        blackboard.put("robot.position.y", y);
        blackboard.put("robot.position.heading", theta);
        blackboard.put("robot.position.confidence", confidence);
        blackboard.put("robot.position.last_update", timestamp);
    }

    /**
     * Set target pose in Blackboard for autonomous navigation
     */
    public void setAsTarget() {
        Blackboard blackboard = Blackboard.getInstance();

        blackboard.put("drive.target.x", x);
        blackboard.put("drive.target.y", y);
        blackboard.put("drive.target.heading", theta);
        blackboard.put("drive.auto.active", true);
    }

    // === STATE MACHINE INTEGRATION ===

    /**
     * Calculate error vector to target pose (for control systems)
     */
    public PoseError errorTo(Pose2D target) {
        double positionError = distanceTo(target);
        double headingError = normalizeAngle(target.theta - this.theta);
        double crossTrackError = calculateCrossTrackError(target);

        return new PoseError(positionError, headingError, crossTrackError);
    }

    /**
     * Calculate cross-track error (perpendicular distance from intended path)
     */
    private double calculateCrossTrackError(Pose2D target) {
        double dx = target.x - this.x;
        double dy = target.y - this.y;
        double targetHeadingRad = Math.toRadians(target.theta);

        // Project displacement onto perpendicular to target heading
        return -dx * Math.sin(targetHeadingRad) + dy * Math.cos(targetHeadingRad);
    }

    /**
     * Check if this pose represents a valid robot position
     */
    public boolean isValid() {
        return !Double.isNaN(x) && !Double.isNaN(y) && !Double.isNaN(theta) &&
               Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(theta) &&
               confidence >= 0.0 && confidence <= 1.0;
    }

    /**
     * Get pose age in milliseconds
     */
    public long getAge() {
        return System.currentTimeMillis() - timestamp;
    }

    /**
     * Check if pose is stale (older than threshold)
     */
    public boolean isStale(long maxAgeMs) {
        return getAge() > maxAgeMs;
    }

    // === UTILITY METHODS ===

    /**
     * Normalize angle to [-180, 180] degrees
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Interpolate between two angles (handles wrap-around)
     */
    private static double interpolateAngle(double angle1, double angle2, double t) {
        double diff = normalizeAngle(angle2 - angle1);
        return normalizeAngle(angle1 + t * diff);
    }

    /**
     * Convert degrees to radians
     */
    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees
     */
    public static double toDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    // === FACTORY METHODS ===

    /**
     * Create pose from polar coordinates (distance, angle from origin)
     */
    public static Pose2D fromPolar(double distance, double angle) {
        double x = distance * Math.cos(Math.toRadians(angle));
        double y = distance * Math.sin(Math.toRadians(angle));
        return new Pose2D(x, y, angle);
    }

    /**
     * Create pose from array [x, y, theta]
     */
    public static Pose2D fromArray(double[] array) {
        if (array.length < 3) {
            throw new IllegalArgumentException("Array must have at least 3 elements [x, y, theta]");
        }
        return new Pose2D(array[0], array[1], array[2]);
    }

    /**
     * Create pose with current timestamp
     */
    public static Pose2D withTimestamp(double x, double y, double theta) {
        return new Pose2D(x, y, theta, System.currentTimeMillis(), 1.0);
    }

    // === COMPARISON AND EQUALITY ===

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;

        Pose2D pose2D = (Pose2D) obj;
        return Double.compare(pose2D.x, x) == 0 &&
               Double.compare(pose2D.y, y) == 0 &&
               Double.compare(pose2D.theta, theta) == 0;
    }

    @Override
    public int hashCode() {
        int result = Double.hashCode(x);
        result = 31 * result + Double.hashCode(y);
        result = 31 * result + Double.hashCode(theta);
        return result;
    }

    @Override
    public String toString() {
        return String.format("Pose2D{x=%.2f, y=%.2f, θ=%.1f°, conf=%.2f, age=%dms}",
                           x, y, theta, confidence, getAge());
    }

    /**
     * Compact string representation
     */
    public String toCompactString() {
        return String.format("(%.1f, %.1f, %.0f°)", x, y, theta);
    }

    /**
     * Detailed string with all fields
     */
    public String toDetailedString() {
        return String.format("Pose2D{x=%.3f, y=%.3f, θ=%.2f°, timestamp=%d, confidence=%.3f, age=%dms, valid=%b}",
                           x, y, theta, timestamp, confidence, getAge(), isValid());
    }

    // === INNER CLASSES ===

    /**
     * Container for pose error calculations
     */
    public static class PoseError {
        public final double positionError;
        public final double headingError;
        public final double crossTrackError;

        public PoseError(double positionError, double headingError, double crossTrackError) {
            this.positionError = positionError;
            this.headingError = headingError;
            this.crossTrackError = crossTrackError;
        }

        public boolean isWithinTolerance(double positionTol, double headingTol) {
            return positionError <= positionTol && Math.abs(headingError) <= headingTol;
        }

        public boolean isWithinTolerance() {
            return isWithinTolerance(TOLERANCE_POSITION, TOLERANCE_HEADING);
        }

        @Override
        public String toString() {
            return String.format("PoseError{pos=%.2f, heading=%.1f°, cross=%.2f}",
                               positionError, headingError, crossTrackError);
        }
    }

    // === COMMON FIELD POSITIONS ===

    /**
     * Pre-defined field positions for common locations
     */
    public static class FieldPositions {
        // Starting positions (adjust based on actual field)
        public static final Pose2D RED_START_LEFT = new Pose2D(12, 12, 0);
        public static final Pose2D RED_START_RIGHT = new Pose2D(12, 132, 0);
        public static final Pose2D BLUE_START_LEFT = new Pose2D(132, 12, 180);
        public static final Pose2D BLUE_START_RIGHT = new Pose2D(132, 132, 180);

        // Center positions
        public static final Pose2D FIELD_CENTER = new Pose2D(Tunables.FIELD_WIDTH / 2, Tunables.FIELD_LENGTH / 2, 0);

        // Scoring positions (examples - adjust for specific game)
        public static final Pose2D RED_SCORING_ZONE = new Pose2D(24, 72, 0);
        public static final Pose2D BLUE_SCORING_ZONE = new Pose2D(120, 72, 180);

        /**
         * Get alliance-specific starting position
         */
        public static Pose2D getStartPosition(String alliance, boolean isLeft) {
            if ("RED".equalsIgnoreCase(alliance)) {
                return isLeft ? RED_START_LEFT : RED_START_RIGHT;
            } else {
                return isLeft ? BLUE_START_LEFT : BLUE_START_RIGHT;
            }
        }
    }
}

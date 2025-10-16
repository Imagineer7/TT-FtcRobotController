package org.firstinspires.ftc.teamcode.util.aurora.auto;

/**
 * Aurora Motor Configuration
 *
 * Updated for your specific robot configuration:
 * - goBILDA motors with 19.2:1 gearbox, 537.7 PPR encoders
 * - 104mm (4.094 inch) diameter wheels
 * - REV Through Bore encoders for dead wheel odometry
 */
public class AuroraMotorConfig {

    // =================================================================
    // DRIVE MOTOR ENCODER SETTINGS (Updated for your goBILDA motors)
    // =================================================================

    // Motor Type Configuration - Your specific goBILDA setup
    public static final MotorType DRIVE_MOTOR_TYPE = MotorType.GOBILDA_CUSTOM_19_2;

    // Wheel Configuration - Your 104mm wheels
    public static final double WHEEL_DIAMETER_INCHES = 4.094;    // 104mm = 4.094 inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI; // ~12.87 inches

    // Drive Motor Encoder Constants (calculated from your motor specs)
    public static final double DRIVE_MOTOR_TICKS_PER_REV = DRIVE_MOTOR_TYPE.ticksPerRevolution; // 537.7
    public static final double DRIVE_MOTOR_TICKS_PER_INCH = DRIVE_MOTOR_TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ~41.8 ticks/inch

    // =================================================================
    // DEAD WHEEL ODOMETRY SETTINGS (from existing DeadWheelOdometry)
    // =================================================================

    // Dead Wheel Configuration
    public static final double DEAD_WHEEL_DIAMETER_INCHES = 2.0;    // Odometry wheel diameter
    public static final double DEAD_WHEEL_CIRCUMFERENCE = DEAD_WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double DEAD_WHEEL_ENCODER_TICKS_PER_REV = 8192.0;  // REV Through Bore
    public static final double DEAD_WHEEL_TICKS_PER_INCH = DEAD_WHEEL_ENCODER_TICKS_PER_REV / DEAD_WHEEL_CIRCUMFERENCE; // ~1303.8 ticks/inch

    // Robot Geometry (from DeadWheelOdometry)
    public static final double TRACK_WIDTH_INCHES = 12.0;        // Distance between left/right dead wheels
    public static final double HORIZONTAL_OFFSET_INCHES = 6.0;   // Forward offset of horizontal wheel

    // =================================================================
    // MOTOR TYPE ENUMERATION
    // =================================================================

    public enum MotorType {
        // REV Robotics Motors
        REV_HD_HEX(28.0, "REV HD Hex Motor"),
        REV_CORE_HEX(288.0, "REV Core Hex Motor"),
        REV_THROUGH_BORE(8192.0, "REV Through Bore Encoder"),

        // goBILDA Motors
        GOBILDA_5202(537.7, "goBILDA 5202 Series"),
        GOBILDA_5203(537.7, "goBILDA 5203 Series"),
        GOBILDA_5204(1425.1, "goBILDA 5204 Series"),

        // Your Custom goBILDA Motor
        GOBILDA_CUSTOM_19_2(537.7, "goBILDA Custom 19.2:1 Motor (RS-555, Planetary, Magnetic Encoder)"),

        // AndyMark Motors
        ANDYMARK_NEVEREST_20(537.7, "AndyMark NeveRest 20"),
        ANDYMARK_NEVEREST_40(1120.0, "AndyMark NeveRest 40"),
        ANDYMARK_NEVEREST_60(1680.0, "AndyMark NeveRest 60"),

        // Tetrix Motors
        TETRIX_TORQUENADO(1440.0, "Tetrix TorqueNADO");

        public final double ticksPerRevolution;
        public final String description;

        MotorType(double ticksPerRevolution, String description) {
            this.ticksPerRevolution = ticksPerRevolution;
            this.description = description;
        }
    }

    // =================================================================
    // UTILITY METHODS
    // =================================================================

    /**
     * Convert drive motor encoder ticks to inches
     */
    public static double driveTicksToInches(int ticks) {
        return ticks / DRIVE_MOTOR_TICKS_PER_INCH;
    }

    /**
     * Convert inches to drive motor encoder ticks
     */
    public static int inchesToDriveTicks(double inches) {
        return (int) Math.round(inches * DRIVE_MOTOR_TICKS_PER_INCH);
    }

    /**
     * Convert dead wheel encoder ticks to inches
     */
    public static double deadWheelTicksToInches(int ticks) {
        return ticks / DEAD_WHEEL_TICKS_PER_INCH;
    }

    /**
     * Convert inches to dead wheel encoder ticks
     */
    public static int inchesToDeadWheelTicks(double inches) {
        return (int) Math.round(inches * DEAD_WHEEL_TICKS_PER_INCH);
    }

    /**
     * Get configuration summary for telemetry
     */
    public static String getConfigSummary() {
        return String.format(
            "Drive: %s (%.1f ticks/inch) | Dead Wheel: %.1f ticks/inch",
            DRIVE_MOTOR_TYPE.description,
            DRIVE_MOTOR_TICKS_PER_INCH,
            DEAD_WHEEL_TICKS_PER_INCH
        );
    }

    /**
     * Get detailed configuration for debugging
     */
    public static String getDetailedConfig() {
        return String.format(
            "=== AURORA MOTOR CONFIG ===\n" +
            "Drive Motor: %s\n" +
            "Drive Ticks/Rev: %.1f\n" +
            "Drive Wheel Diameter: %.3f\" (104mm)\n" +
            "Drive Wheel Circumference: %.2f\"\n" +
            "Drive Ticks/Inch: %.1f\n" +
            "\n" +
            "Dead Wheel Ticks/Rev: %.1f\n" +
            "Dead Wheel Diameter: %.1f\"\n" +
            "Dead Wheel Ticks/Inch: %.1f\n" +
            "\n" +
            "Track Width: %.1f\"\n" +
            "Horizontal Offset: %.1f\"\n" +
            "\n" +
            "Motor Specs:\n" +
            "- RS-555 Brushed DC Motor\n" +
            "- 19.2:1 Planetary Gearbox\n" +
            "- Magnetic Hall Effect Encoder\n" +
            "- 312 RPM @ 12VDC (No Load)\n" +
            "- 24.3 kg⋅cm Stall Torque",
            DRIVE_MOTOR_TYPE.description,
            DRIVE_MOTOR_TICKS_PER_REV,
            WHEEL_DIAMETER_INCHES,
            WHEEL_CIRCUMFERENCE,
            DRIVE_MOTOR_TICKS_PER_INCH,
            DEAD_WHEEL_ENCODER_TICKS_PER_REV,
            DEAD_WHEEL_DIAMETER_INCHES,
            DEAD_WHEEL_TICKS_PER_INCH,
            TRACK_WIDTH_INCHES,
            HORIZONTAL_OFFSET_INCHES
        );
    }

    /**
     * Get motor performance characteristics
     */
    public static String getMotorPerformance() {
        double maxLinearSpeed = (312.0 / 60.0) * WHEEL_CIRCUMFERENCE; // RPM to inches/second
        double ticksPerSecond = maxLinearSpeed * DRIVE_MOTOR_TICKS_PER_INCH;

        return String.format(
            "=== MOTOR PERFORMANCE ===\n" +
            "Max No-Load Speed: 312 RPM\n" +
            "Max Linear Speed: %.1f in/sec (%.1f ft/sec)\n" +
            "Max Encoder Rate: %.0f ticks/sec\n" +
            "Stall Current: 9.2A per motor\n" +
            "No-Load Current: 0.25A per motor",
            maxLinearSpeed,
            maxLinearSpeed / 12.0,
            ticksPerSecond
        );
    }
}

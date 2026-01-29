package org.firstinspires.ftc.teamcode.util.aurora.localization;

/**
 * Predefined starting positions for the DECODE 2025-2026 season.
 * 
 * Field Coordinate System:
 * - Origin: Field center
 * - +X axis: Toward RED alliance wall
 * - +Y axis: Toward AUDIENCE wall (driver stations)
 * - Heading: 0° = facing +X (toward red), CCW positive
 * 
 * All positions in millimeters, headings in radians.
 */
public class PredefinedPoses {
    
    // Field dimensions (DECODE 2025-2026)
    private static final double FIELD_WIDTH_MM = 3658.0;  // 144 inches
    private static final double FIELD_HEIGHT_MM = 3658.0; // 144 inches
    
    // Starting zone offsets from field edges
    private static final double START_ZONE_OFFSET_X = 400.0;  // Distance from alliance wall
    private static final double START_ZONE_OFFSET_Y = 600.0;  // Distance from side walls
    
    /**
     * Available starting positions
     */
    public enum StartPosition {
        /**
         * Red alliance, audience side, left position
         * Starting position closest to red alliance, audience wall, left side
         */
        RED_AUDIENCE_LEFT,
        
        /**
         * Red alliance, audience side, right position  
         * Starting position closest to red alliance, audience wall, right side
         */
        RED_AUDIENCE_RIGHT,
        
        /**
         * Blue alliance, audience side, left position
         * Starting position closest to blue alliance, audience wall, left side
         */
        BLUE_AUDIENCE_LEFT,
        
        /**
         * Blue alliance, audience side, right position
         * Starting position closest to blue alliance, audience wall, right side
         */
        BLUE_AUDIENCE_RIGHT,
        
        /**
         * Red alliance, net zone side
         * Starting position closest to red alliance, opposite audience wall
         */
        RED_NET_ZONE,
        
        /**
         * Blue alliance, net zone side
         * Starting position closest to blue alliance, opposite audience wall
         */
        BLUE_NET_ZONE
    }
    
    /**
     * Get pose for a starting position
     * 
     * @param position Starting position enum
     * @return RobotPose2D with position, heading, and initial covariance
     */
    public static RobotPose2D getPose(StartPosition position) {
        return getPose(position, new LocalizationConfig());
    }
    
    /**
     * Get pose for a starting position with custom config
     * 
     * @param position Starting position enum
     * @param config Configuration for initial uncertainty
     * @return RobotPose2D with position, heading, and initial covariance
     */
    public static RobotPose2D getPose(StartPosition position, LocalizationConfig config) {
        double x, y, heading;
        
        // Compute positions based on field geometry
        // Note: These are example coordinates - adjust for actual field measurements
        double halfField = FIELD_WIDTH_MM / 2.0;
        
        switch (position) {
            case RED_AUDIENCE_LEFT:
                // Red alliance (positive X), audience side (positive Y), left from driver view
                x = halfField - START_ZONE_OFFSET_X;
                y = halfField - START_ZONE_OFFSET_Y;
                heading = Math.toRadians(180);  // Facing toward blue alliance
                break;
                
            case RED_AUDIENCE_RIGHT:
                // Red alliance (positive X), audience side (positive Y), right from driver view
                x = halfField - START_ZONE_OFFSET_X;
                y = -halfField + START_ZONE_OFFSET_Y;
                heading = Math.toRadians(180);  // Facing toward blue alliance
                break;
                
            case BLUE_AUDIENCE_LEFT:
                // Blue alliance (negative X), audience side (positive Y), left from driver view
                x = -halfField + START_ZONE_OFFSET_X;
                y = halfField - START_ZONE_OFFSET_Y;
                heading = Math.toRadians(0);  // Facing toward red alliance
                break;
                
            case BLUE_AUDIENCE_RIGHT:
                // Blue alliance (negative X), audience side (positive Y), right from driver view
                x = -halfField + START_ZONE_OFFSET_X;
                y = -halfField + START_ZONE_OFFSET_Y;
                heading = Math.toRadians(0);  // Facing toward red alliance
                break;
                
            case RED_NET_ZONE:
                // Red alliance (positive X), net zone side (negative Y center)
                x = halfField - START_ZONE_OFFSET_X;
                y = 0.0;  // Centered on field
                heading = Math.toRadians(180);  // Facing toward blue alliance
                break;
                
            case BLUE_NET_ZONE:
                // Blue alliance (negative X), net zone side (negative Y center)
                x = -halfField + START_ZONE_OFFSET_X;
                y = 0.0;  // Centered on field
                heading = Math.toRadians(0);  // Facing toward red alliance
                break;
                
            default:
                // Fallback to origin
                x = 0;
                y = 0;
                heading = 0;
                break;
        }
        
        // Create pose with initial uncertainty from config
        Matrix3x3 initialCovariance = config.getInitialCovarianceMatrix();
        RobotPose2D pose = new RobotPose2D(x, y, heading, initialCovariance);
        pose.timestamp = System.currentTimeMillis();
        
        return pose;
    }
    
    /**
     * Get descriptive name for a starting position
     */
    public static String getName(StartPosition position) {
        switch (position) {
            case RED_AUDIENCE_LEFT:
                return "Red Audience Left";
            case RED_AUDIENCE_RIGHT:
                return "Red Audience Right";
            case BLUE_AUDIENCE_LEFT:
                return "Blue Audience Left";
            case BLUE_AUDIENCE_RIGHT:
                return "Blue Audience Right";
            case RED_NET_ZONE:
                return "Red Net Zone";
            case BLUE_NET_ZONE:
                return "Blue Net Zone";
            default:
                return "Unknown";
        }
    }
    
    /**
     * Get all available start positions
     */
    public static StartPosition[] getAllPositions() {
        return StartPosition.values();
    }
    
    /**
     * Helper method for OpModes to select start position via gamepad
     * 
     * Example usage in init loop:
     * ```java
     * int selectedIndex = 0;
     * while (!isStarted()) {
     *     if (gamepad1.dpad_up && !lastDpadUp) {
     *         selectedIndex = (selectedIndex - 1 + 6) % 6;
     *     }
     *     if (gamepad1.dpad_down && !lastDpadDown) {
     *         selectedIndex = (selectedIndex + 1) % 6;
     *     }
     *     
     *     StartPosition selected = PredefinedPoses.selectPosition(selectedIndex);
     *     telemetry.addData("Start Position", PredefinedPoses.getName(selected));
     *     telemetry.addData("Use D-Pad Up/Down to change", "");
     *     telemetry.update();
     *     
     *     lastDpadUp = gamepad1.dpad_up;
     *     lastDpadDown = gamepad1.dpad_down;
     * }
     * ```
     */
    public static StartPosition selectPosition(int index) {
        StartPosition[] positions = getAllPositions();
        if (index < 0 || index >= positions.length) {
            return positions[0];  // Default to first position
        }
        return positions[index];
    }
    
    /**
     * Get field dimensions for reference
     */
    public static double getFieldWidth() {
        return FIELD_WIDTH_MM;
    }
    
    public static double getFieldHeight() {
        return FIELD_HEIGHT_MM;
    }
}

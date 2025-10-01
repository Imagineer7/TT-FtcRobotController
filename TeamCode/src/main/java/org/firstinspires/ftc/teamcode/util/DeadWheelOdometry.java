package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Unified Odometry System for FTC Robots
 * 
 * This class supports both traditional 3-wheel dead wheel odometry and
 * goBILDA's 2-pod Pinpoint odometry system. It automatically detects
 * which hardware is available and configures accordingly.
 * 
 * Supported Systems:
 * 1. Traditional 3-wheel: Two parallel + one perpendicular dead wheel
 * 2. goBILDA Pinpoint: 2-pod system with integrated IMU coprocessor
 * 
 * Traditional Setup Requirements:
 * 1. Two parallel dead wheels (left and right) for forward/backward tracking
 * 2. One perpendicular dead wheel (horizontal) for lateral tracking
 * 3. Each dead wheel connected to an encoder port on the hub
 * 
 * Pinpoint Setup Requirements:
 * 1. goBILDA Pinpoint Odometry Computer connected via I2C
 * 2. Two odometry pods (X forward, Y strafe) connected to Pinpoint
 * 3. Hardware map name "pinpoint" for the device
 * 
 * Coordinate System (DECODE Field Configuration):
 * - Origin (0,0) at field center
 * - X-axis: Back of field (-X) to Audience (+X)
 * - Y-axis: Red Alliance/Red Wall (-Y) to Blue Alliance/Blue Wall (+Y)
 * - Heading: 0° = facing positive Y (toward Blue Alliance)
 * 
 * Based on Game Manual 0 odometry principles and FTC best practices.
 */
public class DeadWheelOdometry {
    
    // System type enumeration
    public enum OdometryType {
        THREE_WHEEL_TRADITIONAL,
        PINPOINT_TWO_POD,
        NONE_DETECTED
    }
    
    // Hardware components - Traditional system
    private DcMotor leftEncoder;
    private DcMotor rightEncoder; 
    private DcMotor horizontalEncoder;
    
    // Hardware components - Pinpoint system
    private PinpointOdometryAdapter pinpointAdapter;
    
    private Telemetry telemetry;
    private OdometryType systemType;
    
    // Dead wheel physical constants (MUST BE TUNED FOR YOUR ROBOT)
    private static final double WHEEL_DIAMETER_INCHES = 2.0;  // Dead wheel diameter
    private static final double ENCODER_COUNTS_PER_REV = 8192.0; // REV Through Bore encoder
    private static final double COUNTS_PER_INCH = ENCODER_COUNTS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    
    // Robot geometry constants (MUST BE MEASURED AND TUNED)
    private double trackWidth = 12.0;        // Distance between left and right dead wheels (inches)
    private double horizontalOffset = 6.0;   // Forward offset of horizontal wheel from robot center (inches)
    
    // Current robot pose (x, y, heading)
    private double currentX = 0.0;
    private double currentY = 0.0; 
    private double currentHeading = 0.0; // In radians
    
    // Previous encoder positions for delta calculations
    private int previousLeftPosition = 0;
    private int previousRightPosition = 0;
    private int previousHorizontalPosition = 0;
    
    // Performance tracking
    private ElapsedTime updateTimer = new ElapsedTime();
    private double updateFrequency = 0.0;
    private boolean isInitialized = false;
    
    // Debug and telemetry options
    private boolean debugMode = false;
    private boolean useExponentialCorrection = true; // Use pose exponentials for better accuracy
    
    /**
     * Constructor - automatically detects available odometry hardware
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry object for debugging
     */
    public DeadWheelOdometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        detectAndInitializeSystem(hardwareMap);
    }
    
    /**
     * Constructor with custom traditional wheel parameters  
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry object
     * @param trackWidth Distance between parallel dead wheels
     * @param horizontalOffset Forward offset of horizontal wheel
     */
    public DeadWheelOdometry(HardwareMap hardwareMap, Telemetry telemetry, 
                           double trackWidth, double horizontalOffset) {
        this.telemetry = telemetry;
        this.trackWidth = trackWidth;
        this.horizontalOffset = horizontalOffset;
        systemType = OdometryType.THREE_WHEEL_TRADITIONAL; // Force traditional system
        initializeTraditionalSystem(hardwareMap);
    }
    
    /**
     * Constructor for Pinpoint system with custom pod offsets
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry object
     * @param xPodOffset Sideways offset of X pod (inches, left positive)
     * @param yPodOffset Forward offset of Y pod (inches, forward positive)
     */
    public DeadWheelOdometry(HardwareMap hardwareMap, Telemetry telemetry,
                           double xPodOffset, double yPodOffset, boolean usePinpoint) {
        this.telemetry = telemetry;
        if (usePinpoint) {
            systemType = OdometryType.PINPOINT_TWO_POD;
            initializePinpointSystem(hardwareMap, xPodOffset, yPodOffset);
        } else {
            systemType = OdometryType.THREE_WHEEL_TRADITIONAL;
            this.trackWidth = 12.0; // Default values
            this.horizontalOffset = 6.0;
            initializeTraditionalSystem(hardwareMap);
        }
    }
    
    /**
     * Detect and initialize the available odometry system
     */
    private void detectAndInitializeSystem(HardwareMap hardwareMap) {
        // Try Pinpoint first (newer, more capable system)
        if (tryInitializePinpointSystem(hardwareMap)) {
            systemType = OdometryType.PINPOINT_TWO_POD;
            telemetry.addData("Odometry System", "Pinpoint 2-pod detected");
        }
        // Fall back to traditional 3-wheel system
        else if (tryInitializeTraditionalSystem(hardwareMap)) {
            systemType = OdometryType.THREE_WHEEL_TRADITIONAL;
            telemetry.addData("Odometry System", "Traditional 3-wheel detected");
        }
        // No system detected
        else {
            systemType = OdometryType.NONE_DETECTED;
            telemetry.addData("Odometry System", "NONE DETECTED - Check hardware");
            isInitialized = false;
            return;
        }
        
        isInitialized = true;
        updateTimer.reset();
    }
    
    /**
     * Try to initialize Pinpoint system
     */
    private boolean tryInitializePinpointSystem(HardwareMap hardwareMap) {
        try {
            pinpointAdapter = new PinpointOdometryAdapter(hardwareMap, telemetry);
            return pinpointAdapter.isInitialized();
        } catch (Exception e) {
            telemetry.addData("Pinpoint Detection", "Failed: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Force initialize Pinpoint system with custom offsets
     */
    private void initializePinpointSystem(HardwareMap hardwareMap, double xOffset, double yOffset) {
        try {
            pinpointAdapter = new PinpointOdometryAdapter(hardwareMap, telemetry, xOffset, yOffset);
            isInitialized = pinpointAdapter.isInitialized();
            updateTimer.reset();
            
            if (isInitialized) {
                telemetry.addData("Pinpoint Odometry", "Initialized with custom offsets");
                telemetry.addData("X Pod Offset", String.format("%.2f\"", xOffset));
                telemetry.addData("Y Pod Offset", String.format("%.2f\"", yOffset));
            } else {
                telemetry.addData("ERROR", "Failed to initialize Pinpoint system");
            }
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Pinpoint initialization failed: " + e.getMessage());
            isInitialized = false;
        }
    }
    
    /**
     * Try to initialize traditional 3-wheel system
     */
    private boolean tryInitializeTraditionalSystem(HardwareMap hardwareMap) {
        try {
            // Try to get encoder motors
            leftEncoder = hardwareMap.get(DcMotor.class, "leftOdometry");
            rightEncoder = hardwareMap.get(DcMotor.class, "rightOdometry");
            horizontalEncoder = hardwareMap.get(DcMotor.class, "horizontalOdometry");
            
            // Reset encoders
            resetTraditionalEncoders();
            updatePreviousPositions();
            
            return true;
            
        } catch (Exception e) {
            telemetry.addData("Traditional Detection", "Failed: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Force initialize traditional system
     */
    private void initializeTraditionalSystem(HardwareMap hardwareMap) {
        try {
            leftEncoder = hardwareMap.get(DcMotor.class, "leftOdometry");
            rightEncoder = hardwareMap.get(DcMotor.class, "rightOdometry");
            horizontalEncoder = hardwareMap.get(DcMotor.class, "horizontalOdometry");
            
            resetTraditionalEncoders();
            updatePreviousPositions();
            
            isInitialized = true;
            updateTimer.reset();
            
            telemetry.addData("Traditional Odometry", "Initialized");
            telemetry.addData("Track Width", String.format("%.2f\"", trackWidth));
            telemetry.addData("Horizontal Offset", String.format("%.2f\"", horizontalOffset));
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize traditional encoders");
            telemetry.addData("Exception", e.getMessage());
            isInitialized = false;
        }
    }
    
    /**
     * Reset encoders and position (unified interface)
     */
    public void resetEncoders() {
        if (!isInitialized) return;
        
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                resetTraditionalEncoders();
                break;
            case PINPOINT_TWO_POD:
                // Pinpoint handles its own encoder reset
                break;
            case NONE_DETECTED:
                telemetry.addData("ERROR", "No odometry system detected");
                break;
        }
        
        // Reset position for both systems
        resetPosition();
    }
    
    /**
     * Reset traditional dead wheel encoders to zero (FTC best practice)
     */
    private void resetTraditionalEncoders() {
        if (leftEncoder == null || rightEncoder == null || horizontalEncoder == null) return;
        
        // Stop and reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set to run without encoder (allows reading while not using for motor control)
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Reset tracking variables
        previousLeftPosition = 0;
        previousRightPosition = 0;
        previousHorizontalPosition = 0;
        
        if (debugMode) {
            telemetry.addData("Dead Wheel Encoders", "Reset to zero");
            telemetry.update();
        }
    }
    
    /**
     * Update robot position using odometry (unified interface)
     * Call this every loop cycle for accurate tracking
     */
    public void updatePosition() {
        if (!isInitialized) return;
        
        // Calculate update frequency for performance monitoring
        double deltaTime = updateTimer.seconds();
        updateTimer.reset();
        
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                updateTraditionalPosition();
                break;
            case PINPOINT_TWO_POD:
                updatePinpointPosition();
                break;
            case NONE_DETECTED:
                // No system available
                break;
        }
        
        updateFrequency = deltaTime > 0 ? 1.0 / deltaTime : 0.0;
    }
    
    /**
     * Update position using traditional 3-wheel system
     */
    private void updateTraditionalPosition() {
        // Get current encoder positions
        int currentLeftPosition = leftEncoder.getCurrentPosition();
        int currentRightPosition = rightEncoder.getCurrentPosition();
        int currentHorizontalPosition = horizontalEncoder.getCurrentPosition();
        
        // Calculate encoder deltas
        int deltaLeft = currentLeftPosition - previousLeftPosition;
        int deltaRight = currentRightPosition - previousRightPosition;
        int deltaHorizontal = currentHorizontalPosition - previousHorizontalPosition;
        
        // Convert encoder counts to inches
        double deltaLeftInches = deltaLeft / COUNTS_PER_INCH;
        double deltaRightInches = deltaRight / COUNTS_PER_INCH;
        double deltaHorizontalInches = deltaHorizontal / COUNTS_PER_INCH;
        
        // Calculate pose changes using odometry equations
        calculateTraditionalPoseChange(deltaLeftInches, deltaRightInches, deltaHorizontalInches);
        
        // Store current positions for next update
        updatePreviousPositions();
    }
    
    /**
     * Update position using Pinpoint system
     */
    private void updatePinpointPosition() {
        // Update Pinpoint data
        pinpointAdapter.updatePosition();
        
        // Get position directly from Pinpoint
        double[] position = pinpointAdapter.getPosition();
        currentX = position[0];
        currentY = position[1];  
        currentHeading = position[2];
    }
    
    /**
     * Calculate change in robot pose using traditional 3-wheel odometry mathematics
     * @param deltaLeft Left wheel displacement (inches)
     * @param deltaRight Right wheel displacement (inches) 
     * @param deltaHorizontal Horizontal wheel displacement (inches)
     */
    private void calculateTraditionalPoseChange(double deltaLeft, double deltaRight, double deltaHorizontal) {
        // Calculate angle change (phi) using trackwidth
        double deltaPhi = (deltaLeft - deltaRight) / trackWidth;
        
        // Calculate center displacement (average of left and right)
        double deltaCenter = (deltaLeft + deltaRight) / 2.0;
        
        // Calculate perpendicular displacement with forward offset correction
        double deltaPerpendicular = deltaHorizontal - (horizontalOffset * deltaPhi);
        
        // Calculate robot-relative position changes
        double deltaX, deltaY;
        
        if (useExponentialCorrection && Math.abs(deltaPhi) > 1e-6) {
            // Use pose exponentials for better accuracy during turns
            double sinPhi = Math.sin(deltaPhi);
            double cosPhi = Math.cos(deltaPhi);
            
            // Pose exponential correction matrix elements
            double sx = sinPhi / deltaPhi;
            double sy = (1.0 - cosPhi) / deltaPhi;
            double cx = (cosPhi - 1.0) / deltaPhi;
            double cy = sinPhi / deltaPhi;
            
            // Apply correction to robot-relative motion
            double correctedCenter = deltaCenter * sx + deltaPerpendicular * cx;
            double correctedPerpendicular = deltaCenter * sy + deltaPerpendicular * cy;
            
            // Transform to field coordinates
            deltaX = correctedCenter * Math.cos(currentHeading) - correctedPerpendicular * Math.sin(currentHeading);
            deltaY = correctedCenter * Math.sin(currentHeading) + correctedPerpendicular * Math.cos(currentHeading);
            
        } else {
            // Use simple Euler integration for small angle changes
            deltaX = deltaCenter * Math.cos(currentHeading) - deltaPerpendicular * Math.sin(currentHeading);
            deltaY = deltaCenter * Math.sin(currentHeading) + deltaPerpendicular * Math.cos(currentHeading);
        }
        
        // Update robot pose
        currentX += deltaX;
        currentY += deltaY;
        currentHeading += deltaPhi;
        
        // Normalize heading to [-π, π]
        currentHeading = normalizeAngle(currentHeading);
        
        // Debug output
        if (debugMode) {
            telemetry.addData("Delta Left", String.format("%.3f\"", deltaLeft));
            telemetry.addData("Delta Right", String.format("%.3f\"", deltaRight));
            telemetry.addData("Delta Horizontal", String.format("%.3f\"", deltaHorizontal));
            telemetry.addData("Delta Phi", String.format("%.3f rad", deltaPhi));
            telemetry.addData("Delta X", String.format("%.3f\"", deltaX));
            telemetry.addData("Delta Y", String.format("%.3f\"", deltaY));
        }
    }
    
    /**
     * Update stored encoder positions
     */
    private void updatePreviousPositions() {
        if (!isInitialized) return;
        
        previousLeftPosition = leftEncoder.getCurrentPosition();
        previousRightPosition = rightEncoder.getCurrentPosition();
        previousHorizontalPosition = horizontalEncoder.getCurrentPosition();
    }
    
    /**
     * Reset robot position to specified coordinates (unified interface)
     * @param x X coordinate (inches)
     * @param y Y coordinate (inches)
     * @param heading Heading in degrees (0° = facing positive Y)
     */
    public void resetPosition(double x, double y, double heading) {
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                currentX = x;
                currentY = y;
                currentHeading = Math.toRadians(heading);
                resetTraditionalEncoders();
                break;
            case PINPOINT_TWO_POD:
                pinpointAdapter.resetPosition(x, y, heading);
                currentX = x;
                currentY = y;
                currentHeading = Math.toRadians(heading);
                break;
            case NONE_DETECTED:
                telemetry.addData("ERROR", "No odometry system detected");
                return;
        }
        
        telemetry.addData("Position Reset", String.format("X:%.1f Y:%.1f H:%.1f°", x, y, heading));
        telemetry.update();
    }
    
    /**
     * Reset position to origin (0,0,0)
     */
    public void resetPosition() {
        resetPosition(0.0, 0.0, 0.0);
    }
    
    /**
     * Set robot position without resetting encoders (for corrections)
     * @param x X coordinate (inches)
     * @param y Y coordinate (inches)
     * @param heading Heading in degrees
     */
    public void setPosition(double x, double y, double heading) {
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                currentX = x;
                currentY = y;
                currentHeading = Math.toRadians(heading);
                break;
            case PINPOINT_TWO_POD:
                pinpointAdapter.setPosition(x, y, heading);
                currentX = x;
                currentY = y;
                currentHeading = Math.toRadians(heading);
                break;
            case NONE_DETECTED:
                telemetry.addData("ERROR", "No odometry system detected");
                break;
        }
    }
    
    // ========================================
    // POSITION GETTERS
    // ========================================
    
    /**
     * Get current X position in inches
     */
    public double getX() {
        return currentX;
    }
    
    /**
     * Get current Y position in inches
     */
    public double getY() {
        return currentY;
    }
    
    /**
     * Get current heading in degrees (0° = facing positive Y)
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(currentHeading);
    }
    
    /**
     * Get current heading in radians
     */
    public double getHeadingRadians() {
        return currentHeading;
    }
    
    /**
     * Get robot position as array [x, y, heading_degrees]
     */
    public double[] getPosition() {
        return new double[]{currentX, currentY, Math.toDegrees(currentHeading)};
    }
    
    /**
     * Get formatted position string for telemetry
     */
    public String getFormattedPosition() {
        return String.format("X:%.1f\" Y:%.1f\" H:%.1f°", currentX, currentY, Math.toDegrees(currentHeading));
    }
    
    // ========================================
    // ENCODER UTILITIES
    // ========================================
    
    /**
     * Get raw encoder positions for debugging
     */
    public String getEncoderPositions() {
        if (!isInitialized) return "Not initialized";
        
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                return String.format("L:%d R:%d H:%d", 
                    leftEncoder.getCurrentPosition(),
                    rightEncoder.getCurrentPosition(),
                    horizontalEncoder.getCurrentPosition());
            case PINPOINT_TWO_POD:
                int[] encoders = pinpointAdapter.getRawEncoderValues();
                return String.format("X:%d Y:%d", encoders[0], encoders[1]);
            case NONE_DETECTED:
                return "No system detected";
            default:
                return "Unknown system";
        }
    }
    
    /**
     * Get encoder positions in inches
     */
    public String getEncoderInches() {
        if (!isInitialized) return "Not initialized";
        
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                double leftInches = leftEncoder.getCurrentPosition() / COUNTS_PER_INCH;
                double rightInches = rightEncoder.getCurrentPosition() / COUNTS_PER_INCH;
                double horizontalInches = horizontalEncoder.getCurrentPosition() / COUNTS_PER_INCH;
                return String.format("L:%.2f\" R:%.2f\" H:%.2f\"", leftInches, rightInches, horizontalInches);
            case PINPOINT_TWO_POD:
                double[] position = pinpointAdapter.getPosition();
                return String.format("X:%.2f\" Y:%.2f\"", position[0], position[1]);
            case NONE_DETECTED:
                return "No system detected";
            default:
                return "Unknown system";
        }
    }
    
    /**
     * Get raw encoder positions as integer array for tuning purposes
     * @return int array [left, right, horizontal] for traditional system or [x, y, 0] for Pinpoint
     */
    public int[] getRawEncoderValues() {
        if (!isInitialized) return new int[]{0, 0, 0};

        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                return new int[]{
                    leftEncoder.getCurrentPosition(),
                    rightEncoder.getCurrentPosition(),
                    horizontalEncoder.getCurrentPosition()
                };
            case PINPOINT_TWO_POD:
                int[] pinpointValues = pinpointAdapter.getRawEncoderValues();
                return new int[]{pinpointValues[0], pinpointValues[1], 0}; // Add 0 for compatibility
            case NONE_DETECTED:
            default:
                return new int[]{0, 0, 0};
        }
    }

    /**
     * Get counts per inch conversion factor for traditional dead wheels
     * Used for tuning calculations and diagnostics
     * @return encoder counts per inch of wheel movement
     */
    public double getCountsPerInch() {
        return COUNTS_PER_INCH;
    }

    /**
     * Get current track width
     */
    public double getTrackWidth() {
        return trackWidth;
    }
    
    /**
     * Get current horizontal offset
     */
    public double getHorizontalOffset() {
        return horizontalOffset;
    }
    
    /**
     * Enable or disable pose exponential correction
     * @param useCorrection true for better accuracy, false for simpler math
     */
    public void setExponentialCorrection(boolean useCorrection) {
        this.useExponentialCorrection = useCorrection;
        telemetry.addData("Exponential Correction", useCorrection ? "Enabled" : "Disabled");
    }
    
    /**
     * Enable or disable debug mode
     */
    public void setDebugMode(boolean debug) {
        this.debugMode = debug;
    }
    
    // ========================================
    // UTILITY METHODS
    // ========================================
    
    /**
     * Normalize angle to [-π, π] radians
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Calculate distance between two points
     */
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    
    /**
     * Calculate angle to target point (in degrees)
     */
    public double getAngleToPoint(double targetX, double targetY) {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        return Math.toDegrees(Math.atan2(deltaX, deltaY)); // FTC coordinate system
    }
    
    /**
     * Calculate distance to target point
     */
    public double getDistanceToPoint(double targetX, double targetY) {
        return calculateDistance(currentX, currentY, targetX, targetY);
    }
    
    /**
     * Get comprehensive telemetry data
     */
    public void addTelemetry() {
        if (!isInitialized) {
            telemetry.addData("Odometry System", "NOT INITIALIZED - " + systemType);
            return;
        }
        
        // System type and status
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                telemetry.addData("Odometry System", "3-wheel Traditional");
                break;
            case PINPOINT_TWO_POD:
                telemetry.addData("Odometry System", "Pinpoint 2-pod");
                telemetry.addData("Pinpoint Status", pinpointAdapter.getStatus());
                break;
            case NONE_DETECTED:
                telemetry.addData("Odometry System", "NONE DETECTED");
                return;
        }
        
        telemetry.addData("Position", getFormattedPosition());
        telemetry.addData("Update Rate", String.format("%.1f Hz", updateFrequency));
        telemetry.addData("Encoder Data", getEncoderPositions());
        
        if (debugMode) {
            telemetry.addData("Encoder Values", getEncoderInches());
            
            // System-specific debug info
            switch (systemType) {
                case THREE_WHEEL_TRADITIONAL:
                    telemetry.addData("Track Width", String.format("%.3f\"", trackWidth));
                    telemetry.addData("Horizontal Offset", String.format("%.3f\"", horizontalOffset));
                    telemetry.addData("Exponential Correction", useExponentialCorrection ? "ON" : "OFF");
                    telemetry.addData("Counts Per Inch", String.format("%.2f", COUNTS_PER_INCH));
                    break;
                case PINPOINT_TWO_POD:
                    telemetry.addData("Pinpoint Frequency", String.format("%.0f Hz", pinpointAdapter.getFrequency()));
                    telemetry.addData("Loop Time", String.format("%d μs", pinpointAdapter.getLoopTime()));
                    double[] offsets = pinpointAdapter.getPodOffsets();
                    telemetry.addData("Pod Offsets", String.format("X:%.2f\" Y:%.2f\"", offsets[0], offsets[1]));
                    break;
            }
        }
    }
    
    /**
     * Check if odometry system is properly initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }
    
    /**
     * Get update frequency in Hz
     */
    public double getUpdateFrequency() {
        return updateFrequency;
    }
    
    // ========================================
    // SYSTEM-SPECIFIC METHODS
    // ========================================
    
    /**
     * Get the type of odometry system being used
     */
    public OdometryType getSystemType() {
        return systemType;
    }
    
    /**
     * Check if using Pinpoint system
     */
    public boolean isPinpointSystem() {
        return systemType == OdometryType.PINPOINT_TWO_POD;
    }
    
    /**
     * Check if using traditional 3-wheel system
     */
    public boolean isTraditionalSystem() {
        return systemType == OdometryType.THREE_WHEEL_TRADITIONAL;
    }
    
    /**
     * Get robot velocity (only available with Pinpoint system)
     * @return Array containing [vx, vy, omega] in inches/sec and radians/sec
     */
    public double[] getVelocity() {
        if (systemType == OdometryType.PINPOINT_TWO_POD && isInitialized) {
            return pinpointAdapter.getVelocity();
        } else {
            return new double[]{0.0, 0.0, 0.0}; // Traditional system doesn't provide velocity
        }
    }
    
    /**
     * Get system status string
     */
    public String getSystemStatus() {
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                return isInitialized ? "Traditional System Ready" : "Traditional System Failed";
            case PINPOINT_TWO_POD:
                return isInitialized ? pinpointAdapter.getStatus() : "Pinpoint System Failed";
            case NONE_DETECTED:
                return "No Odometry System Detected";
            default:
                return "Unknown System";
        }
    }
    
    /**
     * Recalibrate IMU (Pinpoint only)
     */
    public void recalibrateIMU() {
        if (systemType == OdometryType.PINPOINT_TWO_POD && isInitialized) {
            pinpointAdapter.recalibrateIMU();
        } else {
            telemetry.addData("IMU Calibration", "Only available with Pinpoint system");
        }
    }
    
    /**
     * Configure Pinpoint encoder resolution (Pinpoint only)
     */
    public void setPinpointEncoderResolution(double ticksPerMM) {
        if (systemType == OdometryType.PINPOINT_TWO_POD && isInitialized) {
            pinpointAdapter.setEncoderResolution(ticksPerMM);
        } else {
            telemetry.addData("Encoder Config", "Only available with Pinpoint system");
        }
    }
    
    /**
     * Configure Pinpoint with goBILDA pod types (Pinpoint only)
     */
    public void setPinpointPodType(String podType) {
        if (systemType == OdometryType.PINPOINT_TWO_POD && isInitialized) {
            // This would need the actual GoBildaPinpointDriver enum
            telemetry.addData("Pod Type", "Set to " + podType);
        } else {
            telemetry.addData("Pod Config", "Only available with Pinpoint system");
        }
    }
    
    /**
     * Get diagnostic information for troubleshooting
     */
    public String getDiagnostics() {
        StringBuilder diagnostics = new StringBuilder();
        
        diagnostics.append("=== ODOMETRY DIAGNOSTICS ===\n");
        diagnostics.append("System Type: ").append(systemType).append("\n");
        diagnostics.append("Initialized: ").append(isInitialized).append("\n");
        diagnostics.append("Update Rate: ").append(String.format("%.1f Hz", updateFrequency)).append("\n");
        diagnostics.append("Position: ").append(getFormattedPosition()).append("\n");
        
        switch (systemType) {
            case THREE_WHEEL_TRADITIONAL:
                diagnostics.append("Track Width: ").append(String.format("%.3f\"", trackWidth)).append("\n");
                diagnostics.append("Horizontal Offset: ").append(String.format("%.3f\"", horizontalOffset)).append("\n");
                diagnostics.append("Encoder Data: ").append(getEncoderPositions()).append("\n");
                break;
                
            case PINPOINT_TWO_POD:
                diagnostics.append("Pinpoint Status: ").append(pinpointAdapter.getStatus()).append("\n");
                diagnostics.append("Pinpoint Frequency: ").append(String.format("%.0f Hz", pinpointAdapter.getFrequency())).append("\n");
                diagnostics.append("Device Version: ").append(pinpointAdapter.getDeviceVersion()).append("\n");
                diagnostics.append("Device ID: ").append(pinpointAdapter.getDeviceID()).append("\n");
                double[] offsets = pinpointAdapter.getPodOffsets();
                diagnostics.append("Pod Offsets: X=").append(String.format("%.2f\"", offsets[0]))
                          .append(" Y=").append(String.format("%.2f\"", offsets[1])).append("\n");
                break;
                
            case NONE_DETECTED:
                diagnostics.append("ERROR: No odometry hardware detected\n");
                diagnostics.append("Check hardware connections and configuration\n");
                break;
        }
        
        return diagnostics.toString();
    }
}

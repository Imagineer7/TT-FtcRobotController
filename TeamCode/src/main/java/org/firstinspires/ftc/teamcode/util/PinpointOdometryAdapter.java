package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * PinpointOdometryAdapter - Adapter for goBILDA Pinpoint Odometry Computer
 * 
 * This class adapts the goBILDA Pinpoint 2-pod odometry system to work with our
 * existing DeadWheelOdometry interface. The Pinpoint is a coprocessor that handles
 * two dead wheel pods and an internal IMU to provide high-accuracy positioning.
 * 
 * Hardware Features:
 * - 2 odometry pods (X forward, Y strafe) 
 * - Internal IMU for heading
 * - 1500Hz update frequency
 * - I2C communication at 400kHz
 * - Automatic sensor fusion
 * - Built-in calibration
 * 
 * Coordinate System (DECODE Field Configuration):
 * - X-axis: Back of field (-X) to Audience (+X)
 * - Y-axis: Red Wall/Red Alliance (-Y) to Blue Wall/Blue Alliance (+Y)
 * - Heading: 0° = facing positive Y (toward Blue Alliance)
 * 
 * @author FTC Team
 * @version 1.0
 */
public class PinpointOdometryAdapter {
    
    // Hardware components
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;
    
    // Configuration parameters
    private double xPodOffset;  // Sideways offset of X pod from tracking point (mm)
    private double yPodOffset;  // Forward offset of Y pod from tracking point (mm)
    
    // Conversion constants
    private static final double MM_TO_INCHES = 1.0 / 25.4;
    private static final double INCHES_TO_MM = 25.4;
    
    // Status tracking
    private boolean initialized = false;
    private String lastError = "";
    
    /**
     * Constructor with default offsets
     */
    public PinpointOdometryAdapter(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 0.0, 0.0); // Default to robot center tracking
    }
    
    /**
     * Constructor with custom pod offsets
     * @param hardwareMap FTC hardware map
     * @param telemetry FTC telemetry
     * @param xPodOffsetInches Sideways offset of X pod (inches, left is positive)
     * @param yPodOffsetInches Forward offset of Y pod (inches, forward is positive)
     */
    public PinpointOdometryAdapter(HardwareMap hardwareMap, Telemetry telemetry, 
                                 double xPodOffsetInches, double yPodOffsetInches) {
        this.telemetry = telemetry;
        this.xPodOffset = xPodOffsetInches * INCHES_TO_MM;
        this.yPodOffset = yPodOffsetInches * INCHES_TO_MM;
        
        try {
            // Initialize Pinpoint driver
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            
            // Configure the Pinpoint
            configurePinpoint();
            
            initialized = true;
            telemetry.addData("Pinpoint Odometry", "Initialized successfully");
            
        } catch (Exception e) {
            lastError = "Failed to initialize Pinpoint: " + e.getMessage();
            telemetry.addData("Pinpoint Error", lastError);
            initialized = false;
        }
    }
    
    /**
     * Configure the Pinpoint with robot-specific parameters
     */
    private void configurePinpoint() {
        // Set pod offsets (convert inches to mm)
        pinpoint.setOffsets(xPodOffset, yPodOffset, DistanceUnit.INCH);
        
        // Set encoder resolution for goBILDA 4-bar pods (most common)
        // Users can change this by calling setEncoderResolution() after initialization
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        // Set encoder directions (can be adjusted if needed)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,  // X encoder
            GoBildaPinpointDriver.EncoderDirection.FORWARD   // Y encoder
        );
        
        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();
        
        // Wait for calibration to complete
        try {
            Thread.sleep(300); // 0.25 seconds + buffer
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    /**
     * Update position data from Pinpoint
     */
    public void updatePosition() {
        if (!initialized) return;
        
        try {
            // Update data from Pinpoint
            pinpoint.update();
        } catch (Exception e) {
            lastError = "Update failed: " + e.getMessage();
        }
    }
    
    /**
     * Get current robot position
     * @return Array containing [x, y, heading] in inches and radians
     */
    public double[] getPosition() {
        if (!initialized) return new double[]{0, 0, 0};
        
        try {
            Pose2D pose = pinpoint.getPosition();
            
            return new double[]{
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH), 
                pose.getHeading(AngleUnit.RADIANS)
            };
        } catch (Exception e) {
            lastError = "Position read failed: " + e.getMessage();
            return new double[]{0, 0, 0};
        }
    }
    
    /**
     * Get current robot velocity
     * @return Array containing [vx, vy, omega] in inches/sec and radians/sec
     */
    public double[] getVelocity() {
        if (!initialized) return new double[]{0, 0, 0};
        
        try {
            // Pinpoint driver may not have velocity method, return zero velocity
            // This is a placeholder - actual velocity calculation would require 
            // tracking position changes over time
            return new double[]{0, 0, 0};
        } catch (Exception e) {
            lastError = "Velocity read failed: " + e.getMessage();
            return new double[]{0, 0, 0};
        }
    }
    
    /**
     * Reset robot position to specified coordinates
     * @param x X position in inches
     * @param y Y position in inches  
     * @param heading Heading in degrees
     */
    public void resetPosition(double x, double y, double heading) {
        if (!initialized) return;
        
        try {
            // Create pose in mm and radians (Pinpoint's native units)
            Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, 
                                       AngleUnit.DEGREES, heading);
            
            // Set the new position
            pinpoint.setPosition(newPose);
            
            // Recalibrate IMU if needed
            pinpoint.recalibrateIMU();
            
        } catch (Exception e) {
            lastError = "Reset position failed: " + e.getMessage();
        }
    }
    
    /**
     * Set position without IMU recalibration
     * @param x X position in inches
     * @param y Y position in inches
     * @param heading Heading in degrees
     */
    public void setPosition(double x, double y, double heading) {
        if (!initialized) return;
        
        try {
            Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, 
                                       AngleUnit.DEGREES, heading);
            pinpoint.setPosition(newPose);
        } catch (Exception e) {
            lastError = "Set position failed: " + e.getMessage();
        }
    }
    
    /**
     * Get raw encoder values
     * @return Array containing [xEncoder, yEncoder, 0] (0 for compatibility)
     */
    public int[] getRawEncoderValues() {
        if (!initialized) return new int[]{0, 0, 0};
        
        try {
            return new int[]{
                pinpoint.getEncoderX(),
                pinpoint.getEncoderY(), 
                0 // No horizontal encoder in 2-pod system
            };
        } catch (Exception e) {
            lastError = "Encoder read failed: " + e.getMessage();
            return new int[]{0, 0, 0};
        }
    }
    
    /**
     * Check if system is initialized and ready
     */
    public boolean isInitialized() {
        if (!initialized) return false;
        
        try {
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            return status == GoBildaPinpointDriver.DeviceStatus.READY;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Get system status string
     */
    public String getStatus() {
        if (!initialized) return "Not Initialized: " + lastError;
        
        try {
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            double frequency = pinpoint.getFrequency();
            
            String statusString = "";
            switch (status) {
                case READY:
                    statusString = String.format("Ready (%.0f Hz)", frequency);
                    break;
                case CALIBRATING:
                    statusString = "Calibrating IMU";
                    break;
                case NOT_READY:
                    statusString = "Starting Up";
                    break;
                case FAULT_NO_PODS_DETECTED:
                    statusString = "ERROR: No pods detected";
                    break;
                case FAULT_X_POD_NOT_DETECTED:
                    statusString = "ERROR: X pod not detected";
                    break;
                case FAULT_Y_POD_NOT_DETECTED:
                    statusString = "ERROR: Y pod not detected";
                    break;
                default:
                    statusString = "Unknown status";
            }
            
            return statusString;
            
        } catch (Exception e) {
            return "Status read failed: " + e.getMessage();
        }
    }
    
    /**
     * Configure encoder resolution for goBILDA pods
     */
    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods podType) {
        if (!initialized) return;
        
        try {
            pinpoint.setEncoderResolution(podType);
        } catch (Exception e) {
            lastError = "Encoder resolution setup failed: " + e.getMessage();
        }
    }
    
    /**
     * Configure encoder resolution with custom ticks per mm
     * Note: Pinpoint driver only supports predefined pod types, not custom tick values
     */
    public void setEncoderResolution(double ticksPerMM) {
        if (!initialized) return;
        
        // Convert common tick values to appropriate pod types
        // This is a best-effort mapping since Pinpoint only accepts enum values
        GoBildaPinpointDriver.GoBildaOdometryPods podType;
        if (ticksPerMM > 19.0) {
            podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        } else {
            podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        }
        
        try {
            pinpoint.setEncoderResolution(podType);
        } catch (Exception e) {
            lastError = "Custom encoder resolution setup failed: " + e.getMessage();
        }
    }
    
    /**
     * Set encoder directions
     */
    public void setEncoderDirections(boolean xForward, boolean yForward) {
        if (!initialized) return;
        
        try {
            pinpoint.setEncoderDirections(
                xForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD : 
                          GoBildaPinpointDriver.EncoderDirection.REVERSED,
                yForward ? GoBildaPinpointDriver.EncoderDirection.FORWARD : 
                          GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
        } catch (Exception e) {
            lastError = "Encoder direction setup failed: " + e.getMessage();
        }
    }
    
    /**
     * Get pod offsets for debugging
     * @return Array containing [xOffset, yOffset] in inches
     */
    public double[] getPodOffsets() {
        return new double[]{
            xPodOffset * MM_TO_INCHES,
            yPodOffset * MM_TO_INCHES
        };
    }
    
    /**
     * Get loop frequency for diagnostics
     */
    public double getFrequency() {
        if (!initialized) return 0.0;
        
        try {
            return pinpoint.getFrequency();
        } catch (Exception e) {
            return 0.0;
        }
    }
    
    /**
     * Get loop time for diagnostics (in microseconds)
     */
    public int getLoopTime() {
        if (!initialized) return 0;
        
        try {
            return pinpoint.getLoopTime();
        } catch (Exception e) {
            return 0;
        }
    }
    
    /**
     * Recalibrate IMU only (without position reset)
     */
    public void recalibrateIMU() {
        if (!initialized) return;
        
        try {
            pinpoint.recalibrateIMU();
        } catch (Exception e) {
            lastError = "IMU recalibration failed: " + e.getMessage();
        }
    }
    
    /**
     * Get device version for diagnostics
     */
    public int getDeviceVersion() {
        if (!initialized) return 0;
        
        try {
            return pinpoint.getDeviceVersion();
        } catch (Exception e) {
            return 0;
        }
    }
    
    /**
     * Get device ID for diagnostics (should be 2 for Pinpoint)
     */
    public int getDeviceID() {
        if (!initialized) return 0;
        
        try {
            return pinpoint.getDeviceID();
        } catch (Exception e) {
            return 0;
        }
    }
}
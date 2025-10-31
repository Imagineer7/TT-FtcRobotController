package org.firstinspires.ftc.teamcode.util.aurora.lightning;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;
public class OdoHelper {

    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;
    private boolean isInitialized = false;

    // Default configuration values - Updated for user's specific pod placement
    private static final double DEFAULT_X_OFFSET = -154.0; // mm - X pod is 154mm to the RIGHT of center (negative)
    private static final double DEFAULT_Y_OFFSET = 0;  // mm - Y pod is 154mm to the LEFT of center (positive)
    private static final GoBildaPinpointDriver.GoBildaOdometryPods DEFAULT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // Current position and velocity storage
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentHeading = 0.0;

    private final double[] velocity = new double[3]; // [vx, vy, omega]

    /**
     * Initialize the Dead Wheel Odometry system
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     */
    public void DeadWheelOdometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            telemetry.addData("DeadWheelOdometry", "Initializing...");
            // Get the Pinpoint device from hardware map
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

            // Configure the Pinpoint with default settings
            configureDefaultPinpoint();

            isInitialized = true;
            telemetry.addData("DeadWheelOdometry", "Initialized successfully with GoBilda Pinpoint");

        } catch (Exception e) {
            telemetry.addData("DeadWheelOdometry", "Failed to initialize: " + e.getMessage());
            throw new RuntimeException("Failed to initialize DeadWheelOdometry", e);
        }
    }

    private void configureDefaultPinpoint() {
        configurePinpoint(DEFAULT_X_OFFSET, DEFAULT_Y_OFFSET, DEFAULT_POD_TYPE);
    }

    /**
     * Configure the Pinpoint with custom settings
     */
    private void configurePinpoint(double xOffset, double yOffset,
                                   GoBildaPinpointDriver.GoBildaOdometryPods podType) {
        if (pinpoint == null) return;

        // Set pod offsets relative to the robot's center
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        // Set the type of odometry pods being used
        pinpoint.setEncoderResolution(podType);

        // Set encoder directions (assuming standard configuration)
        // X encoder should increase when moving forward
        // Y encoder should increase when moving left (strafe)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();

        telemetry.addData("Pinpoint Config", "X:%.1f Y:%.1f Pod:%s",
                xOffset, yOffset, podType.name());
    }

    /**
     * Update the odometry readings - call this every loop
     */
    public void updatePosition() {
        if (!isInitialized || pinpoint == null) return;

        try {
            // Update all sensor readings
            pinpoint.update();

            // Cache the current position values (convert mm to inches)
            currentX = pinpoint.getPosX(DistanceUnit.INCH);
            currentY = pinpoint.getPosY(DistanceUnit.INCH);
            currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

            // Cache velocity values (convert mm/s to in/s, rad/s to deg/s)
            velocity[0] = pinpoint.getVelX(DistanceUnit.INCH); // X velocity in in/s
            velocity[1] = pinpoint.getVelY(DistanceUnit.INCH); // Y velocity in in/s
            velocity[2] = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES); // Angular velocity in deg/s

        } catch (Exception e) {
            telemetry.addData("DeadWheelOdometry", "Update data error: " + e.getMessage());
        }
    }

    /**
     * Get current X position in inches
     * @return X position
     */
    public double getX() {
        return currentX;
    }

    /**
     * Get current Y position in inches
     * @return Y position
     */
    public double getY() {
        return currentY;
    }

    /**
     * Get current heading in degrees
     * @return Heading in degrees
     */
    public double getHeadingDegrees() {
        return currentHeading;
    }

    /**
     * Get current heading in radians
     * @return Heading in radians
     */
    public double getHeadingRadians() {
        return Math.toRadians(currentHeading);
    }

    /**
     * Get current velocity [vx, vy, omega]
     * @return Array of velocities [x_vel (in/s), y_vel (in/s), angular_vel (deg/s)]
     */
    public double[] getVelocity() {
        return velocity.clone();
    }

    /**
     * Set the robot's position
     * @param x X position in inches
     * @param y Y position in inches
     * @param heading Heading in degrees
     */
    public void setPosition(double x, double y, double heading) {
        if (!isInitialized || pinpoint == null) return;

        try {
            // Convert inches to mm and degrees to radians for Pinpoint
            Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y,
                    AngleUnit.DEGREES, heading);
            pinpoint.setPosition(newPose);

            // Update cached values
            currentX = x;
            currentY = y;
            currentHeading = heading;

            telemetry.addData("Position Set", "(%.1f, %.1f) @ %.1f°", x, y, heading);

        } catch (Exception e) {
            telemetry.addData("DeadWheelOdometry", "Set position error: " + e.getMessage());
        }
    }

    /**
     * Reset the odometry system to origin
     */
    public void reset() {
        if (!isInitialized || pinpoint == null) return;

        try {
            pinpoint.resetPosAndIMU();
            currentX = 0.0;
            currentY = 0.0;
            currentHeading = 0.0;
            velocity[0] = velocity[1] = velocity[2] = 0.0;

            telemetry.addData("DeadWheelOdometry", "Reset to origin");

        } catch (Exception e) {
            telemetry.addData("DeadWheelOdometry", "Reset error: " + e.getMessage());
        }
    }

    /**
     * Recalibrate the IMU (robot must be stationary)
     */
    public void recalibrateIMU() {
        if (!isInitialized || pinpoint == null) return;

        try {
            pinpoint.recalibrateIMU();
            telemetry.addData("DeadWheelOdometry", "IMU recalibrated");
        } catch (Exception e) {
            telemetry.addData("DeadWheelOdometry", "IMU calibration error: " + e.getMessage());
        }
    }

    /**
     * Get the device status
     * @return Device status string
     */
    public String getDeviceStatus() {
        if (!isInitialized || pinpoint == null) return "NOT_INITIALIZED";

        try {
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            return status.name();
        } catch (Exception e) {
            return "ERROR: " + e.getMessage();
        }
    }

    /**
     * Get the update frequency of the Pinpoint
     * @return Frequency in Hz
     */
    public double getFrequency() {
        if (!isInitialized || pinpoint == null) return 0.0;

        try {
            return pinpoint.getFrequency();
        } catch (Exception e) {
            return 0.0;
        }
    }

    /**
     * Get raw encoder values
     * @return Array of [xEncoder, yEncoder] counts
     */
    public int[] getEncoderCounts() {
        if (!isInitialized || pinpoint == null) return new int[]{0, 0};

        try {
            return new int[]{pinpoint.getEncoderX(), pinpoint.getEncoderY()};
        } catch (Exception e) {
            return new int[]{0, 0};
        }
    }

    /**
     * Check if the odometry system is ready
     * @return True if initialized and ready
     */
    public boolean isReady() {
        if (!isInitialized || pinpoint == null) return false;

        try {
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            return status == GoBildaPinpointDriver.DeviceStatus.READY;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Add telemetry data for debugging
     */
    public void addTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("=== PINPOINT ODOMETRY ===", "");
        telemetry.addData("Status", getDeviceStatus());
        telemetry.addData("Position", "(%.2f, %.2f) @ %.1f°", getX(), getY(), getHeadingDegrees());
        telemetry.addData("Velocity", "X:%.1f Y:%.1f Ω:%.1f",
                velocity[0], velocity[1], velocity[2]);
        telemetry.addData("Frequency", "%.1f Hz", getFrequency());

        if (isInitialized && pinpoint != null) {
            try {
                int[] encoders = getEncoderCounts();
                telemetry.addData("Encoders", "X:%d Y:%d", encoders[0], encoders[1]);
            } catch (Exception e) {
                telemetry.addData("Encoders", "Error reading");
            }
        }
    }
}

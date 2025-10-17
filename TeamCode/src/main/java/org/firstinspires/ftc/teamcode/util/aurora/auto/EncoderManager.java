package org.firstinspires.ftc.teamcode.util.aurora.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tool.DeadWheelOdometry;
import org.firstinspires.ftc.teamcode.util.tool.GoBildaPinpointDriver;

// Used to manage GoBilda Pinpoint odometry system for precise movement and positioning in autonomous mode
public class EncoderManager {
    private DeadWheelOdometry odometry;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    /**
     * Initialize the encoder manager with hardware map using default Pinpoint configuration
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     */
    public EncoderManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize the GoBilda Pinpoint odometry system with default settings
        this.odometry = new DeadWheelOdometry(hardwareMap, telemetry);

        telemetry.addData("EncoderManager", "Initialized with GoBilda Pinpoint system");
    }

    /**
     * Initialize with custom Pinpoint configuration
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for debugging
     * @param xOffset X pod offset in mm (left of center is positive)
     * @param yOffset Y pod offset in mm (forward of center is positive)
     * @param podType Type of GoBilda odometry pods
     */
    public EncoderManager(HardwareMap hardwareMap, Telemetry telemetry,
                         double xOffset, double yOffset,
                         GoBildaPinpointDriver.GoBildaOdometryPods podType) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize with custom configuration
        this.odometry = new DeadWheelOdometry(hardwareMap, telemetry, xOffset, yOffset, podType);

        telemetry.addData("EncoderManager", "Initialized with custom Pinpoint config");
    }

    /**
     * Initialize odometry system with starting position
     * @param startX Starting X position in inches
     * @param startY Starting Y position in inches
     * @param startHeading Starting heading in degrees
     */
    public void initialize(double startX, double startY, double startHeading) {
        odometry.setPosition(startX, startY, startHeading);
        telemetry.addData("EncoderManager", "Position initialized to (%.1f, %.1f) @ %.1f°",
                         startX, startY, startHeading);
    }

    /**
     * Update encoder readings and calculate position
     */
    public void update() {
        odometry.updatePosition();
    }

    /**
     * Get current X position
     * @return X position in inches
     */
    public double getX() {
        return odometry.getX();
    }

    /**
     * Get current Y position
     * @return Y position in inches
     */
    public double getY() {
        return odometry.getY();
    }

    /**
     * Get current heading
     * @return Heading in degrees
     */
    public double getHeading() {
        return odometry.getHeadingDegrees();
    }

    /**
     * Correct position using external data (e.g., vision system)
     * @param correctedX Corrected X position in inches
     * @param correctedY Corrected Y position in inches
     * @param correctedHeading Corrected heading in degrees
     */
    public void correctPosition(double correctedX, double correctedY, double correctedHeading) {
        odometry.setPosition(correctedX, correctedY, correctedHeading);
        telemetry.addData("EncoderManager", "Position corrected to: (%.1f, %.1f) @ %.1f°",
                         correctedX, correctedY, correctedHeading);
    }

    /**
     * Get current velocity in X direction
     * @return X velocity in inches per second
     */
    public double getVelocityX() {
        double[] velocity = odometry.getVelocity();
        return velocity[0];
    }

    /**
     * Get current velocity in Y direction
     * @return Y velocity in inches per second
     */
    public double getVelocityY() {
        double[] velocity = odometry.getVelocity();
        return velocity[1];
    }

    /**
     * Get current angular velocity
     * @return Angular velocity in degrees per second
     */
    public double getAngularVelocity() {
        double[] velocity = odometry.getVelocity();
        return velocity[2];
    }

    /**
     * Reset odometry system (resets position and recalibrates IMU)
     * Robot must be stationary for IMU calibration
     */
    public void resetEncoders() {
        odometry.reset();
        telemetry.addData("EncoderManager", "Pinpoint system reset and IMU recalibrated");
    }

    /**
     * Recalibrate IMU only (robot must be stationary)
     */
    public void recalibrateIMU() {
        odometry.recalibrateIMU();
        telemetry.addData("EncoderManager", "IMU recalibrated");
    }

    /**
     * Get encoder counts for debugging
     * @return Array of [xEncoder, yEncoder] counts from Pinpoint
     */
    public int[] getEncoderCounts() {
        return odometry.getEncoderCounts();
    }

    /**
     * Get Pinpoint device status
     * @return Device status string
     */
    public String getDeviceStatus() {
        return odometry.getDeviceStatus();
    }

    /**
     * Get Pinpoint update frequency
     * @return Frequency in Hz
     */
    public double getFrequency() {
        return odometry.getFrequency();
    }

    /**
     * Add encoder telemetry data
     */
    public void addTelemetry() {
        telemetry.addData("=== PINPOINT ENCODER DATA ===", "");

        // Current position
        telemetry.addData("Position", "(%.2f, %.2f) @ %.1f°", getX(), getY(), getHeading());

        // Current velocity
        telemetry.addData("Velocity", "X:%.1f Y:%.1f Ω:%.1f",
                         getVelocityX(), getVelocityY(), getAngularVelocity());

        // Device status and frequency
        telemetry.addData("Status", getDeviceStatus());
        telemetry.addData("Frequency", "%.1f Hz", getFrequency());

        // Raw encoder counts
        int[] counts = getEncoderCounts();
        telemetry.addData("Raw Counts", "X:%d Y:%d", counts[0], counts[1]);

        // System status
        telemetry.addData("System Ready", isReady() ? "YES" : "NO");
    }

    /**
     * Check if odometry system is ready
     * @return True if Pinpoint is initialized and ready
     */
    public boolean isReady() {
        return odometry != null && odometry.isReady();
    }
}

package org.firstinspires.ftc.teamcode.util.aurora;
//This class will be for getting positioning and localization data for the robot.
//It may use encoders, IMU, or other sensors to determine the robot's position on the field.
//It will provide methods to get the robot's current coordinates and orientation.
//No dependencies on other AURORA classes, except for AuroraHardwareConfig if necessary.

//It will use the limelight to get position data when available, otherwise it will rely on odometry.
//It should mainly rely on the odometry for localization, using the limelight to correct drift when targets are visible.
//It will provide methods to reset position and update localization data.
//Additionally, it may include methods for calculating distances to targets or waypoints on the field.

//Odometry will be the primary method for localization, with limelight data used for corrections when targets are visible.
//We are using the Gobilda odometry pods with pinpoint computer for our odometry system.
//Its hardware configuration will be defined in AuroraHardwareConfig.java.
//Also its name in the robot configuration file is "odo".
//Use the GoBildaPinpointDriver class to interface with the pinpoint computer for odometry data.

//The X pod offset is NA
//The Y pod offset is NA
public class Localization {
}

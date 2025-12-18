package org.firstinspires.ftc.teamcode.util.aurora;
//This class will be a helper for interfacing with the Limelight vision system.
//It will provide methods to get target data, calculate angles and distances to targets, and configure the Limelight settings.
//It will not depend on other AURORA classes, except for AuroraHardware Config if necessary.

//Reference the Limelight documentation for specific commands and data retrieval methods. Also reference the FTC SDK for hardware integration if needed.
//Also use the examples provided under package org.firstinspires.ftc.robotcontroller.external.samples in this repository for Limelight integration examples.
//the limelights name in the robot configuration file is "limelight".

//This class should include methods to:
//- Get target visibility status
//- Get/Set target horizontal and vertical offsets
//- Get target area
//- Calculate distance to target based on known target height and camera angle
//- Configure Limelight settings (LED mode, camera mode, pipeline selection)

//Pipeline 3 is for AprilTags positioning.
//Pipeline 2 is for Reading the obelisk apriltags to determine the motif pattern. id 21 = "GPP", id 22 = "PGP", id 23 = "PPG".

//When getting position data from the limelight, take multiple readings and average them to reduce noise. Also discard any readings that are over a threshold
//compared to the other readings to filter out outliers. So you take 10 readings, see which ones are close to each other, group them by proximity, and average the largest group. Discard the rest.
//The largest group is likely the most accurate representation of the actual position. This is because the lime light can misinterpret the orientation of an apriltag for a few frames, causing spikes in the data.
//The final position used for navigation should be the averaged value from the largest group of similar readings.

//Groups can be determined by setting a threshold distance. If a reading is within that threshold of another reading, it belongs to the same group.
public class LimelightVisionHelper {
}

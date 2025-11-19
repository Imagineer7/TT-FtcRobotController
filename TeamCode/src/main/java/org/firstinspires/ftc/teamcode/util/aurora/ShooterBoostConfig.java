/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Persistent storage for learned shooter boost parameters
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import android.os.Environment;
import java.io.File;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/**
 * ShooterBoostConfig - Save and load learned boost parameters
 *
 * Stores adaptive learning results to a file on the robot controller
 * so they persist across program restarts
 */
public class ShooterBoostConfig {

    private static final String CONFIG_DIR = Environment.getExternalStorageDirectory().getPath() + "/FIRST/";
    private static final String CONFIG_FILE = "shooter_boost_config.txt";

    /**
     * Save all learned parameters to file
     * @param delay Boost delay in seconds
     * @param duration Boost duration in seconds
     * @param power Boost power (0.0 to 1.0)
     * @param shotInterval Learned shot interval in seconds
     * @param rpmTolerance Learned RPM tolerance
     * @param shotsAnalyzed Number of shots used for learning
     * @return true if save successful, false otherwise
     */
    public static boolean saveAllLearnedParameters(double delay, double duration, double power,
                                                   double shotInterval, double rpmTolerance,
                                                   int shotsAnalyzed) {
        try {
            // Create directory if it doesn't exist
            File dir = new File(CONFIG_DIR);
            if (!dir.exists()) {
                dir.mkdirs();
            }

            // Write parameters to file
            File file = new File(CONFIG_DIR + CONFIG_FILE);
            FileWriter writer = new FileWriter(file);

            writer.write("# Shooter Learning Configuration - Auto-generated\n");
            writer.write("# Last updated: " + System.currentTimeMillis() + "\n");
            writer.write("# Boost Parameters\n");
            writer.write("delay=" + delay + "\n");
            writer.write("duration=" + duration + "\n");
            writer.write("power=" + power + "\n");
            writer.write("# Timing Parameters\n");
            writer.write("shot_interval=" + shotInterval + "\n");
            writer.write("rpm_tolerance=" + rpmTolerance + "\n");
            writer.write("# Learning Stats\n");
            writer.write("shots_analyzed=" + shotsAnalyzed + "\n");

            writer.close();
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Save boost parameters to file (backward compatibility)
     * @param delay Boost delay in seconds
     * @param duration Boost duration in seconds
     * @param power Boost power (0.0 to 1.0)
     * @param shotsAnalyzed Number of shots used for learning
     * @return true if save successful, false otherwise
     */
    public static boolean saveBoostParameters(double delay, double duration, double power, int shotsAnalyzed) {
        return saveAllLearnedParameters(delay, duration, power, 0.150, 40.0, shotsAnalyzed);
    }

    /**
     * Load all learned parameters from file
     * @return double array [delay, duration, power, shotInterval, rpmTolerance, shotsAnalyzed] or null if load fails
     * Note: feedTime is no longer learned but still parsed for backward compatibility with old files
     */
    public static double[] loadAllLearnedParameters() {
        try {
            File file = new File(CONFIG_DIR + CONFIG_FILE);
            if (!file.exists()) {
                return null; // No saved config
            }

            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line;

            double delay = 0.020;
            double duration = 0.180;
            double power = 0.18;
            double shotInterval = 0.150;
            double rpmTolerance = 40.0;
            double shotsAnalyzed = 0;

            while ((line = reader.readLine()) != null) {
                line = line.trim();

                // Skip comments and empty lines
                if (line.startsWith("#") || line.isEmpty()) {
                    continue;
                }

                // Parse key=value pairs
                String[] parts = line.split("=");
                if (parts.length == 2) {
                    String key = parts[0].trim();
                    String value = parts[1].trim();

                    try {
                        switch (key) {
                            case "delay":
                                delay = Double.parseDouble(value);
                                break;
                            case "duration":
                                duration = Double.parseDouble(value);
                                break;
                            case "power":
                                power = Double.parseDouble(value);
                                break;
                            case "shot_interval":
                                shotInterval = Double.parseDouble(value);
                                break;
                            case "rpm_tolerance":
                                rpmTolerance = Double.parseDouble(value);
                                break;
                            case "feed_time":
                                // Parse but ignore - no longer learned (kept for backward compatibility)
                                break;
                            case "shots_analyzed":
                                shotsAnalyzed = Double.parseDouble(value);
                                break;
                        }
                    } catch (NumberFormatException e) {
                        // Skip invalid values
                    }
                }
            }

            reader.close();
            return new double[] { delay, duration, power, shotInterval, rpmTolerance, shotsAnalyzed };

        } catch (IOException e) {
            return null;
        }
    }

    /**
     * Load boost parameters from file (backward compatibility)
     * @return double array [delay, duration, power, shotsAnalyzed] or null if load fails
     */
    public static double[] loadBoostParameters() {
        double[] all = loadAllLearnedParameters();
        if (all == null || all.length < 4) return null;
        return new double[] { all[0], all[1], all[2], all[5] }; // delay, duration, power, shotsAnalyzed
    }

    /**
     * Delete saved configuration (reset to defaults)
     * @return true if delete successful, false otherwise
     */
    public static boolean deleteSavedConfig() {
        try {
            File file = new File(CONFIG_DIR + CONFIG_FILE);
            if (file.exists()) {
                return file.delete();
            }
            return true; // File doesn't exist, consider it deleted
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Check if saved configuration exists
     * @return true if config file exists
     */
    public static boolean configExists() {
        File file = new File(CONFIG_DIR + CONFIG_FILE);
        return file.exists();
    }
}


/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Movement recording and playback system for AURORA
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.*;
import java.util.*;

/**
 * MovementRecorder - Records robot movements and actions during teleop for autonomous playback
 *
 * Features:
 * - High-fidelity recording at 20Hz
 * - Captures drive movements and subsystem states
 * - File-based storage on robot controller
 * - Playback with safety integration
 * - Real-time recording statistics
 */
public class MovementRecorder {

    /**
     * Data structure for recorded waypoints
     */
    public static class ActionWaypoint implements Serializable {
        private static final long serialVersionUID = 1L;

        public final double timestamp;
        public final double axial;
        public final double lateral;
        public final double yaw;
        public final Map<String, Object> actions;

        public ActionWaypoint(double timestamp, double axial, double lateral, double yaw, Map<String, Object> actions) {
            this.timestamp = timestamp;
            this.axial = axial;
            this.lateral = lateral;
            this.yaw = yaw;
            this.actions = new HashMap<>(actions);
        }
    }

    // Recording state
    private List<ActionWaypoint> recording = new ArrayList<>();
    private ElapsedTime recordingTimer = new ElapsedTime();
    private boolean isRecording = false;
    private boolean isPaused = false;
    private String currentRecordingName = "";
    private static final double RECORDING_INTERVAL = 0.05; // 50ms intervals (20Hz)
    private double lastRecordTime = 0;

    // Statistics
    private int totalWaypoints = 0;
    private double recordingDuration = 0;
    private double pausedTime = 0;

    /**
     * Start a new recording session
     */
    public void startRecording(String recordingName) {
        recording.clear();
        recordingTimer.reset();
        isRecording = true;
        isPaused = false;
        currentRecordingName = recordingName;
        lastRecordTime = 0;
        totalWaypoints = 0;
        recordingDuration = 0;
        pausedTime = 0;
    }

    /**
     * Pause/resume the current recording
     */
    public void togglePause() {
        if (!isRecording) return;

        if (isPaused) {
            // Resume recording
            pausedTime += recordingTimer.seconds() - lastRecordTime;
            isPaused = false;
        } else {
            // Pause recording
            isPaused = true;
            lastRecordTime = recordingTimer.seconds();
        }
    }

    /**
     * Stop recording and save to file
     */
    public boolean stopRecording() {
        if (!isRecording) return false;

        isRecording = false;
        isPaused = false;
        recordingDuration = recordingTimer.seconds() - pausedTime;
        return saveRecording();
    }

    /**
     * Record a waypoint if recording is active
     */
    public void recordWaypoint(double axial, double lateral, double yaw, Map<String, Object> actions) {
        if (!isRecording || isPaused) return;

        double currentTime = recordingTimer.seconds() - pausedTime;
        if (currentTime - lastRecordTime >= RECORDING_INTERVAL) {
            recording.add(new ActionWaypoint(currentTime, axial, lateral, yaw, actions));
            lastRecordTime = currentTime;
            totalWaypoints++;
        }
    }

    /**
     * Save the current recording to file
     */
    private boolean saveRecording() {
        try {
            String filename = "/sdcard/FIRST/recordings/" + currentRecordingName + ".rec";
            File file = new File(filename);
            file.getParentFile().mkdirs();

            // Create recording metadata
            Map<String, Object> metadata = new HashMap<>();
            metadata.put("name", currentRecordingName);
            metadata.put("duration", recordingDuration);
            metadata.put("waypoints", totalWaypoints);
            metadata.put("recorded", System.currentTimeMillis());
            metadata.put("version", "1.0");

            // Save both metadata and waypoints
            Map<String, Object> recordingData = new HashMap<>();
            recordingData.put("metadata", metadata);
            recordingData.put("waypoints", recording);

            try (ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(file))) {
                oos.writeObject(recordingData);
            }

            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Load a recording from file
     */
    @SuppressWarnings("unchecked")
    public List<ActionWaypoint> loadRecording(String recordingName) {
        try {
            String filename = "/sdcard/FIRST/recordings/" + recordingName + ".rec";
            try (ObjectInputStream ois = new ObjectInputStream(new FileInputStream(filename))) {
                Map<String, Object> recordingData = (Map<String, Object>) ois.readObject();
                return (List<ActionWaypoint>) recordingData.get("waypoints");
            }
        } catch (Exception e) {
            return new ArrayList<>();
        }
    }

    /**
     * Get metadata for a recording
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> getRecordingMetadata(String recordingName) {
        try {
            String filename = "/sdcard/FIRST/recordings/" + recordingName + ".rec";
            try (ObjectInputStream ois = new ObjectInputStream(new FileInputStream(filename))) {
                Map<String, Object> recordingData = (Map<String, Object>) ois.readObject();
                return (Map<String, Object>) recordingData.get("metadata");
            }
        } catch (Exception e) {
            return new HashMap<>();
        }
    }

    /**
     * List available recordings
     */
    public List<String> getAvailableRecordings() {
        List<String> recordings = new ArrayList<>();
        File recordingsDir = new File("/sdcard/FIRST/recordings/");

        if (recordingsDir.exists() && recordingsDir.isDirectory()) {
            File[] files = recordingsDir.listFiles((dir, name) -> name.endsWith(".rec"));
            if (files != null) {
                for (File file : files) {
                    String name = file.getName();
                    recordings.add(name.substring(0, name.length() - 4)); // Remove .rec extension
                }
            }
        }

        return recordings;
    }

    /**
     * Delete a recording
     */
    public boolean deleteRecording(String recordingName) {
        try {
            String filename = "/sdcard/FIRST/recordings/" + recordingName + ".rec";
            File file = new File(filename);
            return file.delete();
        } catch (Exception e) {
            return false;
        }
    }

    // Getters for recording state
    public boolean isRecording() { return isRecording; }
    public boolean isPaused() { return isPaused; }
    public String getCurrentRecordingName() { return currentRecordingName; }
    public int getWaypointCount() { return recording.size(); }
    public double getRecordingDuration() { return recordingTimer.seconds() - pausedTime; }
    public double getCurrentTime() { return recordingTimer.seconds(); }

    /**
     * Get telemetry data for the recording system
     */
    public RecordingTelemetryData getTelemetryData() {
        return new RecordingTelemetryData(
            isRecording,
            isPaused,
            currentRecordingName,
            getWaypointCount(),
            getRecordingDuration(),
            getAvailableRecordings().size()
        );
    }

    /**
     * Data container for recording telemetry
     */
    public static class RecordingTelemetryData {
        public final boolean isRecording;
        public final boolean isPaused;
        public final String currentRecording;
        public final int waypointCount;
        public final double duration;
        public final int availableRecordings;

        public RecordingTelemetryData(boolean isRecording, boolean isPaused, String currentRecording,
                                    int waypointCount, double duration, int availableRecordings) {
            this.isRecording = isRecording;
            this.isPaused = isPaused;
            this.currentRecording = currentRecording;
            this.waypointCount = waypointCount;
            this.duration = duration;
            this.availableRecordings = availableRecordings;
        }
    }
}

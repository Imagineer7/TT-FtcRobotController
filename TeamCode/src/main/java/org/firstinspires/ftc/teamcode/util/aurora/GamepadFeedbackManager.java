/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Advanced Gamepad Feedback Manager for AURORA System
 */

package org.firstinspires.ftc.teamcode.util.aurora;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * GamepadFeedbackManager - Centralized haptic and LED feedback system
 *
 * Provides advanced gamepad feedback features:
 * - Rumble patterns (simple, complex, rhythmic)
 * - LED color control (if supported by gamepad)
 * - Feedback profiles for different events
 * - Coordinated multi-gamepad feedback
 * - Smart queuing and priority system
 *
 * Usage:
 * 1. Create instance with gamepads
 * 2. Register feedback events
 * 3. Call update() in main loop
 * 4. Trigger events as needed
 */
public class GamepadFeedbackManager {

    // ========================================================================================
    // FEEDBACK PROFILES - Predefined patterns for common events
    // ========================================================================================

    /**
     * Feedback profile - defines how a gamepad should respond to an event
     */
    public static class FeedbackProfile {
        public String name;
        public RumblePattern rumblePattern;
        public LEDColor ledColor;
        public int ledDurationMs;
        public int priority; // Higher = more important

        public FeedbackProfile(String name, RumblePattern pattern, LEDColor color, int ledDuration, int priority) {
            this.name = name;
            this.rumblePattern = pattern;
            this.ledColor = color;
            this.ledDurationMs = ledDuration;
            this.priority = priority;
        }
    }

    /**
     * Rumble pattern definition
     */
    public static class RumblePattern {
        public double[] leftIntensities;   // Array of left motor intensities (0.0 to 1.0)
        public double[] rightIntensities;  // Array of right motor intensities (0.0 to 1.0)
        public int[] durations;            // Duration of each pulse in ms
        public int[] pauses;               // Pause between pulses in ms

        public RumblePattern(double[] left, double[] right, int[] durations, int[] pauses) {
            this.leftIntensities = left;
            this.rightIntensities = right;
            this.durations = durations;
            this.pauses = pauses;
        }

        // Quick pattern constructors
        public static RumblePattern single(double intensity, int duration) {
            return new RumblePattern(
                new double[]{intensity},
                new double[]{intensity},
                new int[]{duration},
                new int[]{0}
            );
        }

        public static RumblePattern doublePulse(double intensity, int pulseDuration, int pauseDuration) {
            return new RumblePattern(
                new double[]{intensity, intensity},
                new double[]{intensity, intensity},
                new int[]{pulseDuration, pulseDuration},
                new int[]{pauseDuration, 0}
            );
        }

        public static RumblePattern triplePulse(double intensity, int pulseDuration, int pauseDuration) {
            return new RumblePattern(
                new double[]{intensity, intensity, intensity},
                new double[]{intensity, intensity, intensity},
                new int[]{pulseDuration, pulseDuration, pulseDuration},
                new int[]{pauseDuration, pauseDuration, 0}
            );
        }

        public static RumblePattern crescendo(int steps, int stepDuration) {
            double[] intensities = new double[steps];
            int[] durations = new int[steps];
            int[] pauses = new int[steps];

            for (int i = 0; i < steps; i++) {
                intensities[i] = (i + 1) / (double)steps;
                durations[i] = stepDuration;
                pauses[i] = 0;
            }

            return new RumblePattern(intensities, intensities, durations, pauses);
        }

        public static RumblePattern heartbeat(int beats) {
            double[] left = new double[beats * 2];
            double[] right = new double[beats * 2];
            int[] durations = new int[beats * 2];
            int[] pauses = new int[beats * 2];

            for (int i = 0; i < beats; i++) {
                left[i * 2] = 1.0;
                right[i * 2] = 1.0;
                durations[i * 2] = 100;
                pauses[i * 2] = 100;

                left[i * 2 + 1] = 0.6;
                right[i * 2 + 1] = 0.6;
                durations[i * 2 + 1] = 80;
                pauses[i * 2 + 1] = (i < beats - 1) ? 300 : 0;
            }

            return new RumblePattern(left, right, durations, pauses);
        }

        public static RumblePattern emergency() {
            // Rapid alternating left-right pattern
            return new RumblePattern(
                new double[]{1.0, 0.0, 1.0, 0.0, 1.0},
                new double[]{0.0, 1.0, 0.0, 1.0, 0.0},
                new int[]{100, 100, 100, 100, 100},
                new int[]{0, 0, 0, 0, 0}
            );
        }
    }

    /**
     * LED color (if gamepad supports RGB LEDs)
     */
    public enum LEDColor {
        OFF(0, 0, 0),
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 255, 0),
        CYAN(0, 255, 255),
        MAGENTA(255, 0, 255),
        WHITE(255, 255, 255),
        ORANGE(255, 165, 0),
        PURPLE(128, 0, 128);

        public final int r, g, b;

        LEDColor(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    // ========================================================================================
    // PREDEFINED PROFILES FOR AURORA SYSTEM
    // ========================================================================================

    // Success/Positive feedback
    public static final FeedbackProfile SUCCESS = new FeedbackProfile(
        "Success",
        RumblePattern.single(0.5, 150),
        LEDColor.GREEN,
        500,
        5
    );

    public static final FeedbackProfile POSITION_REACHED = new FeedbackProfile(
        "Position Reached",
        RumblePattern.doublePulse(0.4, 100, 80),
        LEDColor.CYAN,
        300,
        6
    );

    // Warning feedback
    public static final FeedbackProfile SOFT_WARNING = new FeedbackProfile(
        "Soft Warning",
        RumblePattern.single(0.3, 100),
        LEDColor.YELLOW,
        400,
        7
    );

    public static final FeedbackProfile LIMIT_WARNING = new FeedbackProfile(
        "Limit Warning",
        RumblePattern.doublePulse(0.6, 150, 100),
        LEDColor.ORANGE,
        600,
        8
    );

    // Critical/Error feedback
    public static final FeedbackProfile STALL_ALERT = new FeedbackProfile(
        "Stall Alert",
        RumblePattern.triplePulse(0.8, 200, 100),
        LEDColor.RED,
        1000,
        9
    );

    public static final FeedbackProfile EMERGENCY = new FeedbackProfile(
        "Emergency",
        RumblePattern.emergency(),
        LEDColor.RED,
        2000,
        10
    );

    // System feedback
    public static final FeedbackProfile MODE_CHANGE = new FeedbackProfile(
        "Mode Change",
        RumblePattern.single(0.3, 100),
        LEDColor.BLUE,
        400,
        4
    );

    public static final FeedbackProfile CALIBRATION_COMPLETE = new FeedbackProfile(
        "Calibration Complete",
        RumblePattern.crescendo(3, 100),
        LEDColor.CYAN,
        800,
        6
    );

    public static final FeedbackProfile HEARTBEAT = new FeedbackProfile(
        "Heartbeat",
        RumblePattern.heartbeat(2),
        LEDColor.MAGENTA,
        1000,
        3
    );

    // ========================================================================================
    // STATE AND HARDWARE
    // ========================================================================================

    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private boolean enabled = true;
    private ElapsedTime feedbackTimer = new ElapsedTime();

    // Pattern playback state
    private RumblePattern currentPattern = null;
    private int currentPatternStep = 0;
    private boolean patternPlaying = false;
    private ElapsedTime patternTimer = new ElapsedTime();
    private Gamepad targetGamepad = null;

    // Priority queue for feedback (simple implementation)
    private FeedbackProfile queuedFeedback = null;
    private Gamepad queuedGamepad = null;

    // ========================================================================================
    // CONSTRUCTOR
    // ========================================================================================

    /**
     * Create feedback manager with gamepads
     * @param gamepad1 Driver gamepad (can be null)
     * @param gamepad2 Operator gamepad (can be null)
     */
    public GamepadFeedbackManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Update gamepads (if they change during operation)
     */
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Enable or disable all feedback
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            stopAll();
        }
    }

    /**
     * Check if feedback is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    // ========================================================================================
    // FEEDBACK TRIGGERING
    // ========================================================================================

    /**
     * Trigger a feedback profile on both gamepads
     */
    public void trigger(FeedbackProfile profile) {
        trigger(profile, true, true);
    }

    /**
     * Trigger a feedback profile on specified gamepads
     * @param profile The feedback profile to play
     * @param gp1 True to trigger on gamepad1
     * @param gp2 True to trigger on gamepad2
     */
    public void trigger(FeedbackProfile profile, boolean gp1, boolean gp2) {
        if (!enabled || profile == null) return;

        // Check if we should interrupt current pattern
        if (patternPlaying && queuedFeedback != null) {
            if (profile.priority <= queuedFeedback.priority) {
                return; // Lower priority, ignore
            }
        }

        // Play pattern on selected gamepads
        if (gp1 && gamepad1 != null) {
            playPattern(profile, gamepad1);
        }
        if (gp2 && gamepad2 != null) {
            playPattern(profile, gamepad2);
        }
    }

    /**
     * Trigger simple rumble on both gamepads
     */
    public void rumble(int durationMs) {
        if (!enabled) return;
        if (gamepad1 != null) gamepad1.rumble(durationMs);
        if (gamepad2 != null) gamepad2.rumble(durationMs);
    }

    /**
     * Trigger custom rumble pattern
     */
    public void rumbleCustom(double leftIntensity, double rightIntensity, int durationMs) {
        if (!enabled) return;
        if (gamepad1 != null) gamepad1.rumble(leftIntensity, rightIntensity, durationMs);
        if (gamepad2 != null) gamepad2.rumble(leftIntensity, rightIntensity, durationMs);
    }

    /**
     * Set LED color (if supported)
     * Note: Most FTC gamepads don't support LED control, but this is here for future compatibility
     */
    public void setLED(LEDColor color, Gamepad gamepad) {
        if (!enabled || gamepad == null) return;
        // LED control would go here if SDK supports it
        // gamepad.setLedColor(color.r, color.g, color.b);
    }

    // ========================================================================================
    // PATTERN PLAYBACK
    // ========================================================================================

    /**
     * Play a complex rumble pattern
     */
    private void playPattern(FeedbackProfile profile, Gamepad gamepad) {
        if (gamepad == null || profile.rumblePattern == null) return;

        RumblePattern pattern = profile.rumblePattern;

        // For single-step patterns, just play directly
        if (pattern.durations.length == 1) {
            gamepad.rumble(
                pattern.leftIntensities[0],
                pattern.rightIntensities[0],
                pattern.durations[0]
            );
            return;
        }

        // For multi-step patterns, queue them
        currentPattern = pattern;
        currentPatternStep = 0;
        targetGamepad = gamepad;
        patternPlaying = true;
        patternTimer.reset();
        queuedFeedback = profile;
        queuedGamepad = gamepad;

        // Start first step
        executePatternStep();
    }

    /**
     * Execute current pattern step
     */
    private void executePatternStep() {
        if (currentPattern == null || targetGamepad == null) return;
        if (currentPatternStep >= currentPattern.durations.length) {
            patternPlaying = false;
            return;
        }

        targetGamepad.rumble(
            currentPattern.leftIntensities[currentPatternStep],
            currentPattern.rightIntensities[currentPatternStep],
            currentPattern.durations[currentPatternStep]
        );
    }

    /**
     * Update pattern playback (call in main loop)
     */
    public void update() {
        if (!patternPlaying) return;

        int currentDuration = currentPattern.durations[currentPatternStep];
        int currentPause = currentPattern.pauses[currentPatternStep];
        int totalStepTime = currentDuration + currentPause;

        if (patternTimer.milliseconds() >= totalStepTime) {
            currentPatternStep++;
            patternTimer.reset();

            if (currentPatternStep < currentPattern.durations.length) {
                executePatternStep();
            } else {
                patternPlaying = false;
                currentPattern = null;
                queuedFeedback = null;
            }
        }
    }

    /**
     * Stop all feedback immediately
     */
    public void stopAll() {
        patternPlaying = false;
        currentPattern = null;
        queuedFeedback = null;

        if (gamepad1 != null) gamepad1.stopRumble();
        if (gamepad2 != null) gamepad2.stopRumble();
    }

    /**
     * Check if a pattern is currently playing
     */
    public boolean isPlaying() {
        return patternPlaying;
    }

    /**
     * Get name of currently playing feedback
     */
    public String getCurrentFeedbackName() {
        return (queuedFeedback != null) ? queuedFeedback.name : "None";
    }
}


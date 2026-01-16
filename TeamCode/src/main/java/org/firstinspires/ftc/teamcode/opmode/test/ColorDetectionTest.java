package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.aurora.IndexingConfig;

/**
 * ColorDetectionTest - Test OpMode to validate pattern-based color detection
 *
 * This OpMode tests the pattern-based color detection algorithm using measured values:
 * - Purple artifact: R:0.1-0.15, G:0.1-0.15, B:0.15-0.3 (Red≈Green, Blue>Red/Green)
 * - Green artifact: R:0.05-0.1, G:0.2-0.35, B:0.1-0.2 (Green dominant, lower values)
 *
 * The system analyzes RGB relationships and characteristic patterns instead of
 * using rigid thresholds, making it more robust to lighting variations.
 *
 * Use this to verify that the pattern-based detection works correctly.
 */
@TeleOp(name = "Color Detection Test", group = "Test")
public class ColorDetectionTest extends OpMode {

    private IndexingConfig config;

    // Test cases with measured values
    private final TestCase[] testCases = {
        // Purple artifact - based on your measurements R:0.1-0.15, G:0.1-0.15, B:0.15-0.3
        new TestCase("Purple (Typical)", 0.12, 0.13, 0.22, "PURPLE"),
        new TestCase("Purple (Low End)", 0.10, 0.10, 0.15, "PURPLE"),
        new TestCase("Purple (High End)", 0.15, 0.15, 0.30, "PURPLE"),
        new TestCase("Purple (R≈G, B high)", 0.125, 0.125, 0.25, "PURPLE"),

        // Green artifact - NEW measurements R:0.05-0.1, G:0.2-0.35, B:0.1-0.2
        new TestCase("Green (Typical)", 0.07, 0.28, 0.15, "GREEN"),
        new TestCase("Green (Low End)", 0.05, 0.20, 0.10, "GREEN"),
        new TestCase("Green (High End)", 0.10, 0.35, 0.20, "GREEN"),
        new TestCase("Green (Mid Range)", 0.075, 0.25, 0.125, "GREEN"),

        // Edge cases that should be UNKNOWN
        new TestCase("Unknown (Too Dark)", 0.02, 0.02, 0.02, "UNKNOWN"),
        new TestCase("Unknown (Red High)", 0.80, 0.20, 0.15, "UNKNOWN"),
        new TestCase("Unknown (All Equal)", 0.50, 0.50, 0.50, "UNKNOWN"),

        // Borderline cases
        new TestCase("Purple Edge Case", 0.08, 0.08, 0.12, "PURPLE"),
        new TestCase("Green Edge Case", 0.06, 0.18, 0.11, "GREEN")
    };

    private int currentTestIndex = 0;
    private boolean testInProgress = false;

    @Override
    public void init() {
        config = new IndexingConfig();
        telemetry.addLine("🎨 COLOR DETECTION TEST");
        telemetry.addLine("Based on measured RGB values:");
        telemetry.addLine("Purple: R:0.1-0.15, G:0.1-0.15, B:0.15-0.3");
        telemetry.addLine("Green:  R:0.05-0.1, G:0.2-0.35, B:0.1-0.2");
        telemetry.addLine("");
        telemetry.addLine("Press [A] to run next test");
        telemetry.addLine("Press [Y] to run all tests");
        telemetry.update();
    }

    @Override
    public void loop() {
        // [A] - Run next test
        if (gamepad1.a && !testInProgress) {
            runSingleTest();
        }

        // [Y] - Run all tests
        if (gamepad1.y && !testInProgress) {
            runAllTests();
        }

        // Reset test progress when buttons released
        if (!gamepad1.a && !gamepad1.y) {
            testInProgress = false;
        }

        displayCurrentStatus();
    }

    private void runSingleTest() {
        testInProgress = true;

        if (currentTestIndex < testCases.length) {
            TestCase test = testCases[currentTestIndex];
            String result = runTest(test);

            telemetry.addLine("🧪 TEST RESULT:");
            telemetry.addData("Test", test.name);
            telemetry.addData("RGB Input", String.format("R:%.2f G:%.2f B:%.2f", test.red, test.green, test.blue));

            // Show ratio scores for both colors
            double purpleScore = config.calculateColorConfidence(test.red, test.green, test.blue, "PURPLE");
            double greenScore = config.calculateColorConfidence(test.red, test.green, test.blue, "GREEN");
            telemetry.addData("Scores", String.format("Purple:%.3f Green:%.3f", purpleScore, greenScore));

            telemetry.addData("Expected", test.expectedColor);
            telemetry.addData("Actual", result);
            telemetry.addData("Status", test.expectedColor.equals(result) ? "✅ PASS" : "❌ FAIL");

            currentTestIndex++;
            if (currentTestIndex >= testCases.length) {
                currentTestIndex = 0; // Reset for next cycle
            }
        }

        telemetry.update();
    }

    private void runAllTests() {
        testInProgress = true;

        telemetry.addLine("🧪 RUNNING ALL TESTS:");
        telemetry.addLine("");

        int passed = 0;
        int total = testCases.length;

        for (TestCase test : testCases) {
            String result = runTest(test);
            boolean pass = test.expectedColor.equals(result);
            if (pass) passed++;

            telemetry.addData(test.name,
                String.format("%s (Expected: %s, Got: %s)",
                    pass ? "✅" : "❌", test.expectedColor, result));
        }

        telemetry.addLine("");
        telemetry.addData("Results", String.format("%d/%d tests passed", passed, total));
        telemetry.addData("Success Rate", String.format("%.1f%%", (passed * 100.0) / total));

        telemetry.update();
    }

    private String runTest(TestCase test) {
        String detected = config.detectArtifactColor(test.red, test.green, test.blue);

        if (!"UNKNOWN".equals(detected)) {
            double confidence = config.calculateColorConfidence(test.red, test.green, test.blue, detected);
            if (confidence >= config.getColorDetectionMinScore()) {
                return detected;
            }
        }

        return "UNKNOWN";
    }

    private void displayCurrentStatus() {
        telemetry.addLine("🎨 COLOR DETECTION TEST");
        telemetry.addLine("");
        telemetry.addLine("Pattern-Based Detection System:");

        telemetry.addLine("Purple Pattern:");
        telemetry.addLine("  • Red ≈ Green (similar values)");
        telemetry.addLine("  • Blue > Red and Blue > Green");
        telemetry.addLine("  • Range: R:0.1-0.15, G:0.1-0.15, B:0.15-0.3");
        telemetry.addLine("");
        telemetry.addLine("Green Pattern:");
        telemetry.addLine("  • Green > Red and Green > Blue");
        telemetry.addLine("  • Values: R:0.05-0.1, G:0.2-0.35, B:0.1-0.2");

        telemetry.addData("Min Score Required", String.format("%.2f", config.getColorDetectionMinScore()));

        telemetry.addLine("");
        telemetry.addLine("How it works:");
        telemetry.addLine("• Analyzes RGB relationships and patterns");
        telemetry.addLine("• Scores based on characteristic signatures");
        telemetry.addLine("• Purple: R≈G similar, Blue dominant");
        telemetry.addLine("• Green: Green dominant, lower values");

        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("[A] Run next test");
        telemetry.addLine("[Y] Run all tests");
        telemetry.addData("Next Test", currentTestIndex < testCases.length ?
            testCases[currentTestIndex].name : "Reset to first");

        telemetry.update();
    }

    /**
     * Test case data structure
     */
    private static class TestCase {
        final String name;
        final double red;
        final double green;
        final double blue;
        final String expectedColor;

        TestCase(String name, double red, double green, double blue, String expectedColor) {
            this.name = name;
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.expectedColor = expectedColor;
        }
    }
}

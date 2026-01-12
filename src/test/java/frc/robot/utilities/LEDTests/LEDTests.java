package frc.robot.utilities.LEDTests;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.LEDs.LEDSubsystem2025;


public class LEDTests {

    private LEDSubsystem2025 ledSubsystem = LEDSubsystem2025.getInstance();

    @BeforeEach
    public void setup() {
        // Reset the LED subsystem before each test
        ledSubsystem.setPattern(LEDPattern.kOff);
        ledSubsystem.periodic();
    }

    @Test
    public void testLEDStripHasLength() {
        assertNotEquals(null, ledSubsystem.stripBuffer);
        assertNotEquals(0, ledSubsystem.stripBuffer.getLength());
    }

    @Test
    public void testLEDBaseStateIsOff() {
        for (int i = 0; i < ledSubsystem.stripBuffer.getLength(); i++) {
            assertEquals(0, ledSubsystem.stripBuffer.getRed(i),
                "LED " + i + " should be off. It's current red value is " + ledSubsystem.stripBuffer.getRed(i));
            assertEquals(0, ledSubsystem.stripBuffer.getGreen(i),
                "LED " + i + " should be off. It's current green value is " + ledSubsystem.stripBuffer.getGreen(i));
            assertEquals(0, ledSubsystem.stripBuffer.getBlue(i), 
                "LED " + i + " should be off. It's current blue value is " + ledSubsystem.stripBuffer.getBlue(i));
        }
    }

    @Test
    public void testSetPatternAppliesPatternToStrip() {
        LEDPattern pattern = LEDPattern.solid(Color.kRed);
        ledSubsystem.setPattern(pattern);

        // Simulate periodic update to apply the pattern
        ledSubsystem.periodic();

        // Check that the buffer has the expected color for all LEDs
        for (int i = 0; i < ledSubsystem.stripBuffer.getLength(); i++) {
            assertEquals(255, ledSubsystem.stripBuffer.getRed(i), "LED " + i + " should be red");
            assertEquals(0, ledSubsystem.stripBuffer.getGreen(i), "LED " + i + " shouldn't be green");
            assertEquals(0, ledSubsystem.stripBuffer.getBlue(i), "LED " + i + " shouldn't be blue");
        }
    }

    @Test
    public void testSetPatternForDurationAppliesPatternToStrip() {
        LEDPattern pattern = LEDPattern.solid(Color.kBlue);
        double duration = 1.0;
        ledSubsystem.setPatternForDuration(pattern, duration);

        // Simulate periodic update to apply the pattern
        ledSubsystem.periodic();

        // Check that the buffer has the expected color for all LEDs
        for (int i = 0; i < 36; i++) {
            assertEquals(0, ledSubsystem.stripBuffer.getRed(i), "LED " + i + " should be blue (red=0)");
            assertEquals(0, ledSubsystem.stripBuffer.getGreen(i), "LED " + i + " should be blue (green=0)");
            assertEquals(255, ledSubsystem.stripBuffer.getBlue(i), "LED " + i + " should be blue (blue=255)");
        }
    }

    @Test
    public void testCombinePatternsForDurationAppliesPatternsToSections() {
        LEDPattern leftPattern = LEDPattern.solid(Color.kGreen);
        LEDPattern rightPattern = LEDPattern.solid(Color.kYellow);
        LEDPattern secretPattern = LEDPattern.solid(Color.kPurple);
        double duration = 1.0;
        ledSubsystem.combinePatternsForDuration(leftPattern, rightPattern, secretPattern, duration);

        // Simulate periodic update to apply the patterns
        ledSubsystem.periodic();

        // Left side: indices 0-12
        for (int i = 0; i <= 12; i++) {
            assertEquals(0, ledSubsystem.stripBuffer.getRed(i), "LED " + i + " should be green (red=0)");
            assertEquals(128, ledSubsystem.stripBuffer.getGreen(i), "LED " + i + " should be green (green=128)");
            assertEquals(0, ledSubsystem.stripBuffer.getBlue(i), "LED " + i + " should be green (blue=0)");
        }
        // Secret buffer: indices 14-19
        for (int i = 14; i <= 19; i++) {
            assertEquals(128, ledSubsystem.stripBuffer.getRed(i), "LED " + i + " should be purple (red=128)");
            assertEquals(0, ledSubsystem.stripBuffer.getGreen(i), "LED " + i + " should be purple (green=0)");
            assertEquals(128, ledSubsystem.stripBuffer.getBlue(i), "LED " + i + " should be purple (blue=128)");
        }
        // Right side: indices 20-35
        for (int i = 20; i <= 35; i++) {
            assertEquals(255, ledSubsystem.stripBuffer.getRed(i), "LED " + i + " should be yellow (red=255)");
            assertEquals(255, ledSubsystem.stripBuffer.getGreen(i), "LED " + i + " should be yellow (green=255)");
            assertEquals(0, ledSubsystem.stripBuffer.getBlue(i), "LED " + i + " should be yellow (blue=0)");
        }
    }

    @Test
    public void testLEDSubsystemInfiniteDurationIsInfinite() {}
}
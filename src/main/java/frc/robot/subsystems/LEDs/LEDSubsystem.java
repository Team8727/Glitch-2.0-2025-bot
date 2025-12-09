// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.GlitchLEDPatterns.enzoMap;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED lightStrip;
  public final AddressableLEDBuffer stripBuffer;

  /**
   * The pattern that will play once any other pattern stops running on the strip.
   */
  public static final LEDPattern defaultPattern = GlitchLEDPatterns.fire(GlitchLEDPatterns.theCoolerGreen);

  /**
   * Represents a section of the LED strip with a specific pattern and duration.
   */
  private class Section {
    private final AddressableLEDBufferView bufferView;
    private LEDPattern pattern = defaultPattern;

    private static final double infiniteDurationSeconds = -1.0;
    private double durationSeconds = infiniteDurationSeconds;
    private double elapsedSeconds = 0.0;

    public Section(AddressableLEDBuffer buffer, int startIndex, int endIndex) {
      bufferView = buffer.createView(startIndex, endIndex);
    }

    /**
     * Sets the pattern for a duration in seconds.
     * @param pattern The pattern to set.
     * @param durationSeconds The duration in seconds to set the pattern to before setting the strip to the default pattern.
     */
    public void setPattern(LEDPattern pattern, double durationSeconds) {
      this.pattern = pattern;
      this.durationSeconds = durationSeconds;
      this.elapsedSeconds = 0.0;
    }

    /**
     * Sets the pattern for an infinite duration.
     * @param pattern The pattern to set.
     */
    public void setPattern(LEDPattern pattern) {
      setPattern(pattern, infiniteDurationSeconds);
    }

    /**
     * Updates the section's pattern if it has a finite duration.
     * Applies the current pattern to the buffer view.
     * Please put this in periodic.
     * 
     * @param deltaTimeSeconds The time since the last update in seconds.
     * @param animPattern The pattern that the animation will overlay.
     * @param animBool A boolean that decides if the animation plays on the section.
     */

    public void update(double deltaTimeSeconds, LEDPattern animPattern, boolean animBool) {
      if (durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;
      }
      if (elapsedSeconds >= durationSeconds && pattern == LEDPattern.kOff) {
        if (!animBool) {
          Section.this.setPattern(LEDPattern.solid(Color.kBlack), 0);
        } else {
          Section.this.setPattern(defaultPattern, infiniteDurationSeconds);
        }
        durationSeconds = infiniteDurationSeconds;
        elapsedSeconds = 0.0;
      } else if (elapsedSeconds >= durationSeconds && durationSeconds != infiniteDurationSeconds) {
        pattern = LEDPattern.kOff;
      } else {
        pattern.applyTo(this.bufferView);
      }
    }

    public AddressableLEDBufferView getBufferView() {
      return this.bufferView;
    }
    public int getLength() {
      return this.bufferView.getLength();
    }
  }

  private final Section leftSide;
  private final Section rightSide;
  private final Section secretBuffer;
  
  /** Creates a new LEDSubsystem. */
  private LEDSubsystem() {
    // LED setup and port configuration
    lightStrip = new AddressableLED(5); // Correct PWM port
    stripBuffer = new AddressableLEDBuffer(36); // Correct LED count
    leftSide = new Section(stripBuffer, 0, 13);
    rightSide = new Section(stripBuffer, 35, 20);
    secretBuffer = new Section(stripBuffer, 14, 19);

    lightStrip.setLength(stripBuffer.getLength());

    lightStrip.setData(stripBuffer);
    lightStrip.start();
  }

  static LEDSubsystem instance = null;

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
      return instance;
    } else {
      return instance;
    }
  }

  /** TODO: Change or delete this when using a robot other than the 2025 robot
   * Sets a pattern to each section of the strip for an infinite duration.
   * @param pattern The LEDPattern to apply.
   */
  public void setPattern(LEDPattern pattern) {
    setPatternForDuration(pattern, Section.infiniteDurationSeconds);
  }

  /** TODO: Change or delete this when using a robot other than the 2025 robot
   * Sets a pattern to each section of the strip for a duration in seconds
   * @param pattern the LEDPattern to apply
   * @param seconds the duration in seconds to set the pattern to before turning the strip off
   */
  public void setPatternForDuration(LEDPattern pattern, double seconds) {
    leftSide.setPattern(pattern, seconds);
    rightSide.setPattern(pattern, seconds);
    secretBuffer.setPattern(pattern, seconds);
  }

  /* TODO: Change or delete this when using a robot other than the 2025 robot
   * Sets different patterns to each Section of the strip for an infinite duration.
   */
  public void combinePatterns(LEDPattern leftPattern, LEDPattern rightPattern, LEDPattern secretPattern) {
    combinePatternsForDuration(leftPattern, rightPattern, secretPattern, Section.infiniteDurationSeconds);
  }

  /* TODO: Change or delete this when using a robot other than the 2025 robot
   * Sets different patterns to each Section of the strip for a duration in seconds.
   */
  public void combinePatternsForDuration(LEDPattern leftPattern, LEDPattern rightPattern, LEDPattern secretPattern, double seconds) {
    leftSide.setPattern(leftPattern, seconds);
    rightSide.setPattern(rightPattern, seconds);
    secretBuffer.setPattern(secretPattern, seconds);
  }

  public void enzoLEDS(enzoMap enzoMap, double seconds) {
    combinePatternsForDuration(enzoMap.getEnzoMap(), enzoMap.getEnzoMap(), defaultPattern, seconds);
  }

  @Override
  public void periodic() {
    final double deltaTimeSeconds = 0.02; // TODO: Is there a way to ensure this is accurate even with overruns?

      leftSide.update(
        deltaTimeSeconds, 
        defaultPattern, 
        true);
      rightSide.update(
        deltaTimeSeconds, 
        defaultPattern, 
        true);
      secretBuffer.update(
        deltaTimeSeconds, 
        defaultPattern, 
        false);
    
    lightStrip.setData(stripBuffer);
  }
}
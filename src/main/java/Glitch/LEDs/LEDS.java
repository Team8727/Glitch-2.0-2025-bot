// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Glitch.LEDs;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LEDS extends SubsystemBase {
  private final AddressableLED lightStrip;
  public final AddressableLEDBuffer stripBuffer;
  private final List<Section> sectionList;

  public static final double infiniteDurationSeconds = -1.0;

  /**
   * Represents a section of the LED strip with a specific pattern and duration.
   */
  public class Section {
    private final AddressableLEDBufferView bufferView;
    private LEDPattern pattern = LEDPattern.kOff;

    private static final double infiniteDurationSeconds = -1.0;
    private double durationSeconds = infiniteDurationSeconds;
    private double elapsedSeconds = 0.0;

    private LEDPattern basePattern;

    private Section(int startIndex, int endIndex) {
      bufferView = stripBuffer.createView(startIndex, endIndex);
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

    public void setBase(LEDPattern pattern) {
      if (pattern == null) {
        pattern = LEDPattern.kOff;
      } else {
        this.basePattern = pattern;
      }
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

    public void update(double deltaTimeSeconds) {
      if (durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;
        if (elapsedSeconds >= durationSeconds) {
          Section.this.setPattern(basePattern);
        }
      }

      pattern.applyTo(this.bufferView);
    }

    public AddressableLEDBufferView getBufferView() {
      return this.bufferView;
    }

    public int getLength() {
      return this.bufferView.getLength();
    }
  }
  
  /** Creates a new LEDSubsystem. */
  protected LEDS(int port, int length, int... sectionLengths) {
    // LED setup and port configuration
    lightStrip = new AddressableLED(port); // Correct PWM port
    stripBuffer = new AddressableLEDBuffer(length); // Correct LED count
    sectionList = new java.util.ArrayList<Section>();

    int index = 0;
  
    for (int sectionLength: sectionLengths) {
      if (sectionLength == 0) {
        throw new IllegalArgumentException("Section lengths must be positive integers.");
      }
      if (sectionLength < 0) {
        sectionList.add(new Section(index - sectionLength - 1, index));
        index -= sectionLength;
      } else {
        sectionList.add(new Section(index, index + sectionLength - 1));
        index += sectionLength;
      }
    }

    lightStrip.setLength(stripBuffer.getLength());

    lightStrip.setData(stripBuffer);
    lightStrip.start();
  }

  public List<Section> getSections() {
    return sectionList;
  }

  @Override
  public void periodic() {
    final double deltaTimeSeconds = 0.02; // TODO: Is there a way to ensure this is accurate even with overruns?

    for (Section section : sectionList) {
      section.update(deltaTimeSeconds);
    }
    lightStrip.setData(stripBuffer);
  }
}
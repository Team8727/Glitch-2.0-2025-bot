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
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDs.LEDPatterns.enzoMap;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED lightStrip;
  private final AddressableLEDBuffer stripBuffer;
  private boolean altLogic = false;
  private boolean noiseLogic = false;
  private LEDPattern firePattern;
  private LEDPattern noisePattern;
  private boolean fireViews;
  private boolean skipUpdate = false;


  // HACK: Flip blue and green channels on real robot until we figure out 
  // the root cause of the sim/real color discrepancy
  public static Color getColor(Color color) {
    final boolean kFlipBlueAndGreen = Robot.isReal();
    return kFlipBlueAndGreen ? new Color(color.red, color.blue, color.green) : color;
  }

  public static final LEDPattern defaultPattern = LEDPattern.solid(getColor(Color.kGreen));

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
      altLogic = false;
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
     * @param deltaTimeSeconds The time since the last update in seconds.
     */
    public void update(double deltaTimeSeconds, LEDPattern firePattern) {
      if (durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;

        if (elapsedSeconds >= durationSeconds) {
          // pattern = defaultPattern;
          pattern = LEDPattern.kOff;
          fireAnimation(firePattern, true);
          durationSeconds = infiniteDurationSeconds;
          elapsedSeconds = 0.0;
        }
      }
      pattern.applyTo(bufferView);
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
  public LEDSubsystem(Elevator elevator) {
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

  public void resetToDefaultPattern() {
    setPattern(defaultPattern);
  }

  /**
   * Sets a pattern for an infinite duration.
   * @param pattern The LEDPattern to apply.
   */
  public void setPattern(LEDPattern pattern) {
    setPatternForDuration(pattern, Section.infiniteDurationSeconds);
  }

  /**
   * Sets a pattern for a duration in seconds
   * @param pattern the LEDPattern to apply
   * @param seconds the duration in seconds to set the pattern to before turning the strip off
   */
  public void setPatternForDuration(LEDPattern pattern, double seconds) {
    leftSide.setPattern(pattern, seconds);
    rightSide.setPattern(pattern, seconds);
    secretBuffer.setPattern(pattern, seconds);
  }

  /**
   * Turn all the LEDs off.
   */
  public void turnLEDsOff() {
    setPattern(LEDPattern.solid(Color.kBlack));
  }

  public void combinePatterns(LEDPattern leftPattern, LEDPattern rightPattern, LEDPattern secretPattern) {
    combinePatternsForDuration(leftPattern, rightPattern, secretPattern, Section.infiniteDurationSeconds);
  }

  public void combinePatternsForDuration(LEDPattern leftPattern, LEDPattern rightPattern, LEDPattern secretPattern, double seconds) {
    leftSide.setPattern(leftPattern, seconds);
    rightSide.setPattern(rightPattern, seconds);
    secretBuffer.setPattern(secretPattern, seconds);
  }

  public void enzoLEDS(enzoMap enzoMap, double seconds) {
    combinePatternsForDuration(enzoMap.getEnzoMap(), enzoMap.getEnzoMap(), defaultPattern, seconds);
  }

  // This uses sine waves and random number generation to create an interesting flickering fire pattern
  // It is overlaid on top of another preexisting pattern, and can be used on the strip or a whole or just its sides.
  public void fireAnimation (LEDPattern pattern) {
    altLogic = true;
    noiseLogic = false;
    firePattern = pattern;
    fireViews = false;
    pattern.applyTo(stripBuffer);
    for (int i = 0; i < 36; i ++) {
      if ((1.5 * (Math.sin(Math.random())) + (i/36.0)) > 1.3) {
        stripBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void fireAnimation (LEDPattern pattern, boolean bufferViews) {
    if (!skipUpdate) {
      altLogic = true;
      firePattern = pattern;
      fireViews = true;
      pattern.applyTo(leftSide.getBufferView());
      pattern.applyTo(rightSide.getBufferView());
      LEDPattern.solid(Color.kBlack).applyTo(secretBuffer.getBufferView());
      for (int i = 0; i < leftSide.getLength(); i++) {
        if ((1.5 * (Math.sin(Math.random())) + (i / 14.0)) > 1.3) {
          leftSide.getBufferView().setRGB(i, 0, 0, 0);
        }
      }
      for (int i = 0; i < rightSide.getLength(); i++) {
        if ((1.5 * (Math.sin(Math.random())) + (i / 16.0)) > 1.3) {
          rightSide.getBufferView().setRGB(i, 0, 0, 0);
        }
      }
      skipUpdate = true;
    } else {
      skipUpdate = false;
    }
  }

  // This activates a random noise function separately so that we can run the logic without having to deal with all this.
  public void activateRandomNoise(LEDPattern pattern) {
    altLogic = true;
    noiseLogic = true;
    noisePattern = pattern;
    noisePattern.applyTo(stripBuffer);
    fakeBuffer = new AddressableLEDBuffer(stripBuffer.getLength());
    randomNoiseAnimation(noisePattern);
  }

  // This function creates a fun random noise overlay that took way too long to make.
  // It, like the fire animation, uses LEDPatterns so you could do some cool stuff with it.
  private void randomNoiseAnimation(LEDPattern pattern) {
    altLogic = true;
    noiseLogic = true;
    int ledsOn = 0;
    pattern.applyTo(fakeBuffer);
    for (int i = 0; i < stripBuffer.getLength(); i ++) {
      if(!(stripBuffer.getRed(i) == 0 && stripBuffer.getGreen(i) == 0 && stripBuffer.getBlue(i) == 0)) {
        ledsOn += 1;
      }
    }
    for (int i = 0; i < stripBuffer.getLength(); i ++) {
      if ((ledsOn == 0) || Math.random() > 0.5) {
        stripBuffer.setRGB(i, fakeBuffer.getRed(i), fakeBuffer.getGreen(i), fakeBuffer.getBlue(i));
        ledsOn += 1;
      }
      if ((Math.random() * stripBuffer.getLength()) < (ledsOn) * 0.5) {
        stripBuffer.setRGB(i, 0, 0, 0);
        ledsOn -= 1;
      }
    }
  }

  @Override
  public void periodic() {
    if (altLogic) {
      if (noiseLogic) {
        randomNoiseAnimation(noisePattern);
      } else {
        if (fireViews) {
          fireAnimation(firePattern, fireViews);
        } else {
          fireAnimation(firePattern);
        }
      }
    } else {
      final double deltaTimeSeconds = 0.02; // TODO: Is there a way to ensure this is accurate even with overruns?
      leftSide.update(deltaTimeSeconds, firePattern);
      rightSide.update(deltaTimeSeconds, firePattern);
      secretBuffer.update(deltaTimeSeconds, firePattern);
    }
    lightStrip.setData(stripBuffer);
  }
}
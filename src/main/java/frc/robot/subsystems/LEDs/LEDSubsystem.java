// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDs.LEDPatterns.enzoMap;

import java.security.KeyFactory;
import java.util.List;
import java.util.Map;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED lightStrip;
  private final AddressableLEDBuffer stripBuffer;
  private boolean altLogic = false;
  private LEDPattern firePattern;
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
    public void update(double deltaTimeSeconds) {
      if (durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;

        if (elapsedSeconds >= durationSeconds) {
          // pattern = defaultPattern;
          pattern = LEDPattern.kOff;
          fireAnimation(LEDPatterns.theCoolerGreen, true);
          durationSeconds = infiniteDurationSeconds;
          elapsedSeconds = 0.0;
        }
      }
      pattern.applyTo(bufferView);
    }

    public AddressableLEDBufferView getBufferView() {
      return this.bufferView;
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

  // public void activateSecretPattern() {
  //   secretBuffer.setPattern(LEDPatterns.blinkyGreen);
  // }

  // public void deactivateSecretPattern() {
  //   secretBuffer.setPattern(defaultPattern); // TODO: might want to set this to whatever the other strips are set to
  // }

  public void enzoLEDS(enzoMap enzoMap, double seconds) {
    combinePatternsForDuration(enzoMap.getEnzoMap(), enzoMap.getEnzoMap(), defaultPattern, seconds);
  }

  // This works now!!!
  public void fireAnimation (LEDPattern pattern) {
    altLogic = true;
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
      for (int i = 0; i < 14; i++) {
        if ((1.5 * (Math.sin(Math.random())) + (i / 14.0)) > 1.3) {
          leftSide.getBufferView().setRGB(i, 0, 0, 0);
        }
      }
      for (int i = 0; i < 16; i++) {
        if ((1.5 * (Math.sin(Math.random())) + (i / 16.0)) > 1.3) {
          rightSide.getBufferView().setRGB(i, 0, 0, 0);
        }
      }
      skipUpdate = true;
    } else {
      skipUpdate = false;
    }
  }

  @Override
  public void periodic() {
    if (altLogic) {
      if (fireViews) {
        fireAnimation(firePattern, fireViews);
      } else {
        fireAnimation(firePattern);
      }
    } else {
      final double deltaTimeSeconds = 0.02; // TODO: Is there a way to ensure this is accurate even with overruns?
      leftSide.update(deltaTimeSeconds);
      rightSide.update(deltaTimeSeconds);
      secretBuffer.update(deltaTimeSeconds);
    }
    lightStrip.setData(stripBuffer);
  }
}

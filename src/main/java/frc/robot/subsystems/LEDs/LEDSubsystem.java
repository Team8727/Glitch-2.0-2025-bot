// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.AddressableLED;
import java.lang.reflect.InvocationTargetException;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDPatterns.enzoMap;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED lightStrip;
  private final AddressableLEDBuffer stripBuffer;
  private AddressableLEDBuffer fakeBuffer;
  private Method closingAnimation;
  // private boolean fireViews;
  // private boolean skipUpdate = false;

  public static final LEDPattern defaultPattern = LEDPattern.solid(Color.kGreen);

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
     * @param closingAnimation The method to call when the current pattern ends.
     * @param animPattern The pattern that the animation will overlay.
     * @param animBool A boolean that decides if the animation plays on the section.
     */

    public void update(double deltaTimeSeconds, Method closingAnimation, LEDPattern animPattern, boolean animBool) {
      if (durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;
      }
      if (elapsedSeconds >= durationSeconds && pattern == LEDPattern.kOff) {
        try {
          if (!animBool) {
            Section.this.setPattern(LEDPattern.solid(Color.kBlack), 0);
          } else {
            closingAnimation.invoke(LEDSubsystem.this, animPattern, Section.this);
          }
        } catch (IllegalAccessException | InvocationTargetException e) {
            e.printStackTrace();
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
  public LEDSubsystem() {
    // LED setup and port configuration
    lightStrip = new AddressableLED(5); // Correct PWM port
    stripBuffer = new AddressableLEDBuffer(36); // Correct LED count
    fakeBuffer = new AddressableLEDBuffer(stripBuffer.getLength());
    leftSide = new Section(stripBuffer, 0, 13);
    rightSide = new Section(stripBuffer, 35, 20);
    secretBuffer = new Section(stripBuffer, 14, 19);

    // if you want to use a different animation (like the random noise)
    // just change the string inside the getDeclaredMethod function
    try {
        closingAnimation = LEDSubsystem.class.getDeclaredMethod("fireAnimation", LEDPattern.class, Section.class);
    } catch (NoSuchMethodException e) {
        throw new RuntimeException("Method not found", e);
    }

    lightStrip.setLength(stripBuffer.getLength());

    lightStrip.setData(stripBuffer);
    lightStrip.start();
  }

  /**
   * Sets a pattern to each section of the strip for an infinite duration.
   * @param pattern The LEDPattern to apply.
   */
  public void setPattern(LEDPattern pattern) {
    setPatternForDuration(pattern, Section.infiniteDurationSeconds);
  }

  /**
   * Sets a pattern to each section of the strip for a duration in seconds
   * @param pattern the LEDPattern to apply
   * @param seconds the duration in seconds to set the pattern to before turning the strip off
   */
  public void setPatternForDuration(LEDPattern pattern, double seconds) {
    leftSide.setPattern(pattern, seconds);
    rightSide.setPattern(pattern, seconds);
    secretBuffer.setPattern(pattern, seconds);
  }

  /*
   * Sets different patterns to each Section of the strip for an infinite duration.
   */
  public void combinePatterns(LEDPattern leftPattern, LEDPattern rightPattern, LEDPattern secretPattern) {
    combinePatternsForDuration(leftPattern, rightPattern, secretPattern, Section.infiniteDurationSeconds);
  }

  /*
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

  /**
   * This uses sine waves and random number generation to create a flickering fire animation,
   * which is then overlaid on top of another preexisting pattern.
   * @param pattern The pattern that the fire overlays.
   */
  public void fireAnimation (LEDPattern pattern) {
    pattern.applyTo(stripBuffer);
    for (int i = 0; i < stripBuffer.getLength(); i ++) {
      if ((1.5 * (Math.sin(Math.random())) + (i / (double) stripBuffer.getLength())) > 1.3) {
        stripBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  /**
   * Uses some complicated logic to create a fire animation.
   * This version is special and only exists to prevent weird update issues. Please don't use it outside of this class.
   * @param pattern The pattern that the fire overlays.
   * @param bufferView The buffer view that the animation will play on.
   */
  public void fireAnimation (LEDPattern pattern, AddressableLEDBufferView bufferView) {
    pattern.applyTo(bufferView);
    for (int i = 0; i < bufferView.getLength(); i ++) {
      if ((1.5 * (Math.sin(Math.random())) + (i / (double) stripBuffer.getLength())) > 1.3) {
        bufferView.setRGB(i, 0, 0, 0);
      }
    }
  }

  /**
   * Uses some complicated logic to create a fire animation.
   * @param pattern The pattern that the fire overlays.
   * @param section The Section that the animation will play on.
   */
  public void fireAnimation (LEDPattern pattern, Section section) {
    fireAnimation(pattern, section.getBufferView());
  }

  /**
  * This activates a random noise function separately so that we can run the animation without having to deal with all the logic.
  * May be obsolete.
  * @param pattern The pattern that the random noise overlays.
  */ 
  public void activateRandomNoise(LEDPattern pattern) {
    pattern.applyTo(stripBuffer);
    if (fakeBuffer.getLength() != stripBuffer.getLength()) {
      fakeBuffer = new AddressableLEDBuffer(stripBuffer.getLength());
    }
    randomNoiseAnimation(pattern);
  }

  public void activateRandomNoise(LEDPattern pattern, Section section) {
    randomNoiseAnimation(pattern, section.getBufferView());
  }

  /**
  * This activates a random noise function separately so that we can run the animation without having to deal with all the logic.
  * May be obsolete.
  * @param pattern The pattern that the random noise overlays.
  */ 
  public void activateRandomNoise(LEDPattern pattern, AddressableLEDBufferView bufferView) {
    pattern.applyTo(bufferView);
    if (fakeBuffer.getLength() != bufferView.getLength()) {
      fakeBuffer = new AddressableLEDBuffer(bufferView.getLength());
    }
    randomNoiseAnimation(pattern, bufferView);
  }

  /** 
  * This function creates a fun random noise overlay that took way too long to make.
  * It, like the fire animation, uses LEDPatterns so you could do some cool stuff with it.
  *
  * This wasn't originally intended to work with multiple buffer views at once, so be warned.
  * @param pattern The pattern that the random noise overlays.
  */
  private void randomNoiseAnimation(LEDPattern pattern) {
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

  /** 
  * This function creates a fun random noise overlay that took way too long to make.
  * It, like the fire animation, uses LEDPatterns so you could do some cool stuff with it.
  *
  * This wasn't originally intended to work with multiple buffer views at once, so be warned.
  * @param pattern The pattern that the random noise overlays.
  * @param bufferView The buffer view that the animation will play on.
  */
  private void randomNoiseAnimation(LEDPattern pattern, AddressableLEDBufferView bufferView) {
    int ledsOn = 0;
    pattern.applyTo(fakeBuffer);
    for (int i = 0; i < bufferView.getLength(); i ++) {
      if(!(bufferView.getRed(i) == 0 && bufferView.getGreen(i) == 0 && bufferView.getBlue(i) == 0)) {
        ledsOn += 1;
      }
    }
    for (int i = 0; i < bufferView.getLength(); i ++) {
      if ((ledsOn == 0) || Math.random() > 0.5) {
        bufferView.setRGB(i, fakeBuffer.getRed(i), fakeBuffer.getGreen(i), fakeBuffer.getBlue(i));
        ledsOn += 1;
      }
      if ((Math.random() * bufferView.getLength()) < (ledsOn) * 0.5) {
        bufferView.setRGB(i, 0, 0, 0);
        ledsOn -= 1;
      }
    }
  }

  @Override
  public void periodic() {
    final double deltaTimeSeconds = 0.02; // TODO: Is there a way to ensure this is accurate even with overruns?

      leftSide.update(
        deltaTimeSeconds, 
        closingAnimation, 
        defaultPattern, 
        true);
      rightSide.update(
        deltaTimeSeconds, 
        closingAnimation, 
        defaultPattern, 
        true);
      secretBuffer.update(
        deltaTimeSeconds, 
        closingAnimation, 
        defaultPattern, 
        false);
    
    lightStrip.setData(stripBuffer);
  }
}
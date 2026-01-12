// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import Glitch.LEDs.GlitchLEDPatterns;
import Glitch.LEDs.LEDS;
import Glitch.LEDs.GlitchLEDPatterns.enzoMap;

public class LEDSubsystem2025 extends LEDS {

  /**
   * The pattern that will play once any other pattern stops running on the strip.
   */
  public static final LEDPattern defaultPattern = GlitchLEDPatterns.fire(GlitchLEDPatterns.theCoolerGreen);

  private final LEDS.Section leftSide;
  private final LEDS.Section rightSide;
  private final LEDS.Section secretBuffer;
  
  /** Creates a new LEDSubsystem. */
  private LEDSubsystem2025() {

    super(0, 36, 14, 6, -16);

    leftSide = getSections().get(0);
    leftSide.setBase(defaultPattern);
    rightSide = getSections().get(2);
    rightSide.setBase(defaultPattern);
    secretBuffer = getSections().get(1);
    secretBuffer.setBase(LEDPattern.kOff);    
  }

  static LEDSubsystem2025 instance = null;

  public static LEDSubsystem2025 getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem2025();
      return instance;
    } else {
      return instance;
    }
  }

  /**
   * Sets a pattern to each section of the strip for an infinite duration.
   * @param pattern The LEDPattern to apply.
   */
  public void setPattern(LEDPattern pattern) {
    setPatternForDuration(pattern, LEDS.infiniteDurationSeconds);
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

  /**
   * Sets different patterns to each Section of the strip for an infinite duration.
   */
  public void combinePatterns(LEDPattern leftPattern, LEDPattern rightPattern, LEDPattern secretPattern) {
    combinePatternsForDuration(leftPattern, rightPattern, secretPattern, LEDS.infiniteDurationSeconds);
  }

  /**
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
}
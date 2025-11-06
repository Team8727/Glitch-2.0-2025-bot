// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.Map;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

/** 
 * This class contains all the premade LED patterns used in the robot.
 * Feel free to add more!
 */
public class LEDPatterns {
      // Define LED Patterns
  public static final LEDPattern purple = LEDPattern.solid(Color.kPurple);
  
  // Rainbow pattern with a scrolling mask
  public static final LEDPattern rainbow = LEDPattern.rainbow(
    256, 
    256)
    .scrollAtRelativeSpeed(
      Percent.per(Second).of(15))
      .reversed()
      .mask(
        LEDPattern.steps(
          Map.of(
              0.0, Color.kWhite,
              0.25, Color.kBlack,
              0.75, Color.kWhite))
      .scrollAtRelativeSpeed(
        Percent.per(Second).of(20)));

  /** Blue gradient pattern with a scrolling mask (its not actually blue i lied) */
  public static final LEDPattern blue =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kGreen)
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  // Green to purple gradient pattern
  public static final LEDPattern ace =
      LEDPattern.gradient(GradientType.kContinuous, Color.kPurple, Color.kGreen)
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  public static final LEDPattern green = LEDPattern.solid(Color.kGreen);

  public static final LEDPattern blinkyGreen = LEDPattern.solid(Color.kGreen).blink(Second.of(0.1));
  
  public static final LEDPattern theCoolerGreen = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    Color.kGreen,
    Color.kForestGreen,
    Color.kDarkGreen)
    .scrollAtRelativeSpeed(Percent.per(Second).of(25 * Math.sin(Math.random() * 3)));

  public static final LEDPattern darkGreen = LEDPattern.gradient(
    GradientType.kDiscontinuous,
    Color.kGreen,
    Color.kDarkGreen);

  /**
   * Creates a linear progress bar overlay on top of the given pattern.
   * @param pattern The pattern the progress bar will overlay
   * @param currentProgress The current progress value (e.g., current height of the elevator)
   * @param maxProgress The maximum progress value (e.g., maximum height of the elevator)
   */
  public static LEDPattern linearProgress(LEDPattern pattern, double currentProgress, double maxProgress) {
    return pattern.mask(LEDPattern.progressMaskLayer(() -> currentProgress / maxProgress));
  }

  // Elevator progress pattern (2025)
  public static final LEDPattern elevatorProgress = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    Color.kGreen, 
    Color.kYellow, 
    Color.kOrange, 
    Color.kRed);
  // Coral pickup pattern (2025)
  public static final LEDPattern coralPickup = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    Color.kGreen, 
    Color.kPink, 
    Color.kYellow, 
    Color.kRed)
      .blink(Second.of(0.5));

  // Algae pickup pattern
  public static final LEDPattern algaePickup = LEDPattern.gradient(
    GradientType.kDiscontinuous,
    Color.kGreen,
    Color.kPurple,
    Color.kOrange,
    Color.kRed)
      .blink(Second.of(0.5));

  public static final LEDPattern fire = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    Color.kWhite,
    Color.kYellow,
    Color.kOrange,
    Color.kRed);

  public static enum enzoMap {
    NORMAL(Map.of(
    0.0, Color.kBlack,
    0.08, Color.kGreen,
    0.48, Color.kWhite,
    0.56, Color.kBlack,
    0.72, Color.kWhite,
    0.80, Color.kGreen,
    0.92, Color.kBlack)),

    STARTLED(Map.of(
    0.0, Color.kBlack,
    0.08, Color.kGreen,
    0.32, Color.kWhite,
    0.48, Color.kBlack,
    0.64, Color.kWhite,
    0.80, Color.kGreen,
    0.92, Color.kBlack)),

    DISAPPOINTED(Map.of(
    0.0, Color.kBlack,
    0.08, Color.kGreen,
    0.48, Color.kWhite,
    0.56, Color.kBlack,
    0.72, Color.kGreen,
    0.92, Color.kBlack)),

    HAPPY(Map.of(
    0.0, Color.kBlack,
    0.08, Color.kGreen,
    0.56, Color.kBlack,
    0.72, Color.kWhite,
    0.80, Color.kGreen,
    0.92, Color.kBlack)),

    BLINKING(Map.of(
    0.0, Color.kBlack,
    0.08, Color.kGreen,
    0.92, Color.kBlack));

    private final Map<Number, Color> map;

    enzoMap(Map<Number, Color> map) {
      this.map = map;
    }

    public LEDPattern getEnzoMap() {
      return LEDPattern.steps(this.map);
    }
  }

  /** 
  * Creates a new LEDPatterns.
  */
  public LEDPatterns() {}
}

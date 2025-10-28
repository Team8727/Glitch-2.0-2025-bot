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
    Elevator m_elevator;
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

  // Blue gradient pattern with a scrolling mask
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
  * Elevator progress bar pattern
  * IMPORTANT: This will only work if you provided an Elevator object to the constructor of this class.
  * ALSO IMPORTANT: This pattern was based off of the elevator for the 2025 bot so it will take tinkering to get to work for your bot.
  */ 
  public final LEDPattern elevatorProgress = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    Color.kGreen, 
    Color.kYellow, 
    Color.kOrange, 
    Color.kRed)
  .mask(LEDPattern.progressMaskLayer(
    () -> m_elevator.getElevatorHeight() / Elevator.ElevatorPosition.L4.getOutputRotations()));
  // Coral pickup pattern
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
  * This version of LEDPatterns just won't have the elevator progress bar.
  */
  public LEDPatterns() {}

  /** 
  * Creates a new LEDPatterns. 
  * @param elevator This parameter is used for a specific LED pattern that displays the elevator's progress. It is not used for any other patterns.
  * This class does use other values from the elevator, including some in Constants.kElevator, but they aren't necessary for anything other than the elevator pattern.
  */
  public LEDPatterns(Elevator elevator) {
    m_elevator = elevator;
  }
}

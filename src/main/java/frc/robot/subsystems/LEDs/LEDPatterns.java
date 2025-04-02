// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDs.LEDSubsystem;

/** Add your docs here. */
public class LEDPatterns {
    Elevator m_elevator;
      // Define LED Patterns
  public static final LEDPattern purple = LEDPattern.solid(LEDSubsystem.getColor(Color.kPurple));
  
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
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, LEDSubsystem.getColor(Color.kBlue), LEDSubsystem.getColor(Color.kGreen))
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  // Green to purple gradient pattern
  public static final LEDPattern ace =
      LEDPattern.gradient(GradientType.kContinuous, LEDSubsystem.getColor(Color.kPurple), LEDSubsystem.getColor(Color.kGreen))
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  public static final LEDPattern green = LEDPattern.solid(LEDSubsystem.getColor(Color.kGreen));

  public static final LEDPattern blinkyGreen = LEDPattern.solid(LEDSubsystem.getColor(Color.kGreen)).blink(Second.of(0.1));
  
  public static final LEDPattern theCoolerGreen = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    LEDSubsystem.getColor(Color.kGreen),
    LEDSubsystem.getColor(Color.kForestGreen),
    LEDSubsystem.getColor(Color.kDarkGreen));
    // .scrollAtRelativeSpeed(Percent.per(Second).of(25));

  public static final LEDPattern darkGreen = LEDPattern.gradient(
    GradientType.kDiscontinuous,
    LEDSubsystem.getColor(Color.kGreen),
    LEDSubsystem.getColor(Color.kDarkGreen)
  );
  
  // Elevator progress bar pattern
  public final LEDPattern elevatorProgress = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    LEDSubsystem.getColor(Color.kGreen), 
    LEDSubsystem.getColor(Color.kYellow), 
    LEDSubsystem.getColor(Color.kOrange), 
    Color.kRed)
  .mask(LEDPattern.progressMaskLayer(
    () -> m_elevator.getElevatorHeight() / kElevator.ElevatorPosition.L4.getOutputRotations()));
  // Coral pickup pattern
  public static final LEDPattern coralPickup = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    LEDSubsystem.getColor(Color.kGreen), 
    LEDSubsystem.getColor(Color.kPink), 
    LEDSubsystem.getColor(Color.kYellow), 
    Color.kRed)
      .blink(Second.of(0.5));

  // Algae pickup pattern
  public static final LEDPattern algaePickup = LEDPattern.gradient(
    GradientType.kDiscontinuous,
    LEDSubsystem.getColor(Color.kGreen),
    LEDSubsystem.getColor(Color.kPurple),
    LEDSubsystem.getColor(Color.kOrange),
    Color.kRed)
      .blink(Second.of(0.5));

  public static final LEDPattern fire = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    Color.kWhite,
    LEDSubsystem.getColor(Color.kYellow),
    LEDSubsystem.getColor(Color.kOrange),
    Color.kRed);

  public static enum enzoMap {
    NORMAL(Map.of(
    0.0, Color.kBlack,
    0.08, LEDSubsystem.getColor(Color.kGreen),
    0.48, Color.kWhite,
    0.56, Color.kBlack,
    0.72, Color.kWhite,
    0.80, LEDSubsystem.getColor(Color.kGreen),
    0.92, Color.kBlack)),

    STARTLED(Map.of(
    0.0, Color.kBlack,
    0.08, LEDSubsystem.getColor(Color.kGreen),
    0.32, Color.kWhite,
    0.48, Color.kBlack,
    0.64, Color.kWhite,
    0.80, LEDSubsystem.getColor(Color.kGreen),
    0.92, Color.kBlack)),

    DISAPPOINTED(Map.of(
    0.0, Color.kBlack,
    0.08, LEDSubsystem.getColor(Color.kGreen),
    0.48, Color.kWhite,
    0.56, Color.kBlack,
    0.72, LEDSubsystem.getColor(Color.kGreen),
    0.92, Color.kBlack)),

    HAPPY(Map.of(
    0.0, Color.kBlack,
    0.08, LEDSubsystem.getColor(Color.kGreen),
    0.56, Color.kBlack,
    0.72, Color.kWhite,
    0.80, LEDSubsystem.getColor(Color.kGreen),
    0.92, Color.kBlack)),

    BLINKING(Map.of(
    0.0, Color.kBlack,
    0.08, LEDSubsystem.getColor(Color.kGreen),
    0.92, Color.kBlack));

    private final Map<Number, Color> map;

    enzoMap(Map<Number, Color> map) {
      this.map = map;
    }

    public LEDPattern getEnzoMap() {
      return LEDPattern.steps(this.map);
    }
  }

  public LEDPatterns(Elevator elevator) {
    m_elevator = elevator;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Glitch.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

/** 
 * This class contains all the premade LED patterns used in the robot.
 * Feel free to add more!
 */
public class GlitchLEDPatterns {
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

  // LEDPattern modifying methods is a mouthful!

  /**
   * This pattern creates a fire overlay that makes the given pattern look like it's made of fire.
   * @param pattern The pattern that the fire overlay applies to.
   */
  public static LEDPattern fire(LEDPattern pattern, double updateTime) {
    return (reader, writer) -> {

      double randomOffset = Math.random() * 1.33;
      long actualUpdateTime = (long) Seconds.of(updateTime).in(Microseconds);
      long updateLimit = (long) Seconds.of(0.039).in(Microseconds);

      if (RobotController.getTime() % actualUpdateTime < updateLimit 
          && RobotController.getTime() % actualUpdateTime > 0) {

        pattern.applyTo(reader, writer);

        reader.forEach( (index, red, green, blue) -> {
          if ((1.5 * (Math.sin(randomOffset - Math.random())) + (index / (double) reader.getLength())) > 1.275) {
            writer.setRGB(index, 0, 0, 0);
          } else {
            writer.setRGB(index, red, green, blue);
          }
        });
      }
    };
  }

  /**
   * This pattern creates a fire overlay that makes the given pattern look like it's made of fire.
   * @param pattern The pattern that the fire overlay applies to.
   */
  public static LEDPattern fire(LEDPattern pattern) {
    return fire(pattern, 0.07);
  }

  /** 
  * This pattern creates a fun random noise overlay that took way too long to make.
  * @param pattern The pattern that the random noise overlays.
  */
  public static LEDPattern randomNoise(LEDPattern pattern, double updateTime) {
    return (reader, writer) -> {

      int ledsOn = 0;
      long actualUpdateTime = (long) Seconds.of(updateTime).in(Microseconds);
      long updateLimit = (long) Seconds.of(0.039).in(Microseconds);

      if (RobotController.getTime() % actualUpdateTime < updateLimit 
          && RobotController.getTime() % actualUpdateTime > 0) {
            pattern.applyTo(reader, writer);
            for (int i = 0; i < reader.getLength(); i ++) {
              if(!(reader.getRed(i) == 0 && reader.getGreen(i) == 0 && reader.getBlue(i) == 0)) {
                ledsOn += 1;
              }
            } // TODO: Make sure that this works the way you want it and change the default pattern back to normal once you're done
            for (int i = 0; i < reader.getLength(); i ++) {
              double cycleRandom = Math.sin(Math.random());
              if ((ledsOn < reader.getLength()/2) || cycleRandom >= 0.4) {
                writer.setRGB(i, reader.getRed(i), reader.getGreen(i), reader.getBlue(i));
                ledsOn += 1;
              }
              if (reader.getLength() - ledsOn < reader.getLength()/8 || cycleRandom < 0.4) {
                writer.setRGB(i, 0, 0, 0);
                ledsOn -= 1;
              }
            }
          }
    };
  }

  public static LEDPattern randomNoise(LEDPattern pattern) {
    return randomNoise(pattern, 0.05);
  }

  /** 
  * Creates a new LEDPatterns.
  */
  public GlitchLEDPatterns() {}
}

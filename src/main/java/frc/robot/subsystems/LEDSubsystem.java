// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

import frc.robot.Robot;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED lightStrip;
  private final AddressableLEDBuffer stripBuffer;

  // HACK: Flip blue and green channels on real robot until we figure out 
  // the root cause of the sim/real color discrepancy
  private static Color getColor(Color color) {
    final boolean kFlipBlueAndGreen = Robot.isReal();
    return kFlipBlueAndGreen ? new Color(color.red, color.blue, color.green) : color;
  }

  private static final LEDPattern defaultPattern = LEDPattern.solid(getColor(Color.kGreen));

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
     * @param deltaTimeSeconds The time since the last update in seconds.
     */
    public void update(double deltaTimeSeconds) {
      if (durationSeconds != infiniteDurationSeconds) {
        elapsedSeconds += deltaTimeSeconds;

        if (elapsedSeconds >= durationSeconds) {
          pattern = defaultPattern;
          durationSeconds = infiniteDurationSeconds;
          elapsedSeconds = 0.0;
        }
      }

      pattern.applyTo(bufferView);
    }
  }

  private final Section leftSide;
  private final Section rightSide;
  private final Section secretBuffer;

  private Elevator m_elevator;

  private boolean triggerSecretPattern = false;

  // Define LED Patterns
  public static final LEDPattern purple = LEDPattern.solid(getColor(Color.kPurple));
  
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
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, getColor(Color.kBlue), getColor(Color.kGreen))
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  // Green to purple gradient pattern
  public static final LEDPattern ace =
      LEDPattern.gradient(GradientType.kContinuous, getColor(Color.kGreen), getColor(Color.kPurple))
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  public static final LEDPattern green = LEDPattern.solid(getColor(Color.kGreen));

  public static final LEDPattern blinkyGreen = LEDPattern.solid(getColor(Color.kGreen)).blink(Second.of(0.1));
  // Elevator progress bar pattern
  public static final LEDPattern elevatorProgressBase = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    getColor(Color.kGreen), 
    getColor(Color.kYellow), 
    getColor(Color.kOrange), 
    Color.kRed);
  public final LEDPattern elevatorProgressMap = LEDPattern.progressMaskLayer(
    () -> m_elevator.getElevatorHeight() / kElevator.ElevatorPosition.L4.getOutputRotations());
  public final LEDPattern elevatorProgress = elevatorProgressBase.mask(elevatorProgressMap);
  // Coral pickup pattern
  public static final LEDPattern coralPickup = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    getColor(Color.kGreen), 
    getColor(Color.kPink), 
    getColor(Color.kYellow), 
    Color.kRed)
      .blink(Second.of(0.5));

  // Algae pickup pattern
  public static final LEDPattern algaePickup = LEDPattern.gradient(
    GradientType.kDiscontinuous,
    getColor(Color.kGreen),
    getColor(Color.kPurple),
    getColor(Color.kOrange),
    Color.kRed)
      .blink(Second.of(0.5));
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(Elevator elevator) {
    // LED setup and port configuration
    lightStrip = new AddressableLED(5); // Correct PWM port
    stripBuffer = new AddressableLEDBuffer(36); // Correct LED count
    leftSide = new Section(stripBuffer, 0, 14);
    rightSide = new Section(stripBuffer, 36, 20);
    secretBuffer = new Section(stripBuffer, 15, 19);

    lightStrip.setLength(stripBuffer.getLength());

    lightStrip.setData(stripBuffer);
    lightStrip.start();

    m_elevator = elevator;
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

  public void combinePatterns(LEDPattern leftPattern, LEDPattern rightPattern) {
    combinePatternsForDuration(leftPattern, rightPattern, Section.infiniteDurationSeconds);
  }

  public void combinePatternsForDuration(LEDPattern leftPattern, LEDPattern rightPattern, double seconds) {
    leftSide.setPattern(leftPattern, seconds);
    rightSide.setPattern(rightPattern, seconds);
  }

  public void activateSecretPattern() {
    secretBuffer.setPattern(blinkyGreen);
  }

  public void deactivateSecretPattern() {
    secretBuffer.setPattern(defaultPattern); // TODO: might want to set this to whatever the other strips are set to
  }

  @Override
  public void periodic() {
    final double deltaTimeSeconds = 0.02; // TODO: Is there a way to ensure this is accurate even with overruns?
    leftSide.update(deltaTimeSeconds);
    rightSide.update(deltaTimeSeconds);
    secretBuffer.update(deltaTimeSeconds);

    lightStrip.setData(stripBuffer);
  }
}

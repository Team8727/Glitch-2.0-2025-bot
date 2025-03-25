package frc.robot.utilities.BaseSystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utilities.BaseSystems.Motors.Motor;
import frc.robot.utilities.NetworkTableLogger;

public abstract class Pivot extends SubsystemBase {

  private final Motor motor;

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0,0);

  private final ArmFeedforward pivotFeedforward;

  public final NetworkTableLogger logger;

  private final double dt;
  private final double zeroedAngelFromHorizontal;

  private final double allowedError;

  /**
   * Creates a new Pivot.
   *
   * @param motor The motor to use for the pivot.
   * @param zeroedAngelFromHorizontal The angle from horizontal to zero the pivot at
   * @param maxVelocity The maximum velocity of the pivot
   * @param maxAcceleration The maximum acceleration of the pivot
   * @param allowedError The allowed error for the pivot in degrees
   * @param ks The static gain of the pivot
   * @param kg The gravity gain of the pivot
   * @param kv The velocity gain of the pivot
   * @param ka The acceleration gain of the pivot
   * @param dt The time step for the profile
   */
  public Pivot(
    Motor motor,
    double zeroedAngelFromHorizontal,
    double maxVelocity,
    double maxAcceleration,
    double allowedError,
    double ks,
    double kg,
    double kv,
    double ka,
    double dt) {
    logger = new NetworkTableLogger(this.getName());

    profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    pivotFeedforward =  new ArmFeedforward(ks, kg, kv, ka ,dt);
    this.dt = dt;

    this.zeroedAngelFromHorizontal = zeroedAngelFromHorizontal;

    this.motor = motor;

    this.allowedError = allowedError/360;
  }

  /**
   * Creates a new Pivot.
   *
   * @param motor The motor to use for the pivot.
   * @param zeroedAngelFromHorizontal The angle from horizontal to zero the pivot at
   * @param maxVelocity The maximum velocity of the pivot
   * @param maxAcceleration The maximum acceleration of the pivot
   * @param allowedError The allowed error for the pivot in degrees
   * @param ks The static gain of the pivot
   * @param kg The gravity gain of the pivot
   * @param kv The velocity gain of the pivot
   * @param ka The acceleration gain of the pivot
   */
  public Pivot(
    Motor motor,
    double zeroedAngelFromHorizontal,
    double maxVelocity,
    double maxAcceleration,
    double allowedError,
    double ks,
    double kg,
    double kv,
    double ka) {
    this(motor, zeroedAngelFromHorizontal, maxVelocity, allowedError, maxAcceleration, ks, kg, kv, ka, 0.02);
  }

  /**
   * Creates a new Pivot.
   *
   * @param motor The motor to use for the pivot.
   * @param zeroedAngelFromHorizontal The angle from horizontal to zero the pivot at
   * @param maxVelocity The maximum velocity of the pivot
   * @param maxAcceleration The maximum acceleration of the pivot
   * @param allowedError The allowed error for the pivot in degrees
   */
  public Pivot(
    Motor motor,
    double zeroedAngelFromHorizontal,
    double maxVelocity,
    double maxAcceleration,
    double allowedError) {
    this(motor, zeroedAngelFromHorizontal, maxVelocity, maxAcceleration, allowedError, 0, 0, 0, 0);
  }

  /**
   * Sets the pivot position using a trapezoidal profile.
   *
   * @param angleDegrees The angle in degrees to set the pivot to.
   */
  public void setPosition(double angleDegrees) {
    double position = angleDegrees / 360;
    goal = new TrapezoidProfile.State(position, 0);
    setpoint = new TrapezoidProfile.State(motor.getPosition(), 0);
  }

  /**
   * Creates a command to set the pivot position to the specified angle in degrees.
   *
   * @param angleDegrees The angle in degrees to set the pivot to.
   * @return A command that sets the pivot position and waits until the pivot reaches the setpoint.
   */
  public Command setPositionCommand(double angleDegrees) {
    return new InstantCommand(() -> setPosition(angleDegrees))
      .andThen(new WaitUntilCommand(this::isAtSetpoint));
  }

  // set pivot position
  private void setMotorFFAndPIDPosition(double Position) {
    motor.setPosition(
      Position,
      pivotFeedforward.calculateWithVelocities(
        motor.getPosition() + (zeroedAngelFromHorizontal/360),//TODO: convert to radians per second
        motor.getVelocity(),
        setpoint.velocity));
  }

  /**
   * Checks if the pivot is at the setpoint.
   *
   * @return True if the pivot is within 0.01 units of the goal position, false otherwise.
   */
  public boolean isAtSetpoint() {
    return Math.abs(motor.getPosition() - goal.position) < allowedError;
  }

  /**
   * Gets the current position of the pivot.
   *
   * @return The current position of the pivot.
   */
  public double getPosition() {
    return motor.getPosition();
  }

  /**
   * Gets the current applied to the motor.
   *
   * @return The current applied to the motor.
   */
  public double getAppliedCurrent() {
    return motor.getCurrent();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    logger.logDouble("Pivot Position", motor.getPosition() * 360);

    setpoint = profile.calculate(dt, setpoint, goal);

    setMotorFFAndPIDPosition(setpoint.position);
  }
}

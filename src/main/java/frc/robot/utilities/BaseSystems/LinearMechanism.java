package frc.robot.utilities.BaseSystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.utilities.BaseSystems.Motors.Motor;
import frc.robot.utilities.NetworkTableLogger;

public abstract class LinearMechanism extends SubsystemBase {

  private final Motor motor;

  private final NetworkTableLogger logger;

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private final ElevatorFeedforward feedforward;

  private final double dt;

  private final double allowedError;

  private final double rotationsToMeters;

  /**
   * Creates a new LinearMechanism.
   *
   * @param motor The motor to use for the mechanism.
   * @param maxVelocity The maximum velocity of the mechanism
   * @param maxAcceleration The maximum acceleration of the mechanism
   * @param allowedError The allowed error for the mechanism in meters
   * @param rotationsToMeters The conversion factor from rotations to meters
   * @param ks The static gain of the mechanism
   * @param kg The gravity gain of the mechanism
   * @param kv The velocity gain of the mechanism
   * @param ka The acceleration gain of the mechanism
   * @param dt The time step for the profile
   */
  public LinearMechanism(
    Motor motor,
    double maxVelocity,
    double maxAcceleration,
    double allowedError,
    double rotationsToMeters,
    double ks,
    double kg,
    double kv,
    double ka,
    double dt
    ) {

    logger = new NetworkTableLogger(this.getName());

    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    this.motor = motor;

    feedforward = new ElevatorFeedforward(ks, kg, kv, ka, dt);
    this.dt = dt;

    this.allowedError = allowedError;

    this.rotationsToMeters = rotationsToMeters;
  }

  /**
   * Sets the position of the mechanism.
   *
   * @param position The desired position in meters.
   */
  public void setPosition(double position) {
    goal = new TrapezoidProfile.State(position, 0);
    setpoint = new TrapezoidProfile.State(motor.getPosition(), 0);
  }

  /**
   * Creates a command to set the position of the mechanism.
   *
   * @param position The desired position in meters.
   * @return A command that sets the position and waits until the setpoint is reached.
   */
  public Command setPositionCommand(double position) {
    return new InstantCommand(() -> setPosition(position))
      .andThen(new WaitUntilCommand(this::isAtSetpoint));
  }

  // set position
  private void setMotorFFAndPIDPosition(double nextPos) {
    motor.setPosition(//TODO: convert to meters per second
      nextPos,
      feedforward.calculateWithVelocities(motor.getVelocity(), setpoint.velocity));
  }

  /**
   * Checks if the mechanism is at the setpoint.
   *
   * @return True if the mechanism is at the setpoint, false otherwise.
   */
  public boolean isAtSetpoint() {
    return Math.abs(motor.getPosition() - goal.position) < allowedError;
  }

  /**
   * Gets the current position of the mechanism.
   *
   * @return The current position in meters.
   */
  public double getPosition() {
    return motor.getPosition();
  }

  /**
   * Gets the current applied to the motor.
   *
   * @return The current applied to the motor in amperes.
   */
  public double getAppliedCurrent() {
    return motor.getCurrent();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    logger.logDouble("setpoint", setpoint.position);
    logger.logDouble("actualPosition", motor.getPosition());
    logger.logDouble("goal", goal.position);

    setpoint = profile.calculate(dt, setpoint, goal);

    setMotorFFAndPIDPosition(setpoint.position);
  }
}

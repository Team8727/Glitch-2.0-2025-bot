package Glitch.Lib.Motors;

public interface Motor {
  /**
   * Sets the motor to a given velocity.
   *
   * @param speed The speed to set the motor to.
   */
  void setVelocity(double speed);

  /**
   * Sets the motor to a percent output -1 to 1.
   *
   * @param dutyCycle The percentage to set the motor to.
   */
  void setDutyCycle(double dutyCycle);

  /**
   * Sets the motor to a given position.
   *
   * @param position The position to set the motor to.
   */
  void setPosition(double position, double feedforward);

  /**
   * Sets the motor to a given position.
   *
   * @param position The position to set the motor to.
   */
  void setPosition(double position);

  /**
   * Gets the current position of the motor.
   *
   * @return The current position of the motor.
   */
  double getPosition();

  /**
   * Gets the current velocity of the motor.
   *
   * @return The current velocity of the motor.
   */
  double getCurrent();

  /**
   * Gets the current velocity of the motor.
   *
   * @return The current velocity of the motor.
   */
  double getVelocity();

  /**
   * Gets the state of the forward limit switch.
   *
   * @return The state of the limit switch.
   */
  boolean getForwardLimitSwitch();

  /**
   * Gets the state of the reverse limit switch.
   *
   * @return The state of the limit switch.
   */
  boolean getReverseLimitSwitch();
}
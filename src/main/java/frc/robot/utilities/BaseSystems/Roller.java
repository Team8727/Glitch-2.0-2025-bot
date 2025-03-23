package frc.robot.utilities.BaseSystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BaseSystems.Motors.Motor;
import frc.robot.utilities.BaseSystems.Motors.SparkMaxMotor;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator;

import java.util.Set;

public abstract class Roller extends SubsystemBase {

  private final Motor motor;

  public final NetworkTableLogger logger;

  /**
   * Creates a new Roller.
   *
   * @param config The configuration for the SparkMax motor
   * @param CANID The CAN ID of the motor
   */
  public Roller(
      Motor motor) {
    logger = new NetworkTableLogger(this.getName());
    this.motor = motor;
  }

  /**
   * Sets the speed of the roller motor in duty cycle mode.
   *
   * @param speed The speed to set the motor to, as a percentage (0.0 to 1.0)
   */
  public void setSpeedDutyCycle(double speed) {
    motor.setDutyCycle(speed);
  }

  /**
   * Sets the speed of the roller motor in velocity mode.
   *
   * @param speed The speed to set the motor to, in RPM
   */
  public void setSpeedVelocity(double speed) {
    motor.setVelocity(speed);
  }

  /**
   * Sets the speed of the roller motor in position mode.
   *
   * @param position The position to set the motor to, in encoder rotations
   */
  public void setPosition(double position) {
    motor.setPosition(position, 0);
  }

  /**
   * gets the position of the roller motor in encoder rotations.
   *
   * @return The position of the motor in encoder rotations
   */
  public double getPosition() {
    return  motor.getPosition();
  }

  /**
   * gets the velocity of the roller motor in RPM.
   *
   * @return The velocity of the motor in RPM
   */
  public double getVelocity() {
    return motor.getVelocity();
  }

  /**
   * gets the current of the roller motor in Amps.
   *
   * @return The current of the motor in Amps
   */
  public double getCurrent() {
    return motor.getCurrent();
  }
}

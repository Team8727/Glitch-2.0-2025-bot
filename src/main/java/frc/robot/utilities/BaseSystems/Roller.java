package frc.robot.utilities.BaseSystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator;

import java.util.Set;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public abstract class Roller extends SubsystemBase {

  private final SparkMax motor;
  private final SparkClosedLoopController motorController;

  public final NetworkTableLogger logger;

  /**
   * Creates a new Roller.
   *
   * @param config The configuration for the SparkMax motor
   * @param CANID The CAN ID of the motor
   */
  public Roller(
      SparkMaxConfig config,
      int CANID) {
    logger = new NetworkTableLogger(this.getName());
    motor =
      getSparkMax(
        CANID,
        SparkLowLevel.MotorType.kBrushless,
        false,
        Set.of(),
        Set.of(
          SparkConfigurator.LogData.POSITION,
          SparkConfigurator.LogData.VELOCITY,
          SparkConfigurator.LogData.VOLTAGE,
          SparkConfigurator.LogData.CURRENT));

    motor.configure(
      config,
      SparkBase.ResetMode.kNoResetSafeParameters,
      SparkBase.PersistMode.kNoPersistParameters);

    motorController = motor.getClosedLoopController();
  }

  /**
   * Sets the speed of the roller motor in duty cycle mode.
   *
   * @param speed The speed to set the motor to, as a percentage (0.0 to 1.0)
   */
  public void setSpeedDutyCycle(double speed) {
    motorController.setReference(speed, SparkBase.ControlType.kDutyCycle);
  }

  /**
   * Sets the speed of the roller motor in velocity mode.
   *
   * @param speed The speed to set the motor to, in RPM
   */
  public void setSpeedVelocity(double speed) {
    motorController.setReference(speed, SparkBase.ControlType.kVelocity);
  }

  /**
   * Sets the speed of the roller motor in position mode.
   *
   * @param position The position to set the motor to, in encoder rotations
   */
  public void setPosition(double position) {
    motorController.setReference(position, SparkBase.ControlType.kPosition);
  }

}

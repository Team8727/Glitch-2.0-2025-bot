package frc.robot.utilities.BaseSystems.Motors;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utilities.SparkConfigurator;


import java.util.Set;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public class SparkMaxMotor implements Motor{
  private final SparkMax motor;
  private final SparkClosedLoopController motorController;
  private final boolean hasAbsoluteEncoder;

  public SparkMaxMotor(SparkMaxConfig config, int CANID, FeedbackSensor encoderType) {
    motor = getSparkMax(
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

    hasAbsoluteEncoder = encoderType == FeedbackSensor.kAbsoluteEncoder;
  }

  @Override
  public void setVelocity(double velocity) {
    motorController.setReference(velocity, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motorController.setReference(dutyCycle, SparkBase.ControlType.kDutyCycle);
  }

  @Override
  public void setPosition(double position, double feedforward) {
    motorController.setReference(
      position,
      SparkBase.ControlType.kPosition,
      ClosedLoopSlot.kSlot0,
      feedforward);
  }

  @Override
  public void setPosition(double position) {
    motorController.setReference(
      position,
      SparkBase.ControlType.kPosition);
  }

  @Override
  public double getPosition() {
    if (hasAbsoluteEncoder) {
      return motor.getAbsoluteEncoder().getPosition();
    } else {
      return motor.getEncoder().getPosition();
    }
  }

  @Override
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  @Override
  public double getVelocity() {
    if (hasAbsoluteEncoder) {
      return motor.getAbsoluteEncoder().getVelocity();
    } else {
      return motor.getEncoder().getVelocity();
    }
  }
}
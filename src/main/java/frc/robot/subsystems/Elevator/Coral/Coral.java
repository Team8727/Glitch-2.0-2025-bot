// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.Coral;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCoral;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public class Coral extends SubsystemBase {
  private final SparkMax backMotor;
  private final SparkMax frontMotor;
  private final SparkMaxConfig frontConfig;
  private final SparkLimitSwitch frontCoralSensor;
  private final SparkLimitSwitch backCoralSensor;
  public boolean elevatorUp = false;

  private final NetworkTableLogger logger = new NetworkTableLogger(this.getSubsystem());

  /** Creates a new Coral. */
  public Coral() {
    backMotor =
        getSparkMax(
            kCoral.intakeRollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));

    SparkMaxConfig backConfig = new SparkMaxConfig();
    backConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .closedLoop
        .pid(0.5, 0, 0);

    backMotor.configure(
      backConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    frontMotor =
        getSparkMax(
            kCoral.outtakeRollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));

    frontConfig = new SparkMaxConfig();
    frontConfig
        .smartCurrentLimit(40) 
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .closedLoop
        .velocityFF(0) 
        .pid(0.5, 0, 0);

    frontMotor.configure(
        frontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    frontCoralSensor = frontMotor.getForwardLimitSwitch();
    backCoralSensor = frontMotor.getReverseLimitSwitch();
  }

  public boolean getFrontCoralSensor() {
    return frontCoralSensor.isPressed();
  }

  public boolean getBackCoralSensor() {
    return backCoralSensor.isPressed();
  }

  public void setIntakeSpeedDuty(double speed) {
    backMotor.getClosedLoopController().setReference(speed, ControlType.kDutyCycle);
  }

  public void setOuttakeSpeedDuty(double speed) {
    frontMotor.getClosedLoopController().setReference(speed, ControlType.kDutyCycle);
  }

  public void setOutakePos(double position) {
    frontMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void setIntakePos(double position) {
    backMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public double getOuttakeRotations() {
    return frontMotor.getEncoder().getPosition();
  }
  
  public void stopDeployer() {
    backMotor.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
    frontMotor.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
  }

  public void holdPosition() {
    frontMotor.getClosedLoopController().setReference(
      frontMotor.getEncoder().getPosition(), 
      ControlType.kPosition);
  }

  public void setSpeed(double speed) {
    frontMotor.getClosedLoopController().setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // if (getFrontCoralSensor() && !elevatorUp) {
    //   setOuttakeSpeedDuty(-.05);
    // } else if (!elevatorUp){
    //   setOuttakeSpeedDuty(0);
    // }
    // This method will be called once per scheduler run

    // PLEASE NEVER CHANGE THIS
    logger.logBoolean("Back Coral Sensor", getBackCoralSensor());
    logger.logBoolean("Front Coral Sensor", getFrontCoralSensor());
  }
}

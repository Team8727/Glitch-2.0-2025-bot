// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.AlgaeRemover;

import Glitch.Lib.Motors.SparkConfigurator.LogData;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

import static Glitch.Lib.Motors.SparkConfigurator.getSparkMax;

public class AlgaeRemoverRollers extends SubsystemBase {
  private final SparkMax removerRollerMotor;
  private final SparkClosedLoopController removerRollerPID;

  /** Creates a new AlgaeRemoverRollers. */
  public AlgaeRemoverRollers() {
    removerRollerMotor =
        getSparkMax(
            13,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(25)
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .pid(0, 0, 0);

    removerRollerMotor.configure(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);

    removerRollerPID = removerRollerMotor.getClosedLoopController();
  }

  public void setRemoverRollerSpeed(double speed) {
    removerRollerPID.setReference(speed, ControlType.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.AlgaeRemover;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeRemover;
import frc.robot.Constants.kAlgaeRemover.kPivot.RemoverPositions;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public class AlgaeRemoverPivot extends SubsystemBase {
  private final SparkMax removerPivotMotor;
  private final SparkClosedLoopController removerPivotPID;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(50, 50)); // TODO: May need to adjust these values later
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(0,0);
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getSubsystem());

  private final ArmFeedforward pivotFeedforward =  new ArmFeedforward(0, 0.13, 0.56);

  private final double kDt = 0.02;

  /** Creates a new AlgaePivot. */
  public AlgaeRemoverPivot() {
    removerPivotMotor =
        getSparkMax(
            kAlgaeRemover.kPivot.removerPivotMotorCANID,
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
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .positionWrappingEnabled(false)
          .outputRange(-1, 1) 
          .pid(1.4, 0, 0);

    removerPivotMotor.configure(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);

    removerPivotPID = removerPivotMotor.getClosedLoopController();

    setPositionTrapazoidal(RemoverPositions.Stowed);
  }

  // set pivot position
  public void setRemoverPos(RemoverPositions angle) {
    double degrees = angle.getDegrees();
    // double rotations = angle * 75.0 / 2.0 / 360;
    removerPivotPID.setReference(degrees / 360, ControlType.kPosition);
  }

  public void setMotorFFAndPIDPosition(double removerPosition) {
    removerPivotPID.setReference(
      removerPosition,
      ControlType.kPosition,
      ClosedLoopSlot.kSlot0);
      // pivotFeedforward.calculate(
      //   removerPosition, 
      //   velocitySetpoint));
  }

  public void setPositionTrapazoidal(RemoverPositions removerPosition) {
    double rotation = removerPosition.getDegrees() / 360;
    m_goal = new TrapezoidProfile.State(rotation, 0);
    m_setpoint = new TrapezoidProfile.State(rotation, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logger.logDouble("remover Pos", removerPivotMotor.getAbsoluteEncoder().getPosition() * 360);

    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    setMotorFFAndPIDPosition(m_setpoint.position);
  }
}

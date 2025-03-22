// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot.IntakePosition;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

import static edu.wpi.first.units.Units.Volt;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public class AlgaeIntakePivot extends SubsystemBase {

  private final SparkMax intakePivotMotor;

  private final ArmFeedforward pivotFeedforward;
  private final SparkClosedLoopController pivotPID;

  private final NetworkTableLogger logger = new NetworkTableLogger(this.getSubsystem());

  private final double kDt = 0.02;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(100, 100)); //TODO: SET THESE
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(0,0);

  /** Creates a new PhantomIntake. */
  public AlgaeIntakePivot() {

// =-=-=-=- pivotMotor Initialization -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Constructor|

    intakePivotMotor =
        getSparkMax(
            kAlgaeIntakePivot.intakePivotMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT)); // TODO: logging everything for now

  // -=-=-=-=- Feedforward (Arm) for the IntakePivot (Not used currently) -=-=-=-=-=-=-|Constructor|

    pivotFeedforward = new ArmFeedforward(0, .74, .92, .03); // TODO: Calculate these using sysID

  // =-=-=-=- pivotMotor config, PID config, and maxMotion Constraints config -=-=-=-=-|Constructor|

    // Setting the output range, PID, and maxMotion constraints for the motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
      // Motor Config
      .smartCurrentLimit(Constants.kAlgaeIntake.kAlgaeIntakePivot.motorCurrentLimit)
      .idleMode(IdleMode.kBrake)
      // PID Control
      .closedLoop
        .outputRange(-1, 1)
        .pid(2, 0.0007, 2)
        .positionWrappingEnabled(false)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Configuring Motor With Config
    intakePivotMotor.configure(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);

  // -=-=-=-=- PID controller for the motor for convenience -=-=-=-=-=-=-=-=|Constructor|

    pivotPID = intakePivotMotor.getClosedLoopController();
    
    setPositionTrapazoidal(IntakePosition.HOME);
  }

// -=-=-=-=-=-=- Methods -=-=-=-=-=-=-=-=-=-=-|Subsystem|

  public void setIntakePivotPositionSimple(IntakePosition intakePositionDegrees) {
    double rotation = (intakePositionDegrees.getIntakePositionDegrees() / 360);
    pivotPID.setReference(rotation, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  private void setMotorFFAndPIDPosition(double intakePosition) {
    pivotPID.setReference(
        intakePosition,
        ControlType.kPosition, 
        ClosedLoopSlot.kSlot0);
    // System.out.println(pivotFeedforward.calculate(
    //   Math.toRadians(95 - (intakePosition/360)), 
    //   VelocitySetpoint));
    // System.out.println("intake" + pivotFeedforward.calculate(intakePosition, VelocitySetpoint));
  }

  public void setPositionTrapazoidal(IntakePosition intakePosition) {
    logger.logString("Algae Intake Pos", intakePosition.toString());
    double rotation = (intakePosition.getIntakePositionDegrees() / 360);
    m_goal = new TrapezoidProfile.State(rotation, 0);
  }

// -=-=-=-=-=-=- Commands -=-=-=-=-=-=-=-=-=-=-|Subsystem|



// -=-=-=-=-=-=- Less Used Methods -=-=-=-=-=-=-|Subsystem|



// -=-=-=-=-=-=- Periodic Override -=-=-=-=-=-=-=-=-|Subsystem|

@Override
public void periodic() {
  if (intakePivotMotor.getAbsoluteEncoder().getPosition() < 0.375) {
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    setMotorFFAndPIDPosition(m_setpoint.position);
  } else {
    pivotPID.setReference(0, ControlType.kDutyCycle);
  }

  // This method will be called once per scheduler run
  logValues();
}

  // -=-=-=--=-=-=-= Logging =-=-=-=-=-=-=-=-=-=-|Subsystem|

  /**
   * Used in subsystem periodic to log and update values
   */
  public void logValues() { // Only for calling in the periodic of this subsystem
    logger.logDouble("Motor Current", intakePivotMotor.getOutputCurrent());
    logger.logDouble("Motor Encoder Value (Relative Encoder):", intakePivotMotor.getEncoder().getPosition());
    logger.logDouble("intake Pos", intakePivotMotor.getAbsoluteEncoder().getPosition() * 360);
  }

  // -=-=-=-=-=-=- System Identification (SysId) + Methods + Commands -=-=-=-=-=-=-=-=-=-=-|Subsystem|

  // Create the SysId routine
  SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (voltage) -> this.runVolts(voltage.in(Volt)),
      null, // No log consumer, since data is recorded by URCL
      this
    )
  ); 

  // For SysIdRoutine Creation (voltage consumer [acceptor])
  public void runVolts(double voltage) {
    intakePivotMotor.setVoltage(voltage);
  }

  // For running the motor using various SysIdRoutine commands for SysId to analyze for PID and FF values
  public Command sysIdRoutine_quasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdRoutine_dynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  // Stopping the SysIdRoutine (for onFalse of the button trigger)
  public Command stopSysIdRoutine() {
    return run(() -> runVolts(0));
  }
}

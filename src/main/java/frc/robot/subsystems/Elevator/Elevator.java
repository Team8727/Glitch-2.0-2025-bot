// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kConfigs;
import frc.robot.Constants.kElevator;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

import static edu.wpi.first.units.Units.Volt;
import static frc.robot.utilities.SparkConfigurator.getFollowerMax;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;
public class Elevator extends SubsystemBase {

  private final SparkMax elevatorMotorR;
  private final SparkMax elevatorMotorL;
  private final SparkClosedLoopController elevatorPID;
  private kElevator.ElevatorPosition targetHeight;
  private kElevator.ElevatorPosition previousHeight;
  private double targetRotations;
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getSubsystem());
  public boolean isHoming = false;

//-=-=-=-=-=-=-=-=-TrapezoidProfile=-=-=-=-=-=-=-=- /

  private static final double kDt = 0.02;

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  public final TrapezoidProfile m_profile =               //in/s
      new TrapezoidProfile(new TrapezoidProfile.Constraints(103.33, 307.85)); //TODO: SET THESE
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  public TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State m_intermediate = new TrapezoidProfile.State();

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- 

  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 1.28, 3.71, 0.23);

  private final ElevatorSim elevatorSim =
    new ElevatorSim(
      3.71, 
      0.23,
      kConfigs.neoMotor,
      0.01,
      2.0,
      true,
      0.0); // Can optionally include standard deviation in measurements, but we don't have that right now!

  private final SparkMaxSim m_SparkMaxSim;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorR = getSparkMax(
      kElevator.elevatorMotorRCANID, 
      SparkMax.MotorType.kBrushless,
      true,
      Set.of(),
      Set.of(
        LogData.POSITION,
        LogData.VELOCITY,
        LogData.VOLTAGE,
        LogData.CURRENT));

    SparkMaxConfig motorRConfig = new SparkMaxConfig();
    motorRConfig
      .smartCurrentLimit(65, 65)
      .idleMode(IdleMode.kBrake)
      .closedLoop
      .pid(.4, 0, 4)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    elevatorMotorR.configure(motorRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorMotorL = getFollowerMax(
      elevatorMotorR, 
      kElevator.elevatorMotorLCANID, 
      SparkMax.MotorType.kBrushless, 
      true);

    elevatorPID = elevatorMotorR.getClosedLoopController();

    m_SparkMaxSim = 
      new SparkMaxSim(elevatorMotorR, kConfigs.neoMotor);

    setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L1);
  }

  public void setDutyCycle(double dutyCycle) {
    elevatorPID.setReference(dutyCycle, ControlType.kDutyCycle);
  }

  public double getCurrentDrawAmps() {
    return elevatorMotorR.getOutputCurrent();
  }

  public void stopElevator() {
    elevatorPID.setReference(0, ControlType.kDutyCycle);
  }

  private void setElevatorHeight(double nextPos) {
    // get double from enum
    elevatorPID.setReference(
      nextPos,
      ControlType.kPosition, 
      ClosedLoopSlot.kSlot0
      );
  }

  public void setElevatorHeightMotionProfile(kElevator.ElevatorPosition height_chosen) {
    // get double from enum
    previousHeight = targetHeight;
    targetHeight = height_chosen;
    logger.logString("elevator level", targetHeight.toString());
    targetRotations = height_chosen.getOutputRotations();
    m_goal = new TrapezoidProfile.State(targetRotations, 0);
    m_setpoint = new TrapezoidProfile.State(elevatorMotorR.getEncoder().getPosition(), 0);
  }

  public kElevator.ElevatorPosition getElevatorSetPosition() {
    return targetHeight;
  }

  public double getElevatorHeight() {
    try {
      return elevatorMotorR.getEncoder().getPosition();
    } catch (NullPointerException e) {
      return 0;
    }
  }

  public void resetElevatorEncoders() {
    elevatorMotorR.getEncoder().setPosition(0);
    elevatorMotorL.getEncoder().setPosition(0);
  }

  // Creates a SysIdRoutine
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volt)), null, this)
    );

  private void runVolts(double voltage) {
    elevatorMotorR.setVoltage(voltage);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logger.logDouble("setpos", targetRotations);
    logger.logDouble("TrapezoidProfile", m_setpoint.position);
    logger.logDouble("simPosition", elevatorSim.getPositionMeters());
    logger.logDouble("actualPos", elevatorMotorR.getEncoder().getPosition());
    logger.logDouble("goal", m_goal.position);
        
    //-=-=-=-=-=-=-=- Trapezoid Profile -=-=-=-=-=-=-=-

    // Retrieve the profiled setpoint for the next timestamp. This setpoint moves toward the goal while obeying the constraints.
    // if (getElevatorHeight() >= m_intermediate.position) {
    //   m_setpoint = m_profile.calculate(kDt, m_setpoint, m_intermediate);
    // } else {
    //   m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    // }

    if (!isHoming) {
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      // Send setpoint to offboard controller PID (I made this in periodic so when the setpositionTrapezoidProfile Method is updated it runs the elevator)
      setElevatorHeight(m_setpoint.position);
    }

    
  }

  public void simulationPeriodic() {
    elevatorSim.setInput(m_SparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    elevatorSim.update(0.02);
    
    m_SparkMaxSim.setPosition(elevatorSim.getPositionMeters());

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps())
    );
  }
}

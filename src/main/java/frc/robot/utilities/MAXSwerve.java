package frc.robot.utilities;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.kSwerve.kModule;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;

import java.util.Set;

import static frc.robot.utilities.SparkConfigurator.getSparkFlex;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public class MAXSwerve {
  private SwerveModuleState targetState = new SwerveModuleState();
  private final double chassisOffset;

  // Hardware
  private final SparkFlex driveNEO;
  private final SparkMax steerNEO;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder;

  // Controls
  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController steerPID;
  private final SimpleMotorFeedforward driveFF;

  // Simulation
  private double simDrivePosition = 0;

  public MAXSwerve(int driveCANId, int steerCANId, double offset) {
    chassisOffset = offset;

    // Initialize hardware
    driveNEO =
        getSparkFlex(
            driveCANId,
            MotorType.kBrushless,
            false,
            Set.of(Sensors.INTEGRATED),
            Set.of(LogData.VOLTAGE, LogData.POSITION, LogData.VELOCITY));
    steerNEO =
        getSparkMax(
            steerCANId,
            MotorType.kBrushless,
            false,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.VOLTAGE, LogData.POSITION, LogData.VELOCITY));

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig
        .encoder
        .positionConversionFactor(kModule.drivingEncoderPositionFactor)
        .velocityConversionFactor(kModule.drivingEncoderVelocityFactor);
    // steerConfig.closedLoop        something's wrong here but im too dumb to figure out what
    //   .feedbackSensor(driveEncoder);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(kModule.kDrive.minOutput, kModule.kDrive.maxOutput)
        .p(kModule.kDrive.kP)
        .d(kModule.kDrive.kD);
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kModule.driveSmartCurrentLimit)
        .secondaryCurrentLimit(kModule.driveMaxCurrent);
    driveNEO.configure(
        driveConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig
        .absoluteEncoder
        .inverted(kModule.invertSteerEncoder)
        .positionConversionFactor(kModule.steeringEncoderPositionFactor)
        .velocityConversionFactor(kModule.steeringEncoderVelocityFactor);
    // steerConfig.closedLoop      something's wrong here but im too dumb to figure out what
    //   .feedbackSensor(driveEncoder);
    steerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(kModule.kSteer.minOutput, kModule.kSteer.maxOutput)
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(kModule.steeringEncoderPositionPIDMaxInput)
        .positionWrappingMinInput(kModule.steeringEncoderPositionPIDMinInput)
        .p(kModule.kSteer.kP)
        .d(kModule.kSteer.kD);
    steerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kModule.driveSmartCurrentLimit)
        .secondaryCurrentLimit(kModule.driveMaxCurrent);
    steerNEO.configure(
        steerConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    driveEncoder = driveNEO.getEncoder();
    steerEncoder = steerNEO.getAbsoluteEncoder();

    // Initialize controls objects
    drivePID = driveNEO.getClosedLoopController();
    steerPID = steerNEO.getClosedLoopController();

    driveFF = new SimpleMotorFeedforward(kModule.kDrive.kS, kModule.kDrive.kV, kModule.kDrive.kA);

    if (!RobotBase.isReal()) targetState.angle = new Rotation2d(steerEncoder.getPosition());
  }

  // Get the corrected (for chassis offset) heading
  public Rotation2d getCorrectedSteer() {
    if (RobotBase.isSimulation()) return targetState.angle;
    return new Rotation2d(steerEncoder.getPosition() + chassisOffset);
  }

  // Get the state of the module (vel, heading)
  public SwerveModuleState getState() {
    if (RobotBase.isSimulation()) return targetState;
    return new SwerveModuleState(driveEncoder.getVelocity(), getCorrectedSteer());
  }

  // Get the targeted state of the module (vel, heading)
  public SwerveModuleState getTargetState() {
    return targetState;
  }

  // Get the position of the module (wheel distance traveled, heading)
  public SwerveModulePosition getPositon() {
    if (RobotBase.isSimulation())
      return new SwerveModulePosition(simDrivePosition, getCorrectedSteer());
    return new SwerveModulePosition(driveEncoder.getPosition(), getCorrectedSteer());
  }

  // Get the error of the heading
  public Rotation2d getHeadingError() {
    return targetState.angle.minus(getCorrectedSteer());
  }

  public void setTargetState(SwerveModuleState desiredState, boolean closedLoopDrive) {
    setTargetState(desiredState, closedLoopDrive, true);
  }

  // Set the module's target state
  public void setTargetState(
      SwerveModuleState state, boolean closedLoopDrive, boolean optimizeHeading) {
    // Optimize the state to prevent having to make a rotation of more than 90 degrees
      if (optimizeHeading) {
      state.optimize(getCorrectedSteer());
    }

    // Scale
    state.speedMetersPerSecond *= Math.cos(Math.abs(getHeadingError().getRadians()));

    // Set the built-in PID for closed loop, or just give a regular voltage for open loop
    if (closedLoopDrive) {
      drivePID.setReference(
          state.speedMetersPerSecond,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          driveFF.calculate(state.speedMetersPerSecond));
    } else {
      driveNEO.setVoltage(driveFF.calculate(state.speedMetersPerSecond));
    }

    steerPID.setReference(
        state.angle.minus(new Rotation2d(chassisOffset)).getRadians(),
        ControlType.kPosition);

    // Record the target state
    targetState = state;
    // Forward euler on the position in sim
    if (RobotBase.isSimulation()) simDrivePosition += targetState.speedMetersPerSecond * 0.02;
  }

  // rawvolts output for SysId
  public void setRawDriveVoltage(double volts) {
    driveNEO.setVoltage(volts);
  }

  // gets the volts that are being applied
  public double getRawDriveNeoVoltage() {
    return driveNEO.getAppliedOutput() * driveNEO.getBusVoltage();
  }

  // Set the module to the chassis X configuraiton
  public void setX() {
    setTargetState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + chassisOffset)), false);
  }

  // Sets motors all to look like an O from birdseye view, used for angular SysId
  public void setO() {
    setTargetState(
        new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4 + chassisOffset)), false);
  }

  // Reset the drive encoder to zero (reset for odometry)
  public void resetEncoder() {
    driveEncoder.setPosition(0);
    if (RobotBase.isSimulation()) simDrivePosition = 0;
  }

  // Put the drive motors into or out of brake mode
  public void setBrakeMode(boolean brake) {
    if (brake) {
      SparkMaxConfig driveconfig = new SparkMaxConfig();
      driveconfig.idleMode(IdleMode.kBrake);
      driveNEO.configure(
          driveconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    } else {
      SparkMaxConfig driveConfig = new SparkMaxConfig();
      driveConfig.idleMode(IdleMode.kCoast);
      driveNEO.configure(
          driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  /**
   * Get the output voltages
   */
  public double[] getVoltages() {
    return new double[] {
      driveNEO.getAppliedOutput() * driveNEO.getBusVoltage(),
      steerNEO.getAppliedOutput() * steerNEO.getBusVoltage()
    };
  }
}

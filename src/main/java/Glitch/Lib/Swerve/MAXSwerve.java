package Glitch.Lib.Swerve;

import Glitch.Lib.Motors.SparkConfigurator.LogData;
import Glitch.Lib.Motors.SparkConfigurator.Sensors;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Set;

import static Glitch.Lib.Motors.SparkConfigurator.getSparkFlex;
import static Glitch.Lib.Motors.SparkConfigurator.getSparkMax;

public class MAXSwerve {
  private SwerveModuleState targetState = new SwerveModuleState();
  private final double chassisOffset;

  // Chassis dimensions from wheel center to center (meters)
  public static final double width = Units.inchesToMeters(RevSwerve.wheelBaseWidth);
  private static final double length = width;

  // Kinematics
  public static final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(
      new Translation2d(length / 2, width / 2), // front right
      new Translation2d(length / 2, -width / 2), // front left
      new Translation2d(-length / 2, width / 2), // back right
      new Translation2d(-length / 2, -width / 2)); // back left

  // Drive PID
  public static final double driveKP = 0.25;
  public static final double driveKD = 0.05;
  public static final double driveKS = 0.068841;
  public static final double driveKV = 2.4568;
  public static final double driveKA = 0.22524;
  private static final double driveMinOutput = -1;
  private static final double driveMaxOutput = 1;

  // Steer PIDs
  public static final double steerKP = 2.5;
  public static final double steerKD = 0;
  private static final double steerMinOutput = -1;
  private static final double steerMaxOutput = 1;

  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  private static final int drivePinionTeeth = 14;
  private static final boolean invertSteerEncoder = true;

  private static final int driveSmartCurrentLimit = 50; // amps
  private static final int driveMaxCurrent = 80; // amps

  // Physical dimensions/values
  private static final double wheelDiameter = 0.97 * Units.inchesToMeters(3);
  private static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);

  // Motor physics
  public static final double neoFreeSpeed = 5820.0 / 60; // rot/s
  public static final double maxWheelSpeed =
    (neoFreeSpeed / driveMotorReduction) * (wheelDiameter * Math.PI); // m/s

  // Encoders
  private static final double drivingEncoderPositionFactor =
    (wheelDiameter * Math.PI) / driveMotorReduction; // meters
  private static final double drivingEncoderVelocityFactor =
    ((wheelDiameter * Math.PI) / driveMotorReduction) / 60.0; // m/s

  private static final double steeringEncoderPositionFactor = (2 * Math.PI); // radians
  private static final double steeringEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

  private static final double steeringEncoderPositionPIDMinInput = 0; // radians
  private static final double steeringEncoderPositionPIDMaxInput =
    steeringEncoderPositionFactor; // radians

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
        .positionConversionFactor(drivingEncoderPositionFactor)
        .velocityConversionFactor(drivingEncoderVelocityFactor);
    // steerConfig.closedLoop        something's wrong here but im too dumb to figure out what
    //   .feedbackSensor(driveEncoder);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(driveMinOutput, driveMaxOutput)
        .p(driveKP)
        .d(driveKD);
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveSmartCurrentLimit)
        .secondaryCurrentLimit(driveMaxCurrent);
    driveNEO.configure(
        driveConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig
        .absoluteEncoder
        .inverted(invertSteerEncoder)
        .positionConversionFactor(steeringEncoderPositionFactor)
        .velocityConversionFactor(steeringEncoderVelocityFactor);
    // steerConfig.closedLoop      something's wrong here but im too dumb to figure out what
    //   .feedbackSensor(driveEncoder);
    steerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(steerMinOutput, steerMaxOutput)
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(steeringEncoderPositionPIDMaxInput)
        .positionWrappingMinInput(steeringEncoderPositionPIDMinInput)
        .p(steerKP)
        .d(steerKD);
    steerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveSmartCurrentLimit)
        .secondaryCurrentLimit(driveMaxCurrent);
    steerNEO.configure(
        steerConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    driveEncoder = driveNEO.getEncoder();
    steerEncoder = steerNEO.getAbsoluteEncoder();

    // Initialize controls objects
    drivePID = driveNEO.getClosedLoopController();
    steerPID = steerNEO.getClosedLoopController();

    driveFF = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

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

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.kModule;
import frc.robot.utilities.MAXSwerve;
import frc.robot.utilities.NetworkTableLogger;

public class SwerveSubsystem extends SubsystemBase {
  // Create the swerve modules
  private final MAXSwerve frontLeftModule =
      new MAXSwerve(
          kSwerve.CANID.frontLeftDrive, kSwerve.CANID.frontLeftSteer, kSwerve.Offsets.frontLeft);
  private final MAXSwerve backLeftModule =
      new MAXSwerve(
          kSwerve.CANID.backLeftDrive, kSwerve.CANID.backLeftSteer, kSwerve.Offsets.backLeft);
  private final MAXSwerve backRightModule =
      new MAXSwerve(
          kSwerve.CANID.backRightDrive, kSwerve.CANID.backRightSteer, kSwerve.Offsets.backRight);
  private final MAXSwerve frontRightModule =
      new MAXSwerve(
          kSwerve.CANID.frontRightDrive, kSwerve.CANID.frontRightSteer, kSwerve.Offsets.frontRight);

  // Create the gyro
  public final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Create the network table logger to log data
  NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

  // create the module positions
  SwerveModulePosition[] modulePositions =
      new SwerveModulePosition[] {
        frontLeftModule.getPositon(),
        backLeftModule.getPositon(),
        backRightModule.getPositon(),
        frontRightModule.getPositon()
      };
  SwerveModuleState[] moduleStates =
      new SwerveModuleState[] {
        frontLeftModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState(),
        frontRightModule.getState()
      };

  SwerveDriveOdometry SwerveOdometry = new SwerveDriveOdometry(
    kSwerve.kinematics,
    getRotation2d(),
    modulePositions);
        
  Pose2d pose2d = new Pose2d();

  SwerveDrivePoseEstimator swervePoseEstimator =
      new SwerveDrivePoseEstimator(
          kSwerve.kinematics, getRotation2d(), modulePositions, pose2d);

  public SwerveSubsystem() {
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
            })
        .start();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(moduleStates);
  }

  public void zeroHeading() {
    navX.reset();
    // navX.setAngleAdjustment(0);
    swervePoseEstimator.resetRotation(pose2d.getRotation());
  }

  // maybe = get corrected steer
  public double getHeading() {
    return swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(); // THIS IS CRUCIAL
  }

  public Rotation2d getRotation2d() {
    if (Robot.isReal()) {
      return navX.getRotation2d();
    } else {
      return new Rotation2d(Math.toRadians(navX.getAngle()));
    }
  }

  @Override 
  public void simulationPeriodic() {
    networkTableLogger.logDouble("sim agnel", navX.getAngle());
  }
  @Override
  public void periodic() {
    networkTableLogger.logDouble("robotHeading", getHeading());
    networkTableLogger.logSwerveModuleState("swerveModuleStates", moduleStates);
    modulePositions[0] = frontLeftModule.getPositon();
    modulePositions[1] = backLeftModule.getPositon();
    modulePositions[2] = backRightModule.getPositon();
    modulePositions[3] = frontRightModule.getPositon();
  }

  public Command XPosition() {
    return run(
        () -> {
          frontLeftModule.setX();
          frontRightModule.setX();
          backLeftModule.setX();
          backRightModule.setX();
        });
  }

  public void stopModules() {}

  public void setModuleStates(SwerveModuleState[] desiredState) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, kModule.maxWheelSpeed);
    networkTableLogger.logSwerveModuleState("desiredModuleStates", desiredState);
    frontLeftModule.setTargetState(desiredState[0], true, true);
    frontRightModule.setTargetState(desiredState[1], true, true);
    backLeftModule.setTargetState(desiredState[2], true, true);
    backRightModule.setTargetState(desiredState[3], true, true);
  }
}

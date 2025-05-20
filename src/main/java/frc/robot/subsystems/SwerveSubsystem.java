package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.kModule;
import frc.robot.Robot;
import frc.robot.utilities.MAXSwerve;
import Glitch.Lib.NetworkTableLogger;

public class SwerveSubsystem extends SubsystemBase {
  // Swerve uses ccw+ angular quanities and a coordinate plane with 0,0 at the robot's center
  // , forward is +x, and a module order based on the quadrant system (front left is first)
  // BL        FL
  //       C
  // BR        FR
  // NOTE: Make sure ModuleLocation and the order of the modules in the kinematics match
  private enum ModuleLocation {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }
  private final int kNumModules = ModuleLocation.values().length;

  // Module angular offsets (rad)
  public static double frontLeftOffset = Math.PI / 2;
  public static double backLeftOffset = -Math.PI;
  public static double backRightOffset = -Math.PI / 2;
  public static double frontRightOffset = 0;

  // Swerve modules
  private final MAXSwerve[] modules = new MAXSwerve[kNumModules];
  private final SwerveModulePosition[] cachedModulePositions = new SwerveModulePosition[kNumModules];
  private final SwerveModuleState[] cachedModuleStates = new SwerveModuleState[kNumModules];

  // Motor CAN IDs
  private static int frontLeftDriveID = 9; //
  private static int frontLeftSteerID = 8; //
  private static int backLeftDriveID = 3; //
  private static int backLeftSteerID = 2; //
  private static int backRightDriveID = 5; //
  private static int backRightSteerID = 4; //
  private static int frontRightDriveID = 7; //
  private static int frontRightSteerID = 6; //

  // Gyro
  private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
  // Simulated Gyro
  private class SimGyro {
    private double nextHeading = 0;
    private double currentHeading = 0;

    public void reset() {
      setNextHeading(0);
      applyNextHeading();
    }

    public void setNextHeading(double heading) {
      nextHeading = heading;
    }

    public void applyNextHeading() {
      currentHeading = nextHeading;
    }

    public double getCurrentHeading() {
      return currentHeading;
    }
  }
  private SimGyro simGyro = new SimGyro();

  // Network Table Logger
  private final NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

  public SwerveSubsystem() {
    initSwerveModules();

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

  // Call this if you ever need to re-initialize the swerve modules
  private void initSwerveModules() {
    modules[ModuleLocation.FRONT_LEFT.ordinal()] = new MAXSwerve(frontLeftDriveID, frontLeftSteerID, frontLeftOffset);
    modules[ModuleLocation.FRONT_RIGHT.ordinal()] = new MAXSwerve(frontRightDriveID, frontRightSteerID, frontRightOffset);
    modules[ModuleLocation.BACK_LEFT.ordinal()] = new MAXSwerve(backLeftDriveID, backLeftSteerID, backLeftOffset);
    modules[ModuleLocation.BACK_RIGHT.ordinal()] = new MAXSwerve(backRightDriveID, backRightSteerID, backRightOffset);
  }

  /**
   * Get the current positions of the swerve modules
   *
   * @return the positions of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions() {
    // Update the cache
    // (OPTIMIZATION: we could rate-limit this if needed)
    for (int i = 0; i < cachedModulePositions.length; ++i) {
      cachedModulePositions[i] = modules[i].getPositon();
    }

    return cachedModulePositions.clone();
  }

  /**
   * Get the current states of the swerve modules
   *
   * @return the states of the swerve modules
   */
  public SwerveModuleState[] getModuleStates() {
    // Update the cache
    // (OPTIMIZATION: we could rate-limit this if needed)
    for (int i = 0; i < cachedModuleStates.length; ++i) {
      cachedModuleStates[i] = modules[i].getState();
    }

    return cachedModuleStates.clone();
  }

  /**
   * Get the current chassis speeds
   *
   * @return the chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Zero the heading of the robot
   */
  public void zeroHeading() {
    navX.reset();
    // navX.setAngleAdjustment(0);
    simGyro.reset();
  }

  /**
   * Get the heading of the robot
   *
   * @return the heading of the robot
   */
  public Rotation2d getHeading() {
    if (Robot.isReal()) {
      return navX.getRotation2d();
    } else {
      return new Rotation2d(simGyro.getCurrentHeading());
    }
  }

  /**
   * Reset the simulated heading of the robot
   * @param headingRadians The heading in radians
   */
  public void setNextSimHeading(double headingRadians) {
    simGyro.setNextHeading(headingRadians);
    networkTableLogger.logDouble("Next Sim Heading", Math.toDegrees(simGyro.nextHeading));
  }

  /**
   * Update the simulated heading of the robot
   */
  public void applySimHeading() {
    simGyro.applyNextHeading();
  }

  @Override
  public void periodic() {
    networkTableLogger.logDouble("Heading", getHeading().getDegrees());
    networkTableLogger.logSwerveModuleState("Swerve Module States", getModuleStates());
  }

  /**
   * Set the chassis speeds of the robot, using robot-relative speeds
   * @param robotRelativeSpeeds the robot-relative speeds
   */
  public void setChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    setModuleStates(kSwerve.kinematics.toSwerveModuleStates(robotRelativeSpeeds));

    if (Robot.isSimulation()) {
      setNextSimHeading(simGyro.nextHeading + robotRelativeSpeeds.omegaRadiansPerSecond * 0.02);
    }
  }

  private void setModuleStates(SwerveModuleState[] desiredState) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, kModule.maxWheelSpeed);
    networkTableLogger.logSwerveModuleState("Desired Swerve Module States", desiredState);

    for (int i = 0; i < modules.length; ++i) {
      modules[i].setTargetState(desiredState[i], true, true);
    }
  }
}

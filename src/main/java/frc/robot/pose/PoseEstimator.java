package frc.robot.pose;

import Glitch.Lib.NetworkTableLogger;
import Glitch.Lib.Swerve.MAXSwerve;
import Glitch.Lib.Swerve.RevSwerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.vision.Vision;

/**
 * Robot-layer Pose subsystem that owns the library pose estimator and fuses vision.
 * <p>
 * This class ties the reusable library-level pose estimator to the robot's swerve subsystem
 * and the robot-layer vision facade. It is responsible for:
 * - Updating odometry/pose with current heading and module positions each loop
 * - Draining latency-compensated vision measurements and injecting them into the estimator
 * - Providing a simple API to get/reset pose and zero heading
 * - Publishing a fused {@link Field2d} for dashboards
 */
public class PoseEstimator extends SubsystemBase {
  private final RevSwerve m_SwerveSubsystem;
  private final Glitch.Lib.Pose.PoseEstimator m_LibEstimator;
  private final Vision m_Vision;
  private final NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName());

  /** Fused robot pose visualization published to NetworkTables. */
  public Field2d field2d = new Field2d();

  /**
   * Constructs the pose subsystem.
   *
   * @param swerveSubsystem Robot swerve subsystem for heading and module positions
   * @param vision Robot-layer vision facade used to supply measurements
   */
  public PoseEstimator(RevSwerve swerveSubsystem, Vision vision) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_Vision = vision;
    this.m_LibEstimator = new Glitch.Lib.Pose.PoseEstimator(
        MAXSwerve.kinematics,
        m_SwerveSubsystem.getHeading(),
        m_SwerveSubsystem.getModulePositions(),
        new Pose2d());

    resetStartPose();
  }

  /**
   * @return The current fused field pose estimate. In simulation, uses the swerve heading.
   */
  public Pose2d get2dPose() {
    Pose2d est = m_LibEstimator.getPose();
    if (Robot.isSimulation()) {
      return new Pose2d(est.getTranslation(), m_SwerveSubsystem.getHeading());
    }
    return est;
  }

  /**
   * Resets the fused pose to the provided value. In simulation, also synchronizes the swerve heading.
   *
   * @param pose2d Target field pose
   */
  public void resetPoseToPose2d(Pose2d pose2d) {
    m_LibEstimator.reset(pose2d);
    if (Robot.isSimulation()) {
      m_SwerveSubsystem.setNextSimHeading(pose2d.getRotation().getRadians());
      m_SwerveSubsystem.applySimHeading();
    }
  }

  /** Resets the pose estimate using the best available vision-derived start pose, or (0,0,0) if unavailable. */
  public void resetStartPose() {
    resetPoseToPose2d(m_Vision.bestStartPose().orElse(new Pose2d()));
  }

  /** Resets the pose estimate to the origin (0,0,0). */
  public void resetToEmptyPose() {
    resetPoseToPose2d(new Pose2d());
  }

  /** Zeros the robot's heading and rotation estimate. */
  public void zeroHeading() {
    m_SwerveSubsystem.zeroHeading();
    m_LibEstimator.resetRotation(new Rotation2d());
  }

  /**
   * Periodic loop: updates odometry, fuses any available vision, and publishes telemetry.
   */
  @Override
  public void periodic() {
    // Update vision and drain measurements
    m_Vision.periodic();

    double now = Timer.getFPGATimestamp();
    m_LibEstimator.updateWithTime(now, m_SwerveSubsystem.getHeading(), m_SwerveSubsystem.getModulePositions());

    for (frc.robot.vision.Vision.Measurement m : m_Vision.drainMeasurements(get2dPose())) {
      m_LibEstimator.addVisionMeasurement(m.getPose(), m.getTimestampSeconds());
    }

    // Update telemetry
    field2d.setRobotPose(get2dPose());
    networkTableLogger.logField2d("Field2d", field2d);
    networkTableLogger.logPose2d("Robot 2d Pose", get2dPose());

    // Also publish camera poses to NetworkTables for visualization
    m_Vision.logCameraPoses(get2dPose());

    // Optional: publish vision debug field if present (sim only)
    m_Vision.getDebugField().ifPresent(f -> networkTableLogger.logField2d("Vision Debug Field", f));
  }

  /**
   * Simulation loop: keeps the swerve heading synchronized after resets.
   */
  @Override
  public void simulationPeriodic() {
    // Maintain heading sync in sim
    m_SwerveSubsystem.applySimHeading();
  }
}

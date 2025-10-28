package Glitch.Lib.Pose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Library-level, vendor-agnostic wrapper around {@link SwerveDrivePoseEstimator}.
 * <p>
 * This class owns pose fusion for a swerve drivetrain, combining gyro heading and
 * module encoder positions into an estimated {@link Pose2d}. It also exposes a hook
 * to inject latency-compensated vision measurements gathered elsewhere.
 * <p>
 * This type contains no vendor- or camera-specific code and is intended to be reused
 * across robots. Vision inputs should be provided by a separate Vision provider.
 */
public class PoseEstimator {
  private final SwerveDrivePoseEstimator estimator;

  /**
   * Constructs a new swerve pose estimator.
   *
   * @param kinematics Swerve kinematics for the drivetrain geometry
   * @param initialHeading Current gyro heading at construction time
   * @param initialModulePositions Current module positions at construction time
   * @param initialPose Initial field pose of the robot
   */
  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d initialHeading,
      SwerveModulePosition[] initialModulePositions,
      Pose2d initialPose) {
    this.estimator = new SwerveDrivePoseEstimator(
        kinematics,
        initialHeading,
        initialModulePositions,
        initialPose);
  }

  /**
   * Updates odometry and pose estimation using the given timestamp, heading, and module states.
   *
   * @param timestampSeconds Monotonic FPGA timestamp in seconds (see {@link edu.wpi.first.wpilibj.Timer})
   * @param heading Current robot heading from gyro/navX
   * @param modulePositions Current measured positions for all swerve modules
   */
  public void updateWithTime(double timestampSeconds, Rotation2d heading, SwerveModulePosition[] modulePositions) {
    estimator.updateWithTime(timestampSeconds, heading, modulePositions);
  }

  /**
   * Injects a single latency-compensated vision measurement into the estimator.
   * <p>
   * The timestamp must correspond to the measurement time so the estimator can
   * back-propagate and re-linearize appropriately.
   *
   * @param pose Estimated field pose from vision
   * @param timestampSeconds Measurement timestamp in seconds
   */
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    estimator.addVisionMeasurement(pose, timestampSeconds);
  }

  /**
   * Returns the current fused field pose estimate.
   *
   * @return Current estimated robot pose
   */
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  /**
   * Resets the internal state to the provided field pose.
   *
   * @param pose New pose to reset to
   */
  public void reset(Pose2d pose) {
    estimator.resetPose(pose);
  }

  /**
   * Resets only the rotation estimate while keeping the current translation.
   *
   * @param rotation New rotation to apply
   */
  public void resetRotation(Rotation2d rotation) {
    estimator.resetRotation(rotation);
  }
}

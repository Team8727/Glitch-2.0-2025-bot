// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.Robot;
import frc.robot.utilities.NetworkTableLogger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PoseEstimator extends SubsystemBase {
  final SwerveSubsystem m_SwerveSubsystem;
  final SwerveDrivePoseEstimator m_SwervePoseEstimator;
  final NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

  private VisionSystemSim visionSim;

  private final PhotonCamera frontRightCamera = new PhotonCamera("frontRight");
  private final PhotonCamera frontLeftCamera = new PhotonCamera("frontLeft");
  private final PhotonCamera centerCamera = new PhotonCamera("center");

  private PhotonCameraSim cameraSimFrontRight;
  private PhotonCameraSim cameraSimFrontLeft;
  private PhotonCameraSim cameraSimCenter;


  private SimCameraProperties cameraProp;

  // Field2d for logging the robot's 2d position on the field to the dashboard like AdvantageScope, Elastic or Glass.
  private Field2d visionDebugField;
  public Field2d field2d = new Field2d();

  /**
   * Creates a new PoseEstimator.
   *
   * @param swerveSubsystem The swerve subsystem used for odometry and heading information.
   */
  public PoseEstimator(SwerveSubsystem swerveSubsystem) {
    // subsystem setups
    m_SwerveSubsystem = swerveSubsystem;
    m_SwervePoseEstimator =
      new SwerveDrivePoseEstimator(
        kSwerve.kinematics,
        m_SwerveSubsystem.getHeading(),
        m_SwerveSubsystem.getModulePositions(),
        new Pose2d()
      );

    // setup camera simulation
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      cameraProp = new SimCameraProperties();
      visionDebugField = visionSim.getDebugField();
      
      visionSim.addAprilTags(kVision.aprilTagFieldLayout);
      // A 640 x 480 camera with a 100 degree diagonal FOV.
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
      // Approximate detection noise with average and standard deviation error in pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(40);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      cameraSimFrontRight = new PhotonCameraSim(frontRightCamera, cameraProp);
      cameraSimFrontLeft = new PhotonCameraSim(frontLeftCamera, cameraProp);
      cameraSimCenter = new PhotonCameraSim(centerCamera, cameraProp);

      // this slows down loop time a lot
      // cameraSimFrontRight.enableDrawWireframe(true);
      // cameraSimFrontLeft.enableDrawWireframe(true);

      visionSim.addCamera(cameraSimFrontRight, kVision.frontRightCamera);
      visionSim.addCamera(cameraSimFrontLeft, kVision.frontLeftCamera);
      visionSim.addCamera(cameraSimCenter, kVision.centerCamera);

    }
    resetStartPose();
  }

  // photon pose estimators
  PhotonPoseEstimator PoseEstimatorFrontLeft =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.frontLeftCamera);
  PhotonPoseEstimator PoseEstimatorFrontRight =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.frontRightCamera);
  PhotonPoseEstimator PoseEstimatorCenter =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.centerCamera);


  /**
   * Gets the starting pose of the robot.
   *
   * @return The starting pose of the robot as a Pose2d object.
   */
  public Pose2d getStartPose2d() {
    // vars
    Pose3d pose3d = new Pose3d();
    PhotonPipelineResult result = frontLeftCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    // if camera sees targets
    if (hasTargets) {
      // find best target
      PhotonTrackedTarget target = result.getBestTarget();
      if (kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        // estimate field to robot pose
        pose3d =
            PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                kVision.frontLeftCamera);
      }
    }
    return pose3d.toPose2d();
  }

  /**
   * Resets the robot's pose to the specified Pose2d.
   *
   * @param pose2d The new pose to reset to.
   */
  public void resetPoseToPose2d(Pose2d pose2d) {
    // m_SwerveSubsystem.navX.setAngleAdjustment(pose2d.getRotation().getDegrees());
    m_SwervePoseEstimator.resetPose(pose2d);
    // System.out.println(m_SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    if (Robot.isSimulation()) {
      m_SwerveSubsystem.setNextSimHeading(pose2d.getRotation().getRadians());
      m_SwerveSubsystem.applySimHeading();
    }
  }

  /**
   * Resets the robot's pose to the starting pose.
   */
  public void resetStartPose() {
    resetPoseToPose2d(getStartPose2d());
  }

  /**
   * Resets the robot's pose to the empty pose.
   */
  public void resetToEmptyPose() {
    resetPoseToPose2d(new Pose2d());
  }

  /**
   * Gets the current 2D pose of the robot.
   *
   * @return The current 2D pose of the robot as a Pose2d object.
   */
  public Pose2d get2dPose() {
    if (Robot.isSimulation()) {
        return new Pose2d(
          m_SwervePoseEstimator.getEstimatedPosition().getTranslation(),
          m_SwerveSubsystem.getHeading());
    }

    return m_SwervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot.
   * @param cameraResult The result from the camera pipeline.
   * @param PoseEstimator The pose estimator to use.
   * @return An Optional containing the estimated robot pose.
   */
  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(
      Pose2d prevEstimatedRobotPose,
      PhotonPipelineResult cameraResult,
      PhotonPoseEstimator PoseEstimator) {
    PoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return PoseEstimator.update(cameraResult);
  }

  /**
   * Zeros the robot's heading.
   */
  public void zeroHeading() {
    m_SwerveSubsystem.zeroHeading();
    m_SwervePoseEstimator.resetRotation(new Rotation2d());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param camera The camera to use for the vision measurement.
   * @param poseEstimator The pose estimator to use.
   */
  private void addVisionMeasurement(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
    try {
      // camera 4 pose estimation
      List<PhotonPipelineResult> cameraRes = camera.getAllUnreadResults();
      PhotonPipelineResult cameraLatestRes = cameraRes.get(cameraRes.size() - 1);
      List<PhotonTrackedTarget> targets = cameraLatestRes.getTargets();
      for (PhotonTrackedTarget target : targets) {
        double ambiguity = target.getPoseAmbiguity();
        double distance = (
          Math.sqrt(
            Math.pow(target.getBestCameraToTarget().getX(), 2) 
            + 
            Math.pow(target.getBestCameraToTarget().getY(), 2) 
            +
            Math.pow(target.getBestCameraToTarget().getZ(), 2)));

        if (ambiguity <= 0.2 && ambiguity != -1 && distance < 3.5) {
          Optional<EstimatedRobotPose> cameraPose =
            getEstimatedGlobalPose(
                m_SwervePoseEstimator.getEstimatedPosition(),
                cameraLatestRes, 
                poseEstimator);
        
          m_SwervePoseEstimator.addVisionMeasurement(
              cameraPose.get().estimatedPose.toPose2d(), cameraLatestRes.getTimestampSeconds());
        }
      }
    } catch (Exception ignored) {
    }
  }

  /**
   * Called periodically in simulation.
   */
  @Override
  public void simulationPeriodic() {
    visionSim.update(m_SwervePoseEstimator.getEstimatedPosition());
    networkTableLogger.logPose3d("cam front", visionSim.getCameraPose(cameraSimFrontRight).orElse(new Pose3d()));
    networkTableLogger.logPose3d("cam back up", visionSim.getCameraPose(cameraSimFrontLeft).orElse(new Pose3d()));
    networkTableLogger.logPose3d("cam center", visionSim.getCameraPose(cameraSimCenter).orElse(new Pose3d()));
    networkTableLogger.logField2d("Vision Debug Field", visionDebugField);

    // Changed second parameter of .addVisionMeasurement() to use `Timer.getFPGATimestamp()` which is in seconds (what this method wants) rather than the previous `RobotController.getFPGATime()` which is in microseconds
    m_SwervePoseEstimator.addVisionMeasurement(visionSim.getRobotPose().toPose2d(), Timer.getFPGATimestamp());
    m_SwerveSubsystem.applySimHeading();
  }

  /**
   * Always called periodically.
   */
  @Override
  public void periodic() {
      // camera 3 pose estimation
     addVisionMeasurement(frontLeftCamera, PoseEstimatorFrontLeft);
      // camera 4 pose estimation
     addVisionMeasurement(frontRightCamera, PoseEstimatorFrontRight);
      // center camera pose estimation
     addVisionMeasurement(centerCamera, PoseEstimatorCenter);

    // gyro update
    m_SwervePoseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      m_SwerveSubsystem.getHeading(),
      m_SwerveSubsystem.getModulePositions());

    // Update Field2d with pose to display the robot's visual position on the field to the dashboard
    field2d.setRobotPose(get2dPose());

    // Log the robot's 2d position on the field to the dashboard using the NetworkTableLogger
    networkTableLogger.logField2d("Field2d", field2d);
    networkTableLogger.logPose2d("Robot 2d Pose", get2dPose());
  }
}

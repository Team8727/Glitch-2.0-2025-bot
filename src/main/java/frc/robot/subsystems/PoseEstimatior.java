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
import frc.robot.Robot;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.utilities.NetworkTableLogger;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimatior extends SubsystemBase {
  final SwerveSubsystem m_SwerveSubsystem;
  final SwerveDrivePoseEstimator m_SwervePoseEstimator;
  final NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

  private VisionSystemSim visionSim;

  private PhotonCamera camera1 = new PhotonCamera("backRight");
  private PhotonCamera camera2 = new PhotonCamera("backLeft");
  private PhotonCamera camera3 = new PhotonCamera("front");
  private PhotonCamera camera4 = new PhotonCamera("backUp");

  private PhotonCameraSim cameraSimBackRight;
  private PhotonCameraSim cameraSimBackLeft;
  private PhotonCameraSim cameraSimFront;
  private PhotonCameraSim cameraSimBackUp;

  private SimCameraProperties cameraProp;

  // Field2d for logging the robot's 2d position on the field to the dashboard like AdvantageScope, Elastic or Glass.
  private Field2d visionDebugField;
  public Field2d field2d = new Field2d();

  /** Creates a new PoseEstimation. */
  public PoseEstimatior(SwerveSubsystem swerveSubsystem) {
    // subsystem setups
    m_SwerveSubsystem = swerveSubsystem;
    m_SwervePoseEstimator =     
      new SwerveDrivePoseEstimator(
        kSwerve.kinematics, 
        m_SwerveSubsystem.getHeading(), 
        m_SwerveSubsystem.getModulePositions(), 
        new Pose2d()
      );

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

      cameraSimBackRight = new PhotonCameraSim(camera1, cameraProp);
      cameraSimBackLeft = new PhotonCameraSim(camera2, cameraProp);
      cameraSimFront = new PhotonCameraSim(camera3, cameraProp);
      cameraSimBackUp = new PhotonCameraSim(camera4, cameraProp);

      // this slows down loop time a lot
      // cameraSimBackLeft.enableDrawWireframe(true);
      // cameraSimBackRight.enableDrawWireframe(true);
      // cameraSimFront.enableDrawWireframe(true);
      // cameraSimBackUp.enableDrawWireframe(true);

      visionSim.addCamera(cameraSimBackRight, kVision.camera1Position);
      visionSim.addCamera(cameraSimBackLeft, kVision.camera2Position);
      visionSim.addCamera(cameraSimFront, kVision.camera4Position);
      visionSim.addCamera(cameraSimBackUp, kVision.camera3Position);
    }
    resetStartPose();
  }

  // photon pose estimators
  PhotonPoseEstimator PoseEstimator1 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera1Position);
  PhotonPoseEstimator PoseEstimator2 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera2Position);
  PhotonPoseEstimator PoseEstimator3 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera3Position);
  PhotonPoseEstimator PoseEstimator4 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera4Position);

  // get starting pos with cam1
  public Pose2d getStartPose2d() {
    // vars
    Pose3d pose3d = new Pose3d();
    PhotonPipelineResult result = camera1.getLatestResult();
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
                kVision.camera1Position);
      }
    }
    return pose3d.toPose2d();
  }

  public void resetPoseToPose2d(Pose2d pose2d) {
    // m_SwerveSubsystem.navX.setAngleAdjustment(pose2d.getRotation().getDegrees());
    m_SwervePoseEstimator.resetPose(pose2d);
    // System.out.println(m_SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    
    if (Robot.isSimulation()) {
      m_SwerveSubsystem.setNextSimHeading(pose2d.getRotation().getRadians());
      m_SwerveSubsystem.applySimHeading();
    }
  }

  public void resetStartPose() {
    resetPoseToPose2d(getStartPose2d());
  }

  public void resetToEmptyPose() {
    resetPoseToPose2d(new Pose2d());
  }

  // Get 2d pose: from the poseEstimator
  public Pose2d get2dPose() {
    if (Robot.isSimulation()) {
      Pose2d pose = new Pose2d(
        m_SwervePoseEstimator.getEstimatedPosition().getTranslation(), 
        m_SwerveSubsystem.getHeading());
      return pose;
    }

    return m_SwervePoseEstimator.getEstimatedPosition();
  }

  Optional<EstimatedRobotPose> getEstimatedGlobalPose(
      Pose2d prevEstimatedRobotPose,
      PhotonPipelineResult cameraResult,
      PhotonPoseEstimator PoseEstimator) {
    PoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return PoseEstimator.update(cameraResult);
  }

  public void zeroHeading() {
    m_SwerveSubsystem.zeroHeading();
    m_SwervePoseEstimator.resetRotation(new Rotation2d());
  }

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
    } catch (Exception e) {
    }
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(m_SwervePoseEstimator.getEstimatedPosition());
    networkTableLogger.logPose3d("cam back Right", visionSim.getCameraPose(cameraSimBackRight).orElse(new Pose3d()));
    networkTableLogger.logPose3d("cam back left", visionSim.getCameraPose(cameraSimBackLeft).orElse(new Pose3d()));
    networkTableLogger.logPose3d("cam front", visionSim.getCameraPose(cameraSimFront).orElse(new Pose3d()));
    networkTableLogger.logPose3d("cam back up", visionSim.getCameraPose(cameraSimBackUp).orElse(new Pose3d()));
    m_SwervePoseEstimator.addVisionMeasurement(visionSim.getRobotPose().toPose2d(), RobotController.getFPGATime());
    networkTableLogger.logField2d("Vision Debug Field", visionDebugField);

    m_SwerveSubsystem.applySimHeading();
  }

  @Override
  public void periodic() {
    // // camera 1 pose estimation
    // addVisionMeasurement(camera1, PoseEstimator1);
    // // // camera 2 pose estimation
    // addVisionMeasurement(camera2, PoseEstimator2);
    // // // camera 3 pose estimation
    // addVisionMeasurement(camera3, PoseEstimator3);
    // // // camera 4 pose estimation
    // addVisionMeasurement(camera4, PoseEstimator4);

    // gyro update
    // if (Robot.isReal()) {
      m_SwervePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), 
        m_SwerveSubsystem.getHeading(), 
        m_SwerveSubsystem.getModulePositions());
    // } else {
    //   m_SwervePoseEstimator.updateWithTime(
    //     Timer.getFPGATimestamp(), 
    //     m_SwerveSubsystem.getRotation2d(), 
    //     m_SwerveSubsystem.modulePositions);
    // }
    // Update Field2d with pose to display the robot's visual position on the field to the dashboard
    field2d.setRobotPose(get2dPose());

    // field.setRobotPose(m_swervePoseEstimator.getEstimatedPosition().toPose2d());// 2d pose

    // Log the robot's 2d position on the field to the dashboard using the NetworkTableLogger
    // Utility
    networkTableLogger.logField2d("Field2d", field2d);
    networkTableLogger.logPose2d("Robot 2d Pose", get2dPose());
  }
}

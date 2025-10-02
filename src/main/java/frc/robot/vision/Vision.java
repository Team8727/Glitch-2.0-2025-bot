package frc.robot.vision;

import Glitch.Lib.NetworkTableLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Robot-layer Vision configuration and facade.
 * <p>
 * This class owns all robot-specific vision configuration (camera names, robot-to-camera transforms,
 * filter thresholds, and simulation properties) and builds a library-level Vision provider
 * for real or simulated environments. It presents a minimal API suitable for consumption by
 * the robot-layer pose subsystem.
 */
public class Vision implements AutoCloseable {
  /**
   * Robot-layer measurement wrapper to decouple robot code from library nested types.
   * <p>
   * Values are produced by the underlying library provider but re-wrapped here for stability
   * of the robot API.
   */
  public static class Measurement {
    /** Estimated field pose at capture time. */
    public final Pose2d pose;
    /** Capture timestamp in seconds. */
    public final double timestampSeconds;
    public Measurement(Pose2d pose, double timestampSeconds) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
    }
    /** @return Pose measured by vision at capture time */
    public Pose2d getPose() { return pose; }
    /** @return Measurement timestamp in seconds */
    public double getTimestampSeconds() { return timestampSeconds; }
  }

  private final Glitch.Lib.Vision.Vision.Provider provider;
  private final NetworkTableLogger logger = new NetworkTableLogger("Vision");

  // Camera names as configured in PhotonVision
  private static final String CAM_FRONT_RIGHT = "frontRight";
  private static final String CAM_FRONT_LEFT = "frontLeft";
  private static final String CAM_CENTER = "center";

  // Robot-to-camera transforms
  private static final Transform3d FRONT_LEFT_POS =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(-8), Units.inchesToMeters(19.5)),
          new Rotation3d(Math.toRadians(14.586), Math.toRadians(25), Math.toRadians(34)));

  private static final Transform3d FRONT_RIGHT_POS =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(8), Units.inchesToMeters(19.5)),
          new Rotation3d(Math.toRadians(-14.586), Math.toRadians(25), Math.toRadians(-34)));

  private static final Transform3d CENTER_POS =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(8.5), Units.inchesToMeters(0), Units.inchesToMeters(8.5)),
          new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

  // Thresholds and sim properties
  private static final double MAX_AMBIGUITY = 0.2; // ignore -1 (handled in provider)
  private static final double MAX_DISTANCE_METERS = 3.5;
  private static final int SIM_WIDTH = 640;
  private static final int SIM_HEIGHT = 480;
  private static final double SIM_FOV_DEG = 70;
  private static final int SIM_FPS = 40;
  private static final double SIM_AVG_LAT_MS = 35;
  private static final double SIM_LAT_STD_MS = 5;

  /**
   * Constructs the robot-layer Vision facade and underlying provider.
   * <p>
   * Chooses a real vs. simulated provider based on {@link Robot#isSimulation()} and injects
   * robot-specific camera configurations and thresholds.
   */
  public Vision() {
    List<Glitch.Lib.Vision.Vision.CameraConfig> cameras = Arrays.asList(
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_FRONT_LEFT, FRONT_LEFT_POS),
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_FRONT_RIGHT, FRONT_RIGHT_POS),
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_CENTER, CENTER_POS)
    );

    Glitch.Lib.Vision.Vision.Config cfg = new Glitch.Lib.Vision.Vision.Config(
        cameras,
        MAX_AMBIGUITY,
        MAX_DISTANCE_METERS,
        SIM_WIDTH, SIM_HEIGHT, SIM_FOV_DEG, SIM_FPS,
        SIM_AVG_LAT_MS, SIM_LAT_STD_MS);

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    if (Robot.isSimulation()) {
      this.provider = Glitch.Lib.Vision.Vision.createPhotonVisionSim(cfg, layout);
    } else {
      this.provider = Glitch.Lib.Vision.Vision.createPhotonVision(cfg, layout);
    }
  }

  /** Optional provider hook for per-loop processing. */
  public void periodic() {
    provider.periodic();
  }

  /**
   * Retrieves and maps any available measurements from the provider using the supplied reference pose.
   *
   * @param referencePose Current best estimate of robot pose; used by the provider to disambiguate
   * @return Robot-layer measurements (pose, timestamp) suitable for the pose estimator
   */
  public List<Measurement> drainMeasurements(Pose2d referencePose) {
    return provider.drainMeasurements(referencePose)
        .stream()
        .map(m -> new Measurement(m.pose, m.timestampSeconds))
        .collect(Collectors.toList());
  }

  /**
   * Attempts to compute an initial robot pose from visible tags (e.g., at boot).
   *
   * @return Optional initial field pose if a valid target is currently visible
   */
  public Optional<Pose2d> bestStartPose() {
    return provider.bestStartPose();
  }

  /**
   * @return Optional debug {@link Field2d} provided by the underlying provider (sim only).
   */
  public Optional<Field2d> getDebugField() {
    return provider.getDebugField();
  }

  /**
   * Computes each camera's current field pose from the supplied robot pose and publishes
   * them to NetworkTables for visualization (e.g., AdvantageScope/Glass).
   *
   * @param robotPose Current robot field pose (fused or odometry)
   */
  public void logCameraPoses(Pose2d robotPose) {
    var robotPose3d = new edu.wpi.first.math.geometry.Pose3d(robotPose);
    var camFrontLeft = robotPose3d.transformBy(FRONT_LEFT_POS);
    var camFrontRight = robotPose3d.transformBy(FRONT_RIGHT_POS);
    var camCenter = robotPose3d.transformBy(CENTER_POS);

    logger.logPose3d("/" + CAM_FRONT_LEFT + "/Pose", camFrontLeft);
    logger.logPose3d("/" + CAM_FRONT_RIGHT + "/Pose", camFrontRight);
    logger.logPose3d("/" + CAM_CENTER + "/Pose", camCenter);
  }

  /** Releases any provider resources (no-op by default). */
  @Override
  public void close() {
    provider.close();
  }
}

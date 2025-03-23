// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.Robot;
import frc.robot.commands.CoralCmds.DeployCoralCmd;
import frc.robot.commands.ElevatorCmds.SetElevatorHeightCmd;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.utilities.NetworkTableLogger;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.LinkedHashMap;

public class Autos extends SubsystemBase {
  private final LEDSubsystem m_ledSubsystem;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<>();
  private final PoseEstimator m_PoseEstimator;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getName());

  /**
   * Enum representing the scoring points on the reef.
   * Each enum value contains the zone name, point, right and left poses, and distance to side.
   */
  public enum ReefScorePoints {
    A_B("A-B",
      new Translation2d(3.65, 4.02), // side point
      GetScorePose(new Rotation2d(Math.toRadians(180)), 0, true), // right score location
      GetScorePose(new Rotation2d(Math.toRadians(180)), 0, false), // left score location
      0), // distance to side

    C_D("C-D", 
      new Translation2d(4.07, 3.3), // side point
      GetScorePose(new Rotation2d(Math.toRadians(-120)), 0, true), // right score location
      GetScorePose(new Rotation2d(Math.toRadians(-120)), 0, false), // left score location
      0),  // distance to side

    E_F("E-F", 
      new Translation2d(4.9, 3.3), 
      GetScorePose(new Rotation2d(Math.toRadians(-60)), 0, true), // right score location
      GetScorePose(new Rotation2d(Math.toRadians(-60)), 0, false), // left score location
      0),  // Zone E-F

    G_H("G-H", 
      new Translation2d(5.35, 4.02), 
      GetScorePose(new Rotation2d(Math.toRadians(0)), 0, true), // right score location
      GetScorePose(new Rotation2d(Math.toRadians(0)), 0, false), // left score location
      0),  // Zone G-H

    I_J("I-J", 
      new Translation2d(4.9, 4.75), 
      GetScorePose(new Rotation2d(Math.toRadians(60)), 0, true), // right score location
      GetScorePose(new Rotation2d(Math.toRadians(60)), 0, false), // left score location
      0),  // Zone I-J

    K_L("K-L", 
      new Translation2d(4.07, 4.75), 
      GetScorePose(new Rotation2d(Math.toRadians(120)), 0, true), // right score location 
      GetScorePose(new Rotation2d(Math.toRadians(120)), 0, false), // left score location
      0);  // Zone K-L

    private final String zone;
    private final Translation2d point;
    private final Pose2d rightPose;
    private final Pose2d leftPose; 

    private double distance;

    ReefScorePoints(String zone, Translation2d point, Pose2d rightPose, Pose2d leftPose, double distance) {
      this.zone = zone;
      this.point = point;
      this.rightPose = rightPose;
      this.leftPose = leftPose;
    }

    public String getZone() {
      return zone;
    }

    public Translation2d getPoint() {
      return point;
    }

    public Pose2d getRightPose() {
      return rightPose;
    }

    public Pose2d getLeftPose() {
      return leftPose;
    }

    public void setDistance(double distance) {
      this.distance = distance;
    }

    public double getDistance() {
      return distance;
    }
  }

  /** Creates a new Autos. */
  public Autos(LEDSubsystem ledSubsystem, Coral coralSubsystem, Elevator elevatorSubsystem, PoseEstimator poseEstimator) {
    m_ledSubsystem = ledSubsystem;
    m_coral = coralSubsystem;
    m_elevator = elevatorSubsystem;
    m_PoseEstimator = poseEstimator;

    loadPaths();
    }

  /**
   * Loads the paths from the specified path files.
   */
  private void loadPaths() {
    loadPath("ML-I");
    loadPath("I-CPR");
    loadPath("CPR-J");
    loadPath("MR-F");
    loadPath("L-I");
    loadPath("R-F");
    loadPath("M-H test");
  }

  /**
   * Loads a specific path from the given path name.
   *
   * @param pathName The name of the path file to load.
   */
  private void loadPath(String pathName) {
    try {
      paths.put(pathName, PathPlannerPath.fromPathFile(pathName));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  /**
   * Sets up the auto chooser with different autonomous options.
   */
  public void setupAutoChooser() {
    autoChooser.setDefaultOption("Path M-L4-H", "M_L4_H()");
    autoChooser.addOption("Path L-L4-I", "L_L4_I()");
    autoChooser.addOption("Path R_L4_I", "R_L4_F()");
    autoChooser.addOption("Path ML_L4_I", "ML_L4_I()");
    autoChooser.addOption("Path MR_L4_F", "MR_L4_F()");
    autoChooser.addOption("Min", "Min()");
    autoChooser.addOption("MultiPathTest", "MultiPathTest()");
  }

  /**
   * runs the autonomous command based on the selected option in the auto chooser.
   */
  public void selectAuto() {
    if (autoChooser.getSelected().equals("M_L4_H()")) {
      M_L4_H().schedule();
    } else if (autoChooser.getSelected().equals("L_L4_I()")) {
      L_L4_I().schedule();
    } else if (autoChooser.getSelected().equals("R_L4_F()")) {
      R_L4_F().schedule();
    } else if (autoChooser.getSelected().equals("MR_L4_F()")) {
      MR_L4_F().schedule();
    } else if (autoChooser.getSelected().equals("ML_L4_I()")) {
      ML_L4_I().schedule();
    } else if (autoChooser.getSelected().equals("Min()")) {
      Min().schedule();
    } else if(autoChooser.getSelected().equals("MultiPathTest()")) {
      MultiPathTest().schedule();
    } else {
      System.out.println("something is very wrong if you see this");
    }
  }

  /**
   * Returns the auto chooser.
   *
   * @return The SendableChooser object for selecting autonomous commands.
   */
  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
  }

  /**
   * Aligns the robot to a specified goal pose.
   *
   * @param goal The target pose to align to.
   * @return A command that aligns the robot to the specified pose.
   */
  public Command align(Pose2d goal) {
    return AutoBuilder.pathfindToPose(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel));
  }

  /**
   * Aligns the robot to a specified path then follows it.
   *
   * @param goal The target path to align to.
   * @return A command that aligns the robot to the specified path and follows it.
   */
  public Command alignToPath(PathPlannerPath goal) {
    return AutoBuilder.pathfindThenFollowPath(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel)).andThen(new WaitCommand(0.0001));
  }

  /**
   * Follows a specified path.
   *
   * @param path The path to follow.
   * @return A command that follows the specified path.
   */
  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  /**
   * Aligns the robot to the closest ReefScorePoints side, either to the left or right.
   * The method first determines the closest ReefScorePoints location and then
   * aligns the robot to either the left or right pose of that location.
   *
   * @param right A boolean indicating whether to align to the right (true) or left (false) pose.
   */
  public void alignToClosestSide(boolean right) {
    ReefScorePoints closest = findClosestSide();
    Pose2d goalPose = right ? closest.getRightPose() : closest.getLeftPose();
    if (Robot.isRedAlliance()) {
      goalPose = goalPose.rotateAround(kVision.fieldCenter, new Rotation2d(Math.toRadians(180)));
    }
    align(goalPose).schedule();
  }

  /**
   * Calculates the scoring pose based on the given rotation, distance from the reef, 
   * and whether the scoring is to the right or left side.
   *
   * @param rotation The rotation of the robot in radians.
   * @param distanceFromReef The distance from the reef to the scoring location.
   * @param right A boolean indicating whether to calculate for the right (true) or left (false) side.
   * @return A Pose2d representing the calculated scoring location.
   */
  private static Pose2d GetScorePose(Rotation2d rotation, double distanceFromReef, boolean right) {
    Translation2d reef = new Translation2d(4.4958, 4.026);
    double verticalOffset = 0.165; // offset from the reef to the scoring location
    if (!right) {
      verticalOffset = -verticalOffset; // flip the offset for the left side
    }
    Pose2d baseScoreLocation = 
      new Pose2d(
        reef.plus(
          new Translation2d(
            0.7 + kSwerve.width + distanceFromReef,
            verticalOffset)), 
        new Rotation2d(Math.toRadians(180)));
      return baseScoreLocation.rotateAround(reef, rotation);
  }

  /**
   * Finds the closest ReefScorePoints location to the robot's current position.
   * This method calculates the distance from the robot to each defined ReefScorePoints 
   * and determines which point has the minimum distance.
   *
   * @return The ReefScorePoints enum value representing the closest reef side
   *         to the robot's current location, with its distance value updated.
   */
  private ReefScorePoints findClosestSide() {
    Pose2d robotPose = m_PoseEstimator.get2dPose();
    ReefScorePoints[] points = ReefScorePoints.values();

    // Update distances and find minimum in a single pass
    ReefScorePoints closest = points[0];
    if (!Robot.isRedAlliance()) {
      closest.setDistance(robotPose.getTranslation().getDistance(closest.getPoint()));
    } else {
      closest.setDistance(robotPose.getTranslation().getDistance(closest.getPoint().rotateAround(kVision.fieldCenter, new Rotation2d(Math.toRadians(180)))));
    }

    for (int i = 1; i < points.length; i++) {
      double distance;
      if (!Robot.isRedAlliance()) {
        distance = robotPose.getTranslation().getDistance(points[i].getPoint());
      } else {
        distance = robotPose.getTranslation().getDistance(points[i].getPoint().rotateAround(kVision.fieldCenter, new Rotation2d(Math.toRadians(180))));
      }
      points[i].setDistance(distance);
      if (distance < closest.getDistance()) {
          closest = points[i];
      }
    }
    return closest;
  }

  /**
   * Sets the starting pose of the robot based on the given path.
   * The method checks the alliance color and sets the pose accordingly.
   *
   * @param path The PathPlannerPath object representing the path to set the starting pose for.
   */
  private void setStartPose(PathPlannerPath path) {
    Pose2d startPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) ==  Alliance.Red) {
      startPose = path.flipPath().getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    } else {
      startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    }
    if (Robot.isReal()) {
      m_PoseEstimator.resetPoseToPose2d(startPose);
    } else {
      m_PoseEstimator.resetPoseToPose2d(new Pose2d(startPose.getTranslation(), startPose.getRotation().plus(m_PoseEstimator.get2dPose().getRotation())));
    }
  }

  // Commands for different paths
  private Command Min() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("Min"))),
      alignToPath(paths.get("Min"))
    );
  }

  private Command MultiPathTest() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("L-I"))),
      followPath(paths.get("L-I")),
      followPath(paths.get("I-CPR")),
      followPath(paths.get("CPR-J")) 
    );
  }

  private SequentialCommandGroup M_L4_H() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("M-H test"))),
      followPath(paths.get("M-H test")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(1),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup L_L4_I() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("L-I"))),
      followPath(paths.get("L-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup R_L4_F() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("R-F"))),
      followPath(paths.get("R-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup MR_L4_F() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("MR-F"))),
      followPath(paths.get("MR-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup ML_L4_I() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("ML-I"))),
      followPath(paths.get("ML-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(.5),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsystem),
      new WaitCommand(.2)
      // new InstantCommand(() -> setStartPose(paths.get("I-CPR"))),
      // followPath(paths.get("I-CPR")),
      // new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsystem).withTimeout(1.5),
      // new InstantCommand(() -> setStartPose(paths.get("CPR-J"))),
      // followPath(paths.get("CPR-J")),
      // new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem),
      // new WaitCommand(.5),
      // new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.LinkedHashMap;

import org.json.simple.parser.ParseException;

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
import frc.robot.Constants.kAllianceInfo.RobotAlliance;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.Robot;
import frc.robot.commands.CoralCmds.DeployCoralCmd;
import frc.robot.commands.ElevatorCmds.SetElevatorHeightCmd;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.utilities.NetworkTableLogger;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;

public class Autos extends SubsystemBase {
  private final LEDSubsystem m_ledSubsystem;
  private final LEDPatterns m_ledPatterns;
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
    
    // Marked by zone as depicted below:
    //  
    //   ___________________Blue Alliance__________________ ... _____ 
    //            |  ⟋                        | |     | |        ⟍  | 
    //   _________|⟋         K-⟋  ⟍-J        | |     | |          ⟍|
    //            |        L-⟋      ⟍-I      | |     | |            |
    //   Alliance |       A |         | H     | |Barge| |            |
    //   Wall     |       B |         | G     | |     | |            |
    //   _________|        C-⟍      ⟋-F      | |     | |            |
    //            |⟍         D-⟍  ⟋-E        | |     | |          ⟋|
    //   _________|__⟍________________________|_|_____|_|__ ... _⟋__|

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

    // Getting translations and setting them to objects to avoid making new lists in `this.findClosestPoint()`
    private static List<Translation2d> translation2ds = 
      Arrays.asList(new Translation2d[] {
          A_B.point, C_D.point, E_F.point, G_H.point, I_J.point, K_L.point});

    // Set closest point preliminarily to null
    private static ReefScorePoints closestScorePoint = null;

    ReefScorePoints(String zone, Translation2d point, Pose2d rightPose, Pose2d leftPose, double distance) {
      this.zone = zone;
      this.point = point;
      this.rightPose = rightPose;
      this.leftPose = leftPose;
    }

    /**
     * Finds the ReefScorePoint with the closest Translation2d to the provided robot pose. 
     * Requires RobotAlliance value to determine if it needs to flip the alliance for red alliance
     * @param robotAlliance the alliance of this match/practice
     * @param robotPose the current robot pose on the field
     * @return the closest ReefScorePoint to the provided robot pose
     */
    public static ReefScorePoints findClosestScorePoint(RobotAlliance robotAlliance, Pose2d robotPose) {
          // Iterate through all ReefScorePoints to find the one that matches the closest translation point
        for (var aScorePoint : ReefScorePoints.values()) {
          // Find the closest translation point from the robot's current position and get the point with that component
          if (robotAlliance == RobotAlliance.RED_ALLIANCE) {
            if (aScorePoint.point == robotPose.getTranslation()
                .rotateAround(kVision.fieldCenter, new Rotation2d(Math.toRadians(180)))
                .nearest(translation2ds)) {
              closestScorePoint = aScorePoint;
              break;
            }
          }
          if (aScorePoint.point == robotPose.getTranslation().nearest(translation2ds)) {
            closestScorePoint = aScorePoint;
          }
        }

      // Return closest ReefScorePoint
      return closestScorePoint;
    }
    
    public ReefScorePoints getClosestPoint() {
      return closestScorePoint;
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
  }

  /** Creates a new Autos. */
  public Autos(LEDSubsystem ledSubsystem, LEDPatterns ledPatterns, Coral coralSubsystem, Elevator elevatorSubsystem, PoseEstimator poseEstimator) {
    m_ledSubsystem = ledSubsystem;
    m_ledPatterns = ledPatterns;
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
    autoChooser.addOption("bareMinimum", "bareMinimum()");
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
    } else if (autoChooser.getSelected().equals("bareMinimum()")) {
      bareMinimum().schedule();
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
  public ReefScorePoints findClosestSide() {

    // Get the robot's current pose
    Pose2d robotPose = m_PoseEstimator.get2dPose();

    // Get alliance (enum)
    RobotAlliance alliance = Robot.isRedAlliance() ? RobotAlliance.RED_ALLIANCE : RobotAlliance.BLUE_ALLIANCE;

    // Get closest ReefScorePoint (enum)
    ReefScorePoints closestScorePoint = ReefScorePoints.findClosestScorePoint(alliance, robotPose);

    // Log closest ReefScorePoint's zone
    logger.logString("Zone of closest ReefScorePoint", closestScorePoint.zone);

    return closestScorePoint;
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
  private Command bareMinimum() {
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
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
      new WaitCommand(1),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup L_L4_I() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("L-I"))),
      followPath(paths.get("L-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup R_L4_F() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("R-F"))),
      followPath(paths.get("R-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup MR_L4_F() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("MR-F"))),
      followPath(paths.get("MR-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator)
    );
  }

  private SequentialCommandGroup ML_L4_I() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("ML-I"))),
      followPath(paths.get("ML-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsystem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
      new WaitCommand(.5),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsystem, m_ledPatterns),
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

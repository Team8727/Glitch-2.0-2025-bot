// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.LinkedHashMap;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.Coral.DeployCoralCmd;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.commands.SetElevatorHeightCmd;

public class Autos extends SubsystemBase {
  private final LEDSubsystem m_ledSubsytem;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<String, PathPlannerPath>();
  private final PoseEstimatior m_PoseEstimatior;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getName().toString());

  public enum ReefScorePoints {
    A_B("A-B",
      new Translation2d(3.65, 4.02), // side point
      new Pose2d(), // right score location
      new Pose2d(), // left score location
      0), // distance to side

    C_D("C-D", 
      new Translation2d(4.07, 3.3), // side point
      new Pose2d(), // right score location
      new Pose2d(), // left score location
      0),  // distance to side

    E_F("E-F", 
      new Translation2d(4.9, 3.3), 
      new Pose2d(), 
      new Pose2d(),
      0),  // Zone E-F

    G_H("G-H", 
      new Translation2d(5.35, 4.02), 
      new Pose2d(), 
      new Pose2d(), 
      0),  // Zone G-H

    I_J("I-J", 
      new Translation2d(4.9, 4.75), 
      new Pose2d(4.977, 5.208, new Rotation2d(Math.toRadians(-120))), 
      new Pose2d(), 
      0),  // Zone I-J

    K_L("K-L", 
      new Translation2d(4.07, 4.75), 
      new Pose2d(), 
      new Pose2d(), 
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
  public Autos(LEDSubsystem ledSubsystem, Coral coralSubsystem, Elevator elevatorSubsystem, PoseEstimatior poseEstimatior) {
    m_ledSubsytem = ledSubsystem;
    m_coral = coralSubsystem;
    m_elevator = elevatorSubsystem;
    m_PoseEstimatior = poseEstimatior;

    loadPaths();

    PathPlannerAuto setelevatorheight = new PathPlannerAuto(new SetElevatorHeightCmd(ElevatorPosition.L4, elevatorSubsystem, coralSubsystem, ledSubsystem));

    // // Load a full Choreo trajectory as a PathPlannerPath
    // PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");
    // // Load a split Choreo trajectory as a PathPlannerPath, using the split point with index 1
    // PathPlannerPath exampleChoreoTrajSplit = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj", 1);

    /* Add paths to the hashmap using this format:
      paths.put("<Name>", PathPlannerPath.fromPathFile("<path file name>"));
      ex:
      paths.put("Crazy auto", swerve.followPathWithEventsCommand(paths.get("crazy_auto")));
    */
    
    //paths.put("EXAMPLE", PathPlannerPath.fromChoreoTrajectory("EXAMPLE"));
  }

  private void loadPaths() {
    loadPath("ML-I");
    loadPath("I-CPR");
    loadPath("CPR-J");
    loadPath("MR-F");
    loadPath("L-I");
    loadPath("R-F");
    loadPath("M-H test");
  }

  public void setupAutoChooser() {
    autoChooser.setDefaultOption("Path M-L4-H", "M_L4_H()");
    autoChooser.addOption("Path L-L4-I", "L_L4_I()");
    autoChooser.addOption("Path R_L4_I", "R_L4_F()");
    autoChooser.addOption("Path ML_L4_I", "ML_L4_I()");
    autoChooser.addOption("Path MR_L4_F", "MR_L4_F()");
    autoChooser.addOption("Min", "Min()");
  }

  public void selectAuto() {
    if (autoChooser.getSelected() == "M_L4_H()") {
      M_L4_H().schedule();
    } else if (autoChooser.getSelected() == "L_L4_I()") {
      L_L4_I().schedule();
    } else if (autoChooser.getSelected() == "R_L4_F()") {
      R_L4_F().schedule();
    } else if (autoChooser.getSelected() == "MR_L4_F()") {
      MR_L4_F().schedule();
    } else if (autoChooser.getSelected() == "ML_L4_I()") {
      ML_L4_I().schedule();
    } else if (autoChooser.getSelected() == "Min()") {
      Min().schedule();
    } else {
      System.out.println("somting is very wrong if you see this");
    }
  }
  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
  }

  private void loadPath(String pathName) {
    try {
    paths.put(pathName, PathPlannerPath.fromPathFile(pathName));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  public Command align(Pose2d goal) {
    return AutoBuilder.pathfindToPose(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel))
              .andThen(
                run(() -> m_ledSubsytem.setPatternForDuration(
                    m_ledSubsytem.rainbow, 
                    2)))
                    ;
  }

  public Command alignToPath(PathPlannerPath goal) {
    return AutoBuilder.pathfindThenFollowPath(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel)).andThen(new WaitCommand(0.0001));
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
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
    Pose2d robotPose = m_PoseEstimatior.get2dPose();
    ReefScorePoints[] points = ReefScorePoints.values();

        // Update distances and find minimum in a single pass
    ReefScorePoints closest = points[0];
    closest.setDistance(robotPose.getTranslation().getDistance(closest.getPoint()));
    
    for (int i = 1; i < points.length; i++) {
        double distance = robotPose.getTranslation().getDistance(points[i].getPoint());
        points[i].setDistance(distance);
        if (distance < closest.getDistance()) {
            closest = points[i];
        }
    }
    return closest;
  }

  /**
   * Aligns the robot to the closest ReefScorePoints side, either to the left or right.
   * The method first determines the closest ReefScorePoints location and then
   * aligns the robot to either the left or right pose of that location.
   *
   * @param right A boolean indicating whether to align to the right (true) or left (false) pose.
   * @return A Command that aligns the robot to the specified side of the closest ReefScorePoints.
   */
  public Command alignToClosestSide(boolean right) {
    ReefScorePoints closest = findClosestSide();
    Pose2d goalPose = right ? closest.getRightPose() : closest.getLeftPose();
    if (Robot.isRedAlliance()) {
      goalPose = goalPose.rotateAround(kVision.fieldCenter, new Rotation2d(Math.toRadians(180)));
    }
    return align(goalPose);
  }
  
  private void setStartPose(PathPlannerPath path) {
    Pose2d startPose;
    if (DriverStation.getAlliance().get() ==  Alliance.Red) {
      startPose = path.flipPath().getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    } else {
      startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    }
    m_PoseEstimatior.resetPoseToPose2d(startPose);
  }

  
  private Command Min() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("Min"))),
      alignToPath(paths.get("Min"))
    );
  }

  private SequentialCommandGroup M_L4_H() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("M-H test"))),
      followPath(paths.get("M-H test")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(1),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)
    );
  }

  private SequentialCommandGroup L_L4_I() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("L-I"))),
      followPath(paths.get("L-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)
    );
  }

  private SequentialCommandGroup R_L4_F() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("R-F"))),
      followPath(paths.get("R-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)
    );
  }

  private SequentialCommandGroup MR_L4_F() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("MR-F"))),
      followPath(paths.get("MR-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)
    );
  }

  private SequentialCommandGroup ML_L4_I() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("ML-I"))),
      followPath(paths.get("ML-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(.6),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(.5),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      new WaitCommand(.2)
      // new InstantCommand(() -> setStartPose(paths.get("I-CPR"))),
      // followPath(paths.get("I-CPR")),
      // new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem).withTimeout(1.5),
      // new InstantCommand(() -> setStartPose(paths.get("CPR-J"))),
      // followPath(paths.get("CPR-J")),
      // new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      // new WaitCommand(.5),
      // new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getAutonomousCommand() { // TODO: This is where our autonomous commands will be run, check to see if it works
    // An example command will be run in autonomous
    return 
    null;
  }
}

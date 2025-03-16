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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.Coral.DeployCoralCmd;
import frc.robot.commands.Coral.IntakeCoralCmd;
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
    try {
    paths.put("M-H test", PathPlannerPath.fromPathFile("M-H test"));

    loadPath("ML-I");
    loadPath("I-CPR");
    loadPath("CPR-J");
    loadPath("MR-F");
    loadPath("L-I");
    loadPath("R-F");
    
    paths.put("M-L4-H", PathPlannerPath.fromChoreoTrajectory("M-L4-H"));
    // paths.put("Red-M-L4-H", PathPlannerPath.fromChoreoTrajectory("M-L4-H").flipPath());
    // paths.put("H-PC", PathPlannerPath.fromChoreoTrajectory("H-PC"));
    // paths.put("H-Refill", PathPlannerPath.fromChoreoTrajectory("H-Refill"));
    paths.put("CL-L4-I", PathPlannerPath.fromChoreoTrajectory("CL-L4-I"));
    // paths.put("I-Refill", PathPlannerPath.fromChoreoTrajectory("I-Refill"));
    // paths.put("Refill-J", PathPlannerPath.fromChoreoTrajectory("Refill-J"));
    paths.put("CR-L4-F", PathPlannerPath.fromChoreoTrajectory("CR-L4-F"));
    // paths.put("F-Refill", PathPlannerPath.fromChoreoTrajectory("F-Refill"));
    // paths.put("Refill-E", PathPlannerPath.fromChoreoTrajectory("Refill-E"));
    // paths.put("M-L4-G", PathPlannerPath.fromChoreoTrajectory("M-L4-G"));
    // paths.put("G-Refill", PathPlannerPath.fromChoreoTrajectory("G-Refill"));
    // paths.put("J-Refill", PathPlannerPath.fromChoreoTrajectory("J-Refill"));
    // paths.put("betterMinimum", PathPlannerPath.fromChoreoTrajectory("bareminimum"));
    paths.put("Min", PathPlannerPath.fromChoreoTrajectory("Min"));
    // loadPath("E-Refill");
    // loadPath("ML-L4-I");
    // loadPath("MR-L4-F");
    // loadPath("bareminimum");
    loadPathChoreo("MR-L4-F");
    loadPathChoreo("ML-L4-I");
    // loadPath("M-M+");
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
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

  private void loadPathChoreo(String pathName) {
    try {
    paths.put(pathName, PathPlannerPath.fromChoreoTrajectory(pathName));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
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

package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralCmds.ReindexCoralCmd;
import frc.robot.commands.CoralCmds.RejectCoralCmd;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.BackCoralRoller;
import frc.robot.subsystems.Elevator.Coral.FrontCoralRoller;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.LEDSubsystem2025;
import frc.robot.pose.PoseEstimator; // updated package
import Glitch.Lib.Swerve.RevSwerve;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver2DefaultBindings implements ControllerBindings {
  private final RevSwerve m_SwerveSubsystem;
  private final PoseEstimator m_poseEstimator;
  private final GroundIntakePivot groundIntakePivot;
  private final GroundIntakeRollers groundIntakeRollers;
  private final FrontCoralRoller frontCoralRoller;
  private final BackCoralRoller backCoralRoller;
  private final Elevator m_elevator;
  private final LEDSubsystem2025 m_ledSubsytem;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;

  public Driver2DefaultBindings(
    RevSwerve swerveSubsystem,
    PoseEstimator poseEstimator,
    GroundIntakePivot groundIntakePivot,
    GroundIntakeRollers groundIntakeRollers,
    FrontCoralRoller frontCoralRoller,
    BackCoralRoller backCoralRoller,
    Elevator elevator,
    LEDSubsystem2025 ledSubsystem,
    AlgaeRemoverPivot algaeRemoverPivot,
    AlgaeRemoverRollers algaeRemoverRollers) {
    m_SwerveSubsystem = swerveSubsystem;
    m_poseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_AlgaeRemoverPivot = algaeRemoverPivot;
    m_AlgaeRemoverRollers = algaeRemoverRollers;
  }

  @Override
  public void bind(CommandXboxController controller) {
    //              Drive Commands
    // Zero heading
    controller.start().onTrue(new InstantCommand(m_poseEstimator::zeroHeading));

    //               Coral Commands
    // intake coral
    // controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
    // reindex coral
    controller.povRight().onTrue(new ReindexCoralCmd(backCoralRoller, frontCoralRoller, m_elevator, m_ledSubsytem));
    controller.povLeft().onTrue(new RejectCoralCmd(backCoralRoller, frontCoralRoller));
    //deploy coral
    // controller.leftBumper().onTrue(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator));

    //              Elevator Commands
    // // elevator L1
    // controller.x().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem));
    // // elevator L2
    // controller.a().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L2, m_elevator, m_coral, m_ledSubsytem));
    // // elevator L3
    // controller.b().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsytem));
    // // elevator L4
    // controller.y().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem));

    // zero elevator
    controller.leftStick().and(controller.rightStick()).onTrue(new InstantCommand(m_elevator::resetElevatorEncoders));

    //                Algae Commands
    // // Intake algae
    // controller.rightTrigger().whileTrue(new IntakeAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));
    // // deploy algae
    // controller.rightBumper().onTrue(new ScoreAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));

    // // Remove Algae A2
    // controller.povUp().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, ElevatorPosition.A3, m_elevator, m_ledSubsytem));
    // // Remove Algae A2
    // controller.povDown().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, ElevatorPosition.A2, m_elevator, m_ledSubsytem));

  }
}

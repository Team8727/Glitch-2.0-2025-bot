package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.CoralCmds.DeployCoralCmd;
import frc.robot.commands.CoralCmds.IntakeCoralCmd;
import frc.robot.commands.CoralCmds.ReindexCoralCmd;
import frc.robot.commands.CoralCmds.RejectCoralCmd;
import frc.robot.commands.DriveCmds.SwerveJoystickCmd;
import frc.robot.commands.ElevatorAlgaeCmds.weirdAlgaeIntakeCmd;
import frc.robot.commands.ElevatorAlgaeCmds.weirdAlgaeShootCmd;
import frc.robot.commands.ElevatorCmds.SetElevatorHeightCmd;
import frc.robot.commands.ElevatorCmds.ZeroElevator;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final PoseEstimator m_poseEstimator;
  private final AlgaeIntakePivot m_AlgaeIntakePivot;
  private final AlgaeIntakeRollers m_AlgaeIntakeRollers;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsytem;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;
  private final Autos m_autos;

  public Driver1DefaultBindings(
      SwerveSubsystem swerveSubsystem,
      PoseEstimator poseEstimator,
      AlgaeIntakePivot AlgaeIntakePivot,
      AlgaeIntakeRollers AlgaeIntakeRollers,
      Coral coral,
      Elevator elevator,
      LEDSubsystem ledSubsystem,
      AlgaeRemoverPivot algaeRemoverPivot,
      AlgaeRemoverRollers algaeRemoverRollers,
      Autos autos) {
    m_SwerveSubsystem = swerveSubsystem;
    m_poseEstimator = poseEstimator;
    m_AlgaeIntakePivot = AlgaeIntakePivot;
    m_AlgaeIntakeRollers = AlgaeIntakeRollers;
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_AlgaeRemoverPivot = algaeRemoverPivot;
    m_AlgaeRemoverRollers = algaeRemoverRollers;
    m_autos = autos;
  }

  @Override
  public void bind(CommandXboxController controller) {
    m_SwerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        m_SwerveSubsystem,
        m_elevator,
        controller::getLeftY,
        controller::getLeftX,
        controller::getRightX));

    //              Drive Commands
    // Zero heading
    controller.start().onTrue(new InstantCommand(m_poseEstimator::zeroHeading));

    //               Coral Commands
    // intake coral
    controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
    // deploy coral
    controller.leftBumper().onTrue(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator));

    // reindex coral
    controller.povRight().onTrue(new ReindexCoralCmd(m_coral, m_elevator, m_ledSubsytem));
    // reject coral
    controller.povLeft().onTrue(new RejectCoralCmd(m_coral));

    // auto align
    controller.leftBumper().onTrue(new InstantCommand(() -> m_autos.alignToClosestSide(true))); // Align to closest side when POV right is pressed
    controller.rightBumper().onTrue(new InstantCommand(() -> m_autos.alignToClosestSide(false))); // Align to closest side when POV left is pressed

    //              Elevator Commands
    // elevator L1
    controller.x().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem));
    // elevator L2
    controller.a().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L2, m_elevator, m_coral, m_ledSubsytem));
    // elevator L3
    controller.b().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsytem));
    // elevator L4
    controller.y().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem));

    // zero elevator
    controller.rightTrigger().and(controller.rightBumper()).onTrue(new ZeroElevator(m_elevator));

    // //                Algae Commands
    // // Intake algae
    // controller.rightTrigger().whileTrue(new IntakeAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));
    // // deploy algae
    // controller.rightBumper().onTrue(new ScoreAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));

    // Remove Algae A2
    controller.povDown().whileTrue(new weirdAlgaeIntakeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, ElevatorPosition.A3, m_elevator, m_ledSubsytem, m_coral));
    // Remove Algae A2
    controller.povUp().whileTrue(new weirdAlgaeShootCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers ,m_elevator, m_ledSubsytem, m_coral));
  }

  @Override
  public void unbind(CommandXboxController controller) {
    m_SwerveSubsystem.removeDefaultCommand();
}

}

package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import Glitch.Lib.Swerve.Swerve;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralCmds.DeployCoralCmd;
import frc.robot.commands.CoralCmds.IntakeCoralCmd;
import frc.robot.commands.CoralCmds.RejectCoralCmd;
import frc.robot.commands.DriveCmds.SwerveJoystickCmd;
import frc.robot.commands.ElevatorAlgaeCmds.RemoveAlgaeCmd;
import frc.robot.commands.ElevatorCmds.SetElevatorHeightCmd;
import frc.robot.commands.ElevatorCmds.ZeroElevator;
import frc.robot.commands.GroundCoralCmds.IntakeCoralGroundCmd;
import frc.robot.commands.GroundCoralCmds.ScoreCoralGroundCmd;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.PoseEstimator;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
  private final Swerve m_Swerve;
  private final PoseEstimator m_poseEstimator;
  private final GroundIntakePivot groundIntakePivot;
  private final GroundIntakeRollers groundIntakeRollers;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsytem;
  private final LEDPatterns m_ledPatterns;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;
  private final Autos m_autos;

  public Driver1DefaultBindings(
    Swerve swerve,
    PoseEstimator poseEstimator,
    GroundIntakePivot groundIntakePivot,
    GroundIntakeRollers groundIntakeRollers,
    Coral coral,
    Elevator elevator,
    LEDSubsystem ledSubsystem,
    LEDPatterns ledPatterns,
    AlgaeRemoverPivot algaeRemoverPivot,
    AlgaeRemoverRollers algaeRemoverRollers,
    Autos autos) {
    m_Swerve = swerve;
    m_poseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_ledPatterns = ledPatterns;
    m_AlgaeRemoverPivot = algaeRemoverPivot;
    m_AlgaeRemoverRollers = algaeRemoverRollers;
    m_autos = autos;
  }

  @Override
  public void bind(CommandXboxController controller) {
    m_Swerve.setDefaultCommand(
      new SwerveJoystickCmd(
        m_Swerve,
        m_elevator,
        controller::getLeftY,
        controller::getLeftX,
        controller::getRightX));

    // -=-=-=-=-=-=- Drive Commands -=-=-=-=-=-=-
      // Zero heading
      controller.start().onTrue(new InstantCommand(m_poseEstimator::zeroHeading));
      // Auto align right
      controller.rightBumper().and(controller.rightTrigger().negate()).onTrue(new InstantCommand(() -> m_autos.alignToClosestSide(true))); // Align to closest side when POV right is pressed
      // Auto align left
      controller.leftBumper().and(controller.leftTrigger().negate()).onTrue(new InstantCommand(() -> m_autos.alignToClosestSide(false))); // Align to closest side when POV left is pressed

    // -=-=-=-=-=-=- Coral Commands -=-=-=-=-=-=-
      // intake coral
      controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
      // deploy coral
      controller.rightTrigger().onTrue(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator));

//      // reindex coral
//      controller.povRight().onTrue(new ReindexCoralCmd(m_coral, m_elevator, m_ledSubsytem));
      // reject coral
      controller.povRight().onTrue(new RejectCoralCmd(m_coral));

      // Ground intake coral
      controller.povLeft().whileTrue(new IntakeCoralGroundCmd(groundIntakeRollers, groundIntakePivot, m_ledSubsytem));
      // Ground deploy coral
      controller.povRight().onTrue(new ScoreCoralGroundCmd(groundIntakePivot, groundIntakeRollers, m_ledSubsytem));
    // -=-=-=-=-=-=- Elevator Commands -=-=-=-=-=-=-
      // Elevator L1
      controller.x().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem, m_ledPatterns).andThen(new PrintCommand("hihih")));
      // Elevator L2
      controller.a().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L2, m_elevator, m_coral, m_ledSubsytem, m_ledPatterns));
      // Elevator L3
      controller.b().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsytem, m_ledPatterns));
      // Elevator L4
      controller.y().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem, m_ledPatterns));

      // Zero elevator (back button is the left small button on the controller near the top)
      controller.back().onTrue(new ZeroElevator(m_elevator));

    // -=-=-=-=-=-=- Algae Commands -=-=-=-=-=-=-
      // Remove Algae A2
      controller.povDown().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, Elevator.ElevatorPosition.A2, m_elevator, m_ledSubsytem));
      // shoot algae
      controller.povUp().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, Elevator.ElevatorPosition.A3, m_elevator, m_ledSubsytem));
    // -=-=-=-=-=-=-+-=-=-=-=-=-=-+-=-=-=-=-=-=-
  }

  @Override
  public void unbind(CommandXboxController controller) {
    m_Swerve.removeDefaultCommand();
}

}

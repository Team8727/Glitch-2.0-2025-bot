package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
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
import frc.robot.subsystems.CTRESwerveDrivetrain;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.BackCoralRoller;
import frc.robot.subsystems.Elevator.Coral.FrontCoralRoller;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.pose.PoseEstimator; // updated package
import Glitch.Lib.Swerve.RevSwerve;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
  private final PoseEstimator m_poseEstimator;
  private final GroundIntakePivot groundIntakePivot;
  private final GroundIntakeRollers groundIntakeRollers;
  private final BackCoralRoller backCoralRoller;
  private final FrontCoralRoller frontCoralRoller;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsytem;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;
  private final Autos m_autos;

  public Driver1DefaultBindings(
      CTRESwerveDrivetrain drivetrain,
      PoseEstimator poseEstimator,
      GroundIntakePivot groundIntakePivot,
      GroundIntakeRollers groundIntakeRollers,
      BackCoralRoller backCoralRoller,
      FrontCoralRoller frontCoralRoller,
      Elevator elevator,
      LEDSubsystem ledSubsystem,
      LEDPatterns ledPatterns,
      AlgaeRemoverPivot algaeRemoverPivot,
      AlgaeRemoverRollers algaeRemoverRollers,
      Autos autos,
      CommandXboxController controller) {
    m_poseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_AlgaeRemoverPivot = algaeRemoverPivot;
    m_AlgaeRemoverRollers = algaeRemoverRollers;
    m_autos = autos;

    new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  public Driver1DefaultBindings(
      RevSwerve swerveSubsystem,
      PoseEstimator poseEstimator,
      GroundIntakePivot groundIntakePivot,
      GroundIntakeRollers groundIntakeRollers,
      FrontCoralRoller frontCoralRoller,
      BackCoralRoller backCoralRoller,
      Elevator elevator,
      LEDSubsystem ledSubsystem,
      LEDPatterns ledPatterns,
      AlgaeRemoverPivot algaeRemoverPivot,
      AlgaeRemoverRollers algaeRemoverRollers,
      Autos autos,
      CommandXboxController controller) {
    m_poseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_AlgaeRemoverPivot = algaeRemoverPivot;
    m_AlgaeRemoverRollers = algaeRemoverRollers;
    m_autos = autos;

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem,
        m_elevator,
        controller::getLeftY,
        controller::getLeftX,
        controller::getRightX));

    bind(controller);
  }


  @Override
  public void bind(CommandXboxController controller) {
    // -=-=-=-=-=-=- Drive Commands -=-=-=-=-=-=-
      // Zero heading
      controller.start().onTrue(new InstantCommand(m_poseEstimator::zeroHeading));
      // Auto align right
      controller.rightBumper().and(controller.rightTrigger().negate()).onTrue(new InstantCommand(() -> m_autos.alignToClosestSide(true))); // Align to closest side when POV right is pressed
      // Auto align left
      controller.leftBumper().and(controller.leftTrigger().negate()).onTrue(new InstantCommand(() -> m_autos.alignToClosestSide(false))); // Align to closest side when POV left is pressed

    // -=-=-=-=-=-=- Coral Commands -=-=-=-=-=-=-
      // intake coral
      controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(backCoralRoller, frontCoralRoller, m_elevator, m_ledSubsytem));
      // deploy coral
      controller.rightTrigger().onTrue(new DeployCoralCmd(frontCoralRoller, backCoralRoller, m_ledSubsytem, m_elevator));

//      // reindex coral
//      controller.povRight().onTrue(new ReindexCoralCmd(m_coral, m_elevator, m_ledSubsytem));
      // reject coral
      controller.povRight().onTrue(new RejectCoralCmd(backCoralRoller, frontCoralRoller));

      // Ground intake coral
      controller.povLeft().whileTrue(new IntakeCoralGroundCmd(groundIntakeRollers, groundIntakePivot, m_ledSubsytem));
      // Ground deploy coral
      controller.povRight().onTrue(new ScoreCoralGroundCmd(groundIntakePivot, groundIntakeRollers, m_ledSubsytem));
    // -=-=-=-=-=-=- Elevator Commands -=-=-=-=-=-=-
      // Elevator L1
      controller.x().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L1, m_elevator, frontCoralRoller, m_ledSubsytem).andThen(new PrintCommand("hihih")));
      // Elevator L2
      controller.a().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L2, m_elevator, frontCoralRoller, m_ledSubsytem));
      // Elevator L3
      controller.b().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L3, m_elevator, frontCoralRoller, m_ledSubsytem));
      // Elevator L4
      controller.y().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L4, m_elevator, frontCoralRoller, m_ledSubsytem));

      // Zero elevator (back button is the left small button on the controller near the top)
      controller.back().onTrue(new ZeroElevator(m_elevator));

    // -=-=-=-=-=-=- Algae Commands -=-=-=-=-=-=-
      // Remove Algae A2
      controller.povDown().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, Elevator.ElevatorPosition.A2, m_elevator, m_ledSubsytem));
      // shoot algae
      controller.povUp().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, Elevator.ElevatorPosition.A3, m_elevator, m_ledSubsytem));
    // -=-=-=-=-=-=-+-=-=-=-=-=-=-+-=-=-=-=-=-=-
  }
}

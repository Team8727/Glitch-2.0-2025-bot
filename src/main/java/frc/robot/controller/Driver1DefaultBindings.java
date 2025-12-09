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
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.pose.PoseEstimator; // updated package
import Glitch.Lib.Swerve.RevSwerve;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
  private final PoseEstimator poseEstimator;
  private final GroundIntakePivot groundIntakePivot;
  private final GroundIntakeRollers groundIntakeRollers;
  private final BackCoralRoller backCoralRoller;
  private final FrontCoralRoller frontCoralRoller;
  private final Elevator elevator;
  private final LEDSubsystem ledSubsystem;
  private final AlgaeRemoverPivot algaeRemoverPivot;
  private final AlgaeRemoverRollers algaeRemoverRollers;
  private final Autos autos;

  public Driver1DefaultBindings(
      CTRESwerveDrivetrain drivetrain,
      PoseEstimator poseEstimator,
      GroundIntakePivot groundIntakePivot,
      GroundIntakeRollers groundIntakeRollers,
      BackCoralRoller backCoralRoller,
      FrontCoralRoller frontCoralRoller,
      Elevator elevator,
      LEDSubsystem ledSubsystem,
      AlgaeRemoverPivot algaeRemoverPivot,
      AlgaeRemoverRollers algaeRemoverRollers,
      Autos autos,
      CommandXboxController controller) {
    this.poseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    this.elevator = elevator;
    this.ledSubsystem = ledSubsystem;
    this.algaeRemoverPivot = algaeRemoverPivot;
    this.algaeRemoverRollers = algaeRemoverRollers;
    this.autos = autos;

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
      AlgaeRemoverPivot algaeRemoverPivot,
      AlgaeRemoverRollers algaeRemoverRollers,
      Autos autos,
      CommandXboxController controller) {
    this.poseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    this.elevator = elevator;
    this.ledSubsystem = ledSubsystem;
    this.algaeRemoverPivot = algaeRemoverPivot;
    this.algaeRemoverRollers = algaeRemoverRollers;
    this.autos = autos;

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem,
        this.elevator,
        controller::getLeftY,
        controller::getLeftX,
        controller::getRightX));

    bind(controller);
  }


  @Override
  public void bind(CommandXboxController controller) {
    // -=-=-=-=-=-=- Drive Commands -=-=-=-=-=-=-
      // Zero heading
      controller.start().onTrue(new InstantCommand(poseEstimator::zeroHeading));
      // Auto align right
      controller.rightBumper().and(controller.rightTrigger().negate()).onTrue(new InstantCommand(() -> autos.alignToClosestSide(true))); // Align to closest side when POV right is pressed
      // Auto align left
      controller.leftBumper().and(controller.leftTrigger().negate()).onTrue(new InstantCommand(() -> autos.alignToClosestSide(false))); // Align to closest side when POV left is pressed

    // -=-=-=-=-=-=- Coral Commands -=-=-=-=-=-=-
      // intake coral
      controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(backCoralRoller, frontCoralRoller, elevator, ledSubsystem));
      // deploy coral
      controller.rightTrigger().onTrue(new DeployCoralCmd(frontCoralRoller, backCoralRoller, ledSubsystem, elevator));

//      // reindex coral
//      controller.povRight().onTrue(new ReindexCoralCmd(m_coral, elevator, ledSubsystem));
      // reject coral
      controller.povRight().onTrue(new RejectCoralCmd(backCoralRoller, frontCoralRoller));

      // Ground intake coral
      controller.povLeft().whileTrue(new IntakeCoralGroundCmd(groundIntakeRollers, groundIntakePivot, ledSubsystem));
      // Ground deploy coral
      controller.povRight().onTrue(new ScoreCoralGroundCmd(groundIntakePivot, groundIntakeRollers, ledSubsystem));
    // -=-=-=-=-=-=- Elevator Commands -=-=-=-=-=-=-
      // Elevator L1
      controller.x().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L1, elevator, frontCoralRoller, ledSubsystem).andThen(new PrintCommand("hihih")));
      // Elevator L2
      controller.a().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L2, elevator, frontCoralRoller, ledSubsystem));
      // Elevator L3
      controller.b().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L3, elevator, frontCoralRoller, ledSubsystem));
      // Elevator L4
      controller.y().onTrue(new SetElevatorHeightCmd(Elevator.ElevatorPosition.L4, elevator, frontCoralRoller, ledSubsystem));

      // Zero elevator (back button is the left small button on the controller near the top)
      controller.back().onTrue(new ZeroElevator(elevator));

    // -=-=-=-=-=-=- Algae Commands -=-=-=-=-=-=-
      // Remove Algae A2
      controller.povDown().whileTrue(new RemoveAlgaeCmd(algaeRemoverPivot, algaeRemoverRollers, Elevator.ElevatorPosition.A2, elevator));
      // shoot algae
      controller.povUp().whileTrue(new RemoveAlgaeCmd(algaeRemoverPivot, algaeRemoverRollers, Elevator.ElevatorPosition.A3, elevator));
    // -=-=-=-=-=-=-+-=-=-=-=-=-=-+-=-=-=-=-=-=-
  }
}

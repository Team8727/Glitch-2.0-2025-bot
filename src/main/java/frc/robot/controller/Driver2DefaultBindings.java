package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Coral.ReindexCoralCmd;
import frc.robot.commands.Coral.RejectCoralCmd;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver2DefaultBindings implements ControllerBindings {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final PoseEstimatior m_poseEstimator;
    private final AlgaeIntakePivot m_AlgaeIntakePivot;
    private final AlgaeIntakeRollers m_AlgaeIntakeRollers;
    private final Coral m_coral;
    private final Elevator m_elevator;
    private final LEDSubsystem m_ledSubsytem;
    private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
    private final AlgaeRemoverRollers m_AlgaeRemoverRollers;

    public Driver2DefaultBindings(
        SwerveSubsystem swerveSubsystem,
        PoseEstimatior poseEstimator,
        AlgaeIntakePivot AlgaeIntakePivot,
        AlgaeIntakeRollers AlgaeIntakeRollers,
        Coral coral,
        Elevator elevator,
        LEDSubsystem ledSubsystem,
        AlgaeRemoverPivot algaeRemoverPivot,
        AlgaeRemoverRollers algaeRemoverRollers) {
        m_SwerveSubsystem = swerveSubsystem;
        m_poseEstimator = poseEstimator;
        m_AlgaeIntakePivot = AlgaeIntakePivot;
        m_AlgaeIntakeRollers = AlgaeIntakeRollers;
        m_coral = coral;
        m_elevator = elevator;
        m_ledSubsytem = ledSubsystem;
        m_AlgaeRemoverPivot = algaeRemoverPivot;
        m_AlgaeRemoverRollers = algaeRemoverRollers;
    }

    @Override
    public void bind(CommandXboxController controller) {
        //              Drive Commands
        // Zero heading
        controller.start().onTrue(new InstantCommand(() -> m_poseEstimator.zeroHeading()));

        //               Coral Commands
        // intake coral
        // controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
        // reindex coral
        controller.povRight().onTrue(new ReindexCoralCmd(m_coral, m_elevator, m_ledSubsytem));
        controller.povLeft().onTrue(new RejectCoralCmd(m_coral));
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
        controller.leftStick().and(controller.rightStick()).onTrue(new InstantCommand(() -> m_elevator.resetElevatorEncoders()));
        
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

    @Override
    public void unbind(CommandXboxController controller) {
        m_SwerveSubsystem.removeDefaultCommand();
    }
    
}

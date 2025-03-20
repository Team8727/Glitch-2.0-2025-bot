// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controller.Controller;
import frc.robot.controller.Driver1DefaultBindings;
import frc.robot.controller.Driver2DefaultBindings;
import frc.robot.subsystems.Autos;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem;
  private final PoseEstimatior m_PoseEstimator;
  private final AlgaeIntakePivot m_AlgaeIntakePivot;
  private final AlgaeIntakeRollers m_AlgaeIntakeRollers;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsytem;
  private final Autos m_Autos;
  private final Controller m_mainController = new Controller(Controller.Operator.MAIN);
  private final Controller m_assistController = new Controller(Controller.Operator.ASSIST);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(
      SwerveSubsystem swerveSubsystem,
      PoseEstimatior poseEstimator,
      AlgaeIntakePivot AlgaeIntakePivot,
      AlgaeIntakeRollers AlgaeIntakeRollers,
      AlgaeRemoverPivot AlgaeRemoverPivot,
      AlgaeRemoverRollers AlgaeRemoverRollers,
      Coral coral,
      Elevator elevator,
      LEDSubsystem ledSubsystem,
      Autos autos
      ) {
    m_SwerveSubsystem = swerveSubsystem;
    m_PoseEstimator = poseEstimator;
    m_AlgaeIntakePivot = AlgaeIntakePivot;
    m_AlgaeIntakeRollers = AlgaeIntakeRollers;
    m_AlgaeRemoverPivot = AlgaeRemoverPivot;
    m_AlgaeRemoverRollers = AlgaeRemoverRollers;
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_Autos = autos;

    m_Autos.setupAutoChooser();

  }

  public void teleopInit() {
    m_mainController.applyBindings(
      new Driver1DefaultBindings(
        m_SwerveSubsystem,
        m_PoseEstimator,
        m_AlgaeIntakePivot,
        m_AlgaeIntakeRollers,
        m_coral,
        m_elevator,
        m_ledSubsytem,
        m_AlgaeRemoverPivot,
        m_AlgaeRemoverRollers,
        m_Autos
      )
    );
    
    m_assistController.applyBindings(
      new Driver2DefaultBindings(
        m_SwerveSubsystem,
        m_PoseEstimator,
        m_AlgaeIntakePivot,
        m_AlgaeIntakeRollers,
        m_coral,
        m_elevator,
        m_ledSubsytem,
        m_AlgaeRemoverPivot,
        m_AlgaeRemoverRollers
      )
    );
  }

  public void autonomousInit() {
    m_mainController.clearBindings();
    m_assistController.clearBindings();
  }
}

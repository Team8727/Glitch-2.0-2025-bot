// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.Controller.Controller;
import frc.robot.controller.Driver1DefaultBindings;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.BackCoralRoller;
import frc.robot.subsystems.Elevator.Coral.FrontCoralRoller;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.pose.PoseEstimator; // updated import
import Glitch.Lib.Swerve.RevSwerve;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The RobotContainer class is responsible for defining the structure of the robot.
 * It initializes subsystems, commands, and controller bindings.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain drivetrain; // Command-based swerve drivetrain subsystem
  private final RevSwerve m_SwerveSubsystem; // Swerve subsystem
  private final PoseEstimator m_PoseEstimator; // Pose estimation subsystem
  private final GroundIntakePivot groundIntakePivot; // Ground intake pivot subsystem
  private final GroundIntakeRollers groundIntakeRollers; // Ground intake rollers subsystem
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot; // Algae remover pivot subsystem
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers; // Algae remover rollers subsystem
  private final FrontCoralRoller frontCoralRoller; // front coral roller subsystem
  private final BackCoralRoller backCoralRoller; // back coral roller subsystem
  private final Elevator m_elevator; // Elevator subsystem
  private final LEDSubsystem m_ledSubsystem; // LED subsystem
  private final LEDPatterns m_ledPatterns; // LED patterns subsystem
  private final Autos m_Autos; // Autonomous routines subsystem
  private final Controller m_mainController = new Controller(Controller.Operator.MAIN); // Main controller
  private final Controller m_assistController = new Controller(Controller.Operator.ASSIST); // Assist controller

  /**
   * Constructor for RobotContainer when using a REV Swerve subsystem.
   *
   * @param swerveSubsystem The swerve subsystem
   * @param poseEstimator The pose estimator subsystem
   * @param groundIntakePivot The ground intake pivot subsystem
   * @param groundIntakeRollers The ground intake rollers subsystem
   * @param AlgaeRemoverPivot The algae remover pivot subsystem
   * @param AlgaeRemoverRollers The algae remover rollers subsystem
   * @param elevator The elevator subsystem
   * @param ledSubsystem The LED subsystem
   * @param ledPatterns The LED patterns subsystem
   * @param autos The autonomous routines subsystem
   */
  public RobotContainer(
    RevSwerve swerveSubsystem,
    PoseEstimator poseEstimator,
    GroundIntakePivot groundIntakePivot,
    GroundIntakeRollers groundIntakeRollers,
    AlgaeRemoverPivot AlgaeRemoverPivot,
    AlgaeRemoverRollers AlgaeRemoverRollers,
    FrontCoralRoller frontCoralRoller,
    BackCoralRoller backCoralRoller,
    Elevator elevator,
    LEDSubsystem ledSubsystem,
    LEDPatterns ledPatterns,
    Autos autos
  ) {
    this.drivetrain = null;
    m_SwerveSubsystem = swerveSubsystem;
    m_PoseEstimator = poseEstimator;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    m_AlgaeRemoverPivot = AlgaeRemoverPivot;
    m_AlgaeRemoverRollers = AlgaeRemoverRollers;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    m_ledPatterns = ledPatterns;
    m_Autos = autos;

    // Setup the autonomous chooser
    m_Autos.setupAutoChooser();
  }

  /**
   * Constructor for RobotContainer when using a CTRE drivetrain.
   *
   * @param drivetrain The command-based drivetrain subsystem
   * @param groundIntakePivot The ground intake pivot subsystem
   * @param groundIntakeRollers The ground intake rollers subsystem
   * @param AlgaeRemoverPivot The algae remover pivot subsystem
   * @param AlgaeRemoverRollers The algae remover rollers subsystem
   * @param elevator The elevator subsystem
   * @param ledSubsystem The LED subsystem
   * @param ledPatterns The LED patterns subsystem
   * @param autos The autonomous routines subsystem
   */
  public RobotContainer(
    CommandSwerveDrivetrain drivetrain,
    GroundIntakePivot groundIntakePivot,
    GroundIntakeRollers groundIntakeRollers,
    AlgaeRemoverPivot AlgaeRemoverPivot,
    AlgaeRemoverRollers AlgaeRemoverRollers,
    FrontCoralRoller frontCoralRoller,
    BackCoralRoller backCoralRoller,
    Elevator elevator,
    LEDSubsystem ledSubsystem,
    LEDPatterns ledPatterns,
    Autos autos
  ) {
    this.drivetrain = drivetrain;
    m_SwerveSubsystem = null;
    m_PoseEstimator = null;
    this.groundIntakePivot = groundIntakePivot;
    this.groundIntakeRollers = groundIntakeRollers;
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    m_AlgaeRemoverPivot = AlgaeRemoverPivot;
    m_AlgaeRemoverRollers = AlgaeRemoverRollers;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    m_ledPatterns = ledPatterns;
    m_Autos = autos;

    // Setup the autonomous chooser
    m_Autos.setupAutoChooser();
  }

  /**
   * Initializes the robot for teleoperated mode.
   * Applies bindings for the main controller.
   */
  public void teleopInit() {
    m_mainController.applyBindings(
      new Driver1DefaultBindings(
        m_SwerveSubsystem,
        m_PoseEstimator,
        groundIntakePivot,
        groundIntakeRollers,
        frontCoralRoller,
        backCoralRoller,
        m_elevator,
        m_ledSubsystem,
        m_ledPatterns,
        m_AlgaeRemoverPivot,
        m_AlgaeRemoverRollers,
        m_Autos,
        m_mainController.getController()
      )
    );
    // Uncomment the following block to apply bindings for the assist controller
    /*
    m_assistController.applyBindings(
      new Driver2DefaultBindings(
        m_SwerveSubsystem,
        m_PoseEstimator,
        groundIntakePivot,
        groundIntakeRollers,
        m_coral,
        m_elevator,
        m_ledSubsystem,
        m_AlgaeRemoverPivot,
        m_AlgaeRemoverRollers
      )
    );
    */
  }

  /**
   * Initializes the robot for autonomous mode.
   * Clears bindings for both controllers.
   */
  public void autonomousInit() {
    m_mainController.clearBindings();
    m_assistController.clearBindings();
  }
}

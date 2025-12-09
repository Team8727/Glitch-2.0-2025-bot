// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.NetworkTableLogger;
import Glitch.Lib.Swerve.RevSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.BackCoralRoller;
import frc.robot.subsystems.Elevator.Coral.FrontCoralRoller;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.GlitchLEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.pose.PoseEstimator;
import frc.robot.vision.Vision;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;
//  private final CommandSwerveDrivetrain CTREdrivetrain = TunerConstants.createDrivetrain();
  private final RevSwerve m_SwerveSubsystem = new RevSwerveSubsystem();
  private final Vision m_Vision = new Vision();
  private final PoseEstimator m_PoseEstimator = new PoseEstimator(m_SwerveSubsystem, m_Vision);
  private final Elevator m_elevator = new Elevator();
  private final LEDSubsystem m_ledSubsystem = LEDSubsystem.getInstance();
  private final GlitchLEDPatterns m_ledPatterns = new GlitchLEDPatterns();
  private final NetworkTableLogger logger = new NetworkTableLogger("SHOW UPPPP");
  private final AlgaeRemoverRollers m_AlgeaRemoverRollers = new AlgaeRemoverRollers();
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot = new AlgaeRemoverPivot();
  private final FrontCoralRoller frontCoralRoller = new FrontCoralRoller();
  private final BackCoralRoller backCoralRoller = new BackCoralRoller();
  private final GroundIntakePivot groundIntakePivot = new GroundIntakePivot();
  private final GroundIntakeRollers groundIntakeRollers = new GroundIntakeRollers();
  private final Autos m_Autos = new Autos(m_ledSubsystem, frontCoralRoller, backCoralRoller, m_elevator, m_PoseEstimator);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Configure PathPlanner's AutoBuilder
    try {
      AutoBuilder.configure(
          m_PoseEstimator::get2dPose,
          m_PoseEstimator::resetPoseToPose2d,
          m_SwerveSubsystem::getChassisSpeeds,
          (chassisSpeeds, driveff) -> { // drive command
            // INVERT IF THINGS ARE GOING BACKWARDS
            // if (Robot.isRedAlliance()) {
            //   chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
            // }
            m_SwerveSubsystem.setChassisSpeeds(chassisSpeeds);
          },
          new PPHolonomicDriveController(
            new PIDConstants(
              8,
              0,
              0),
            new PIDConstants(
              4,
              0,
              0)),
        RobotConfig.fromGUISettings(),
        Robot::isRedAlliance,
        // requirements
        m_SwerveSubsystem,
        m_PoseEstimator);
    } catch (IOException | ParseException e) {
      System.out.println("ERROR: Could not process pathplanner config");
      throw new RuntimeException(e);
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer =
        new RobotContainer(
          m_SwerveSubsystem,
          m_PoseEstimator,
          groundIntakePivot,
          groundIntakeRollers,
          m_AlgaeRemoverPivot,
          m_AlgeaRemoverRollers,
          frontCoralRoller,
          backCoralRoller,
          m_elevator,
          m_ledSubsystem,
          m_Autos
            );
    
    // Load autos into chooser and use SmartDashboard to publish
    m_Autos.setupAutoChooser();
    SmartDashboard.putData("Auto choices", m_Autos.getAutoChooser());

    // Set Up PathPlanner to "warm up" the pathplanning system
    PathfindingCommand.warmupCommand().schedule();

    // Log data to a log file using WPILib's DataLogManager
    DataLogManager.logNetworkTables(true);
    DataLogManager.start();

    // Start the URCL logger (logs REV SparkMaxes and SparkFlexes automatically on networktables)
    URCL.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    logger.logDouble("voltage", RobotController.getInputVoltage());

    // m_elevatorSpeedControl = logger.getBoolean("Limit Speed", true);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_ledSubsystem.setPattern(GlitchLEDPatterns.purple);
  }

  /** This function is called periodically during disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.autonomousInit();
    m_ledSubsystem.setPattern(GlitchLEDPatterns.rainbow);

    m_Autos.selectAuto(); //Only enable this if you want the robot to do stuff during autonomous

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    m_ledSubsystem.setPattern(GlitchLEDPatterns.fire(LEDSubsystem.defaultPattern));
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_robotContainer.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // logger.logPose2d("closest", new Pose2d(Autos.ReefScorePoints.findClosestScorePoint(isRedAlliance() ? RobotAlliance.RED_ALLIANCE : RobotAlliance.BLUE_ALLIANCE, m_PoseEstimator.get2dPose()).getPoint(), new Rotation2d()).rotateAround(kVision.fieldCenter, isRedAlliance() ? new Rotation2d(Math.toRadians(180)) : new Rotation2d(Math.toRadians(0))));
    // logger.logString("zone of closest", Autos.ReefScorePoints.findClosestScorePoint(null, m_PoseEstimator.get2dPose()).getZone());
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}

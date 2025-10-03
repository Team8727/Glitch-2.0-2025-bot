package frc.robot.commands.DriveCmds;

import Glitch.Lib.NetworkTableLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Elevator;
import Glitch.Lib.Swerve.RevSwerve;

import java.util.function.Supplier;

public class SwerveJoystickCmd extends Command {

  // Deadzone values
  private final double translationDeadzone = 0.08;
  private final double rotationDeadzone = 0.08;

  // Maximum speeds
  private final double maxTransSpeed = 5;
  private final double maxAngSpeed = 3 * Math.PI;
  private final double minimumDriveSpeed = 4;


  private final RevSwerve m_SwerveSubsystem;
  private final Elevator m_Elevator;
  private final Supplier<Double> m_ySpdFunction, m_xSpdFunction, m_turningSpdFunction;
  private final NetworkTableLogger m_logger = new NetworkTableLogger(this.getName());

  public SwerveJoystickCmd(
      RevSwerve swerveSubsystem,
      Elevator elevator,
      Supplier<Double> ySpdFunction,
      Supplier<Double> xSpdFunction,
      Supplier<Double> turningSpdFunction) {
    m_SwerveSubsystem = swerveSubsystem;
    m_Elevator = elevator;
    m_ySpdFunction = ySpdFunction;
    m_xSpdFunction = xSpdFunction;
    m_turningSpdFunction = turningSpdFunction;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // get joystick values
    double xSpeed = m_ySpdFunction.get();
    double ySpeed = m_xSpdFunction.get();
    double turningSpeed = m_turningSpdFunction.get();

    // apply deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, translationDeadzone);
    ySpeed = MathUtil.applyDeadband(ySpeed, translationDeadzone);
    turningSpeed = MathUtil.applyDeadband(turningSpeed, rotationDeadzone);


    // get elevator height for anti-tipping
    double elevatorHeight = m_Elevator.getElevatorHeight();
    double driveSpeedConversionFactor = (Elevator.ElevatorPosition.L4.getOutputRotations() - (elevatorHeight - minimumDriveSpeed)) / Elevator.ElevatorPosition.L4.getOutputRotations();
    xSpeed = -(xSpeed * maxTransSpeed
     * driveSpeedConversionFactor);  // Scaling to elevator height
    ySpeed = -(ySpeed * maxTransSpeed
     * driveSpeedConversionFactor);  // Scaling to elevator height
    turningSpeed = -(turningSpeed * maxAngSpeed
     * driveSpeedConversionFactor);  // Scaling to elevator height

    // set chassis speed
    ChassisSpeeds chassisSpeeds =
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, turningSpeed, m_SwerveSubsystem.getHeading());

    // Set the swerve module states
    m_SwerveSubsystem.setChassisSpeeds(chassisSpeeds);
    m_logger.logChassisSpeeds("chassis speeds", chassisSpeeds);

    // Update the sim rotation
    if (Robot.isSimulation()) {
      m_SwerveSubsystem.applySimHeading();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

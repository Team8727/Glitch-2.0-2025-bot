package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.DriveSpeedScaling;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.utilities.NetworkTableLogger;

import java.util.function.Supplier;

public class SwerveJoystickCmd extends Command {

  private final SwerveSubsystem m_SwerveSubsystem;
  private final Elevator m_Elevator;
  private final Supplier<Double> m_ySpdFunction, m_xSpdFunction, m_turningSpdFunction;
  private final NetworkTableLogger m_logger = new NetworkTableLogger(this.getName().toString());

  public SwerveJoystickCmd(
      SwerveSubsystem swerveSubsystem,
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
    double xSpeed = -m_ySpdFunction.get();
    double ySpeed = -m_xSpdFunction.get();
    double turningSpeed = m_turningSpdFunction.get();

    // apply deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, kOI.translationDeadzone);
    ySpeed = MathUtil.applyDeadband(ySpeed, kOI.translationDeadzone);
    turningSpeed = MathUtil.applyDeadband(turningSpeed, kOI.rotationDeadzone);


    // get elevator height for anti-tipping
    double elevatorHeight = m_Elevator.getElevatorHeight();
    double driveSpeedConversionFactor = (kElevator.ElevatorPosition.L4.getOutputRotations() - (elevatorHeight - DriveSpeedScaling.minimumDriveSpeed)) / kElevator.ElevatorPosition.L4.getOutputRotations();
    xSpeed = -(xSpeed * kSwerve.maxTransSpeed
     * driveSpeedConversionFactor); // * kSwerve.DriveSpeedScaling.minDriveSpeed; // Scaling to elevator height
    ySpeed = -(ySpeed * kSwerve.maxTransSpeed
     * driveSpeedConversionFactor); // * kSwerve.DriveSpeedScaling.minDriveSpeed; // Scaling to elevator height
    turningSpeed = -(turningSpeed * kSwerve.maxAngSpeed
     * driveSpeedConversionFactor); // * kSwerve.DriveSpeedScaling.minDriveSpeed; // Scaling to elevator height

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
    // m_SwerveSubsystem.stopModules(); // does nothing
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.controller;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CTRESwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

public class CTReSwerveControls {

  public CTReSwerveControls(CTRESwerveDrivetrain drivetrain, CommandXboxController controller) {
    double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    final Telemetry logger = new Telemetry(MaxSpeed);

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    controller.start().and(controller.b()).toggleOnTrue(drivetrain.applyRequest(() -> brake));
    controller.start().and(controller.rightTrigger()).toggleOnTrue(drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // reset the field-centric heading on left bumper press
    controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }
}

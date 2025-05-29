package frc.robot;

import Glitch.Lib.Swerve.Swerve;

public class SwerveSubsystem extends Swerve {
  public SwerveSubsystem() {
    super(
      9,
      8,
      3,
      2,
      5,
      4,
      7,
      6,
      22.52
    );
  }

  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here if needed
  }
}

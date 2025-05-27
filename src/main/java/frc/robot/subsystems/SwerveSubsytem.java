package frc.robot.subsystems;

import Glitch.Lib.Swerve.Swerve;

public class SwerveSubsytem extends Swerve {
  public SwerveSubsytem() {
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
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}

package frc.robot.subsystems.AlgaeIntake;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utilities.BaseSystems.Pivot;

public class TestPivot extends Pivot {
  private static final double maxVelocity = 100;
  private static final double maxAcceleration = 100;
  private static final double zeroedAngelFromHorizontal = 0;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
        .smartCurrentLimit(40)
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .inverted(false)
        .closedLoop
        .pid(0.5, 0, 0);
  }

  public TestPivot() {
    super(config, zeroedAngelFromHorizontal, maxVelocity, maxAcceleration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

  }
}

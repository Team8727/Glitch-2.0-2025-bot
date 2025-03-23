package frc.robot.subsystems.AlgaeIntake;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utilities.BaseSystems.Pivot;

import static frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;

public class TestPivot extends Pivot {
  private static final double maxVelocity = 100;
  private static final double maxAcceleration = 100;
  private static final double zeroedAngelFromHorizontal = 0;
  private static final double allowedError = 2;
  private static final int CANID = kAlgaeIntakePivot.intakePivotMotorCANID;
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
    super(config, CANID, zeroedAngelFromHorizontal, maxVelocity, maxAcceleration, allowedError);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here

  }
}
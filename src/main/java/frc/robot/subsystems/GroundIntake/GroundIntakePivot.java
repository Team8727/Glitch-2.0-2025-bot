package frc.robot.subsystems.GroundIntake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utilities.BaseSystems.Motors.SparkMaxMotor;
import frc.robot.utilities.BaseSystems.Pivot;

import static frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;

public class GroundIntakePivot extends Pivot {
  private static final double maxVelocity = 100000;
  private static final double maxAcceleration = 100000;
  private static final double zeroedAngelFromHorizontal = 100;
  private static final double allowedError = 1;
  private static final int CANID = kAlgaeIntakePivot.intakePivotMotorCANID;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(60)
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .closedLoop
      .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
      .pid(5, 0, 0);
  }

  public GroundIntakePivot() {
    super(
      new SparkMaxMotor(
        config,
        CANID,
        ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder),
      zeroedAngelFromHorizontal,
      maxVelocity,
      maxAcceleration,
      allowedError);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here

  }
}

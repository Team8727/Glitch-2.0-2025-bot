package frc.robot.subsystems.GroundIntake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utilities.BaseSystems.Motors.SparkMaxMotor;
import frc.robot.utilities.BaseSystems.Pivot;

import static frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;

public class GroundIntakePivot extends Pivot {
  private static final int CANID = kAlgaeIntakePivot.intakePivotMotorCANID;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(60)
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .closedLoop
      .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);}

  public GroundIntakePivot() {
    super(
      new SparkMaxMotor(
        config,
        CANID,
        ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder),
      100,
      1000,
      100,
      1);
    setPosition(60);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    if ((getPosition() * 360) > 300) {
      setDutyCycle(.05);
    }
    // Add any additional periodic logic here

  }
}

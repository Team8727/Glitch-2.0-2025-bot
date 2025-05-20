package frc.robot.subsystems.GroundIntake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import Glitch.Lib.Motors.SparkMaxMotor;
import Glitch.Lib.BaseMechanisms.Pivot;

public class GroundIntakePivot extends Pivot {
  public enum IntakePosition {
    HOME(20),
    SCORE(20),
    DOWN(90);

    private final double degrees;
    private IntakePosition(double degrees) { this.degrees = degrees; }

    public double getIntakePositionDegrees() {
      return degrees;
    }
  }

  private static final int CANID = 17;
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

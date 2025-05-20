package frc.robot.subsystems.GroundIntake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import Glitch.Lib.Motors.SparkMaxMotor;
import Glitch.Lib.BaseMechanisms.Roller;

public class GroundIntakeRollers extends Roller {

  private static final int CANID = 16;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(60)
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .inverted(false)
      .closedLoop
      .pid(.5, 0, 0);
  }

  public GroundIntakeRollers() {
    super(new SparkMaxMotor(config, CANID, ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder));
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here

  }
}

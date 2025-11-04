package frc.robot.subsystems.Elevator.Coral;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class BackCoralRoller extends Roller {
  private static final int CANID = 15;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(40)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(true)
      .closedLoop
      .pid(0.5, 0, 0);
  }

  public BackCoralRoller() {
    super(new SparkMaxMotor(config, CANID, ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder));
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here
  }
}

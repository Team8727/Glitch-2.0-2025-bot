package frc.robot.subsystems.GroundIntake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.utilities.BaseSystems.Motors.SparkMaxMotor;
import frc.robot.utilities.BaseSystems.Roller;

import static frc.robot.Constants.kAlgaeRemover.kRollers;

public class GroundIntakeRollers extends Roller {

  private static final int CANID = Constants.kAlgaeIntake.kAlgaeIntakeRollers.rollerMotorCANID;
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

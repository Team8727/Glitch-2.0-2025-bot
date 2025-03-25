package frc.robot.subsystems.Elevator.AlgaeRemover;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utilities.BaseSystems.Motors.SparkMaxMotor;
import frc.robot.utilities.BaseSystems.Roller;

import static frc.robot.Constants.kAlgaeRemover.kRollers;

public class RollerTest extends Roller {

  private static final int CANID = kRollers.removerRollerMotorCANID;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
        .smartCurrentLimit(25)
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .inverted(false)
        .closedLoop
        .pid(0, 0, 0);
  }

  public RollerTest() {
    super(new SparkMaxMotor(config, CANID, ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder));
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here

  }
}

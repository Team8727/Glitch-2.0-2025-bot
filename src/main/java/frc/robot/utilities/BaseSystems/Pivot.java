package frc.robot.utilities.BaseSystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator;

import java.util.Set;

public abstract class Pivot extends SubsystemBase {

  private final SparkMax motor;
  private final SparkClosedLoopController motorController;

  private final TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(0,0);

  private final ArmFeedforward pivotFeedforward;

  public final NetworkTableLogger logger;

  private final double dt;
  private final double zeroedAngelFromHorizontal;

  /**
   * Creates a new Pivot.
   *
   * @param config The configuration for the SparkMax motor
   * @param CANID The CAN ID of the motor
   * @param zeroedAngelFromHorizontal The angle from horizontal to zero the pivot at
   * @param maxVelocity The maximum velocity of the pivot
   * @param maxAcceleration The maximum acceleration of the pivot
   * @param ks The static gain of the pivot
   * @param kg The gravity gain of the pivot
   * @param kv The velocity gain of the pivot
   * @param ka The acceleration gain of the pivot
   * @param dt The time step for the profile
   */
  public Pivot(
      SparkMaxConfig config,
      int CANID,
      double zeroedAngelFromHorizontal,
      double maxVelocity,
      double maxAcceleration,
      double ks,
      double kg,
      double kv,
      double ka,
      double dt) {
    logger = new NetworkTableLogger(this.getName());

    m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    pivotFeedforward =  new ArmFeedforward(ks, kg, kv, ka ,dt);
    this.dt = dt;

    this.zeroedAngelFromHorizontal = zeroedAngelFromHorizontal;

    motor = getSparkMax(
      CANID,
      SparkLowLevel.MotorType.kBrushless,
      false,
      Set.of(),
      Set.of(
        SparkConfigurator.LogData.POSITION,
        SparkConfigurator.LogData.VELOCITY,
        SparkConfigurator.LogData.VOLTAGE,
        SparkConfigurator.LogData.CURRENT));

    motor.configure(
      config,
      SparkBase.ResetMode.kNoResetSafeParameters,
      SparkBase.PersistMode.kNoPersistParameters);

    motorController = motor.getClosedLoopController();
  }

  /**
   * Creates a new Pivot.
   *
   * @param config The configuration for the SparkMax motor
   * @param CANID The CAN ID of the motor
   * @param zeroedAngelFromHorizontal The angle from horizontal to zero the pivot at
   * @param maxVelocity The maximum velocity of the pivot
   * @param maxAcceleration The maximum acceleration of the pivot
   * @param ks The static gain of the pivot
   * @param kg The gravity gain of the pivot
   * @param kv The velocity gain of the pivot
   * @param ka The acceleration gain of the pivot
   */
  public Pivot(
      SparkMaxConfig config,
      int CANID,
      double zeroedAngelFromHorizontal,
      double maxVelocity,
      double maxAcceleration,
      double ks,
      double kg,
      double kv,
      double ka) {
    this(config, CANID, zeroedAngelFromHorizontal, maxVelocity, maxAcceleration, ks, kg, kv, ka, 0.02);
  }

  /**
   * Creates a new Pivot.
   *
   * @param config The configuration for the SparkMax motor
   * @param CANID The CAN ID of the motor
   * @param zeroedAngelFromHorizontal The angle from horizontal to zero the pivot at
   * @param maxVelocity The maximum velocity of the pivot
   * @param maxAcceleration The maximum acceleration of the pivot
   */
  public Pivot(
      SparkMaxConfig config,
      int CANID,
      double zeroedAngelFromHorizontal,
      double maxVelocity,
      double maxAcceleration) {
    this(config, CANID, zeroedAngelFromHorizontal, maxVelocity, maxAcceleration, 0, 0, 0, 0);
  }

  /**
   * Set the pivot position using a trapezoidal profile
   *
   * @param angle The angle to set the pivot to
   */
  public void setPosition(double angle) {
    double position = angle / 360;
    m_goal = new TrapezoidProfile.State(position, 0);
  }

  // set pivot position
  private void setMotorFFAndPIDPosition(double Position) {
    motorController.setReference(
      Position,
      SparkBase.ControlType.kPosition,
      ClosedLoopSlot.kSlot0,
      pivotFeedforward.calculateWithVelocities(
        motor.getAbsoluteEncoder().getPosition() + (zeroedAngelFromHorizontal/360),
        motor.getAbsoluteEncoder().getVelocity(),
        m_setpoint.velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logger.logDouble("Pivot Position", motor.getAbsoluteEncoder().getPosition() * 360);

    m_setpoint = m_profile.calculate(dt, m_setpoint, m_goal);

    setMotorFFAndPIDPosition(m_setpoint.position);
  }
}

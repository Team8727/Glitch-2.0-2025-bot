// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakeRollers;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

public class AlgaeIntakeRollers extends SubsystemBase {
  private final SparkMax intakeRollerMotor;
  private final DigitalInput algaeCheck;
  private final SparkClosedLoopController rollerPID;
  public boolean isMoving = false;
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getSubsystem());


  /** Creates a new AlgaeIntakeRollers. */
  public AlgaeIntakeRollers() {

// -=-=-=-=-=- rollerMotor Initialization -=-=-=-=-=-|Contructor|

    intakeRollerMotor =
        getSparkMax(
            kAlgaeIntakeRollers.rollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));

// -=-=-=-=-=- pivotMotor config, PID config, and maxMotion constraints config -=-=-=-|Contructor|

    // Setting the output range, PID, and maxMotion constraints for the motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
      // Motor Config
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      // PID Control
      .closedLoop
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Configuring Motor With Config
    intakeRollerMotor.configure(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);

// -=-=-=-=- Instantiate Intake Sensor -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Contructor|

    algaeCheck = new DigitalInput(kAlgaeIntakeRollers.sensorChannel);

// -=-=-=-=- PID controller for the motor for convenience -=-=-=-|Contructor|

    rollerPID = intakeRollerMotor.getClosedLoopController();
  }

// -=-=-=-=-=-=- Subsystem Methods -=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Subsystem|

  /**
   * Sets the speed of the rollers, implementing built-in PID
   * @param speed : the speed to provide to the motor's built-in PID controller
   * @apiNote This also includes feedforward because we passed in a velocity FF 
   * gain into the built-in controller, and it will automatically use the FF.
   */
  public void setRollerSpeedDuty(double speed) {
    rollerPID.setReference(speed, ControlType.kDutyCycle); 
  }

  public void stopRollers() {
    setRollerSpeedDuty(0);
  }

  public boolean getAlgaeCheck() {
    return !algaeCheck.get();
  }

// -=-=-=-=-=-=-=-= Logging =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Subsystem|
  /**
   * Used in subsystem periodic to log and update values
   */
  public void logValues() {
    logger.logDouble("Motor Current", intakeRollerMotor.getOutputCurrent());
    logger.logBoolean("Intake Sensor (boolean)", algaeCheck.get());
    // Add other relevant values
  }
// -=-=-=-=-=-=-=-=- Commands -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Subsystem|

  private void holdAlgae() {
    if (!isMoving) {
      if (getAlgaeCheck()) {
        setRollerSpeedDuty(.5);
      } else {
      stopRollers();
      }
    }
  }

// -=-=-=-=-=-=- Periodic Override -=-=-=-=-=-=-=-=-=-=-=-|Subsystem|
  
  @Override
  public void periodic() {
    holdAlgae();
    logger.logBoolean("Algae Sensor", getAlgaeCheck());
    // This method will be called once per scheduler run
    logValues();
  }
}


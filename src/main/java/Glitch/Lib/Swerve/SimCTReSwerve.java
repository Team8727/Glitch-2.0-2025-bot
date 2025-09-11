package Glitch.Lib.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class SimCTReSwerve {
  private final TalonFX frontRightDrive = new TalonFX(20);
  private final TalonFX frontLeftDrive = new TalonFX(21);
  private final TalonFX backRightDrive = new TalonFX(22);
  private final TalonFX backLeftDrive = new TalonFX(23);
  private final TalonFX frontRightSteer = new TalonFX(24);
  private final TalonFX frontLeftSteer = new TalonFX(25);
  private final TalonFX backRightSteer = new TalonFX(26);
  private final TalonFX backLeftSteer = new TalonFX(27);

  private final Pigeon2 pigeon = new Pigeon2(28);

  private final CANcoder CANcoderFrontRight = new CANcoder(29);
  private final CANcoder CANcoderFrontLeft = new CANcoder(30);
  private final CANcoder CANcoderBackRight = new CANcoder(31);
  private final CANcoder CANcoderBackLeft = new CANcoder(32);

  private final TalonFXSimState frontRightDriveSim = new TalonFXSimState(frontRightDrive);
  private final TalonFXSimState frontLeftDriveSim = new TalonFXSimState(frontLeftDrive);
  private final TalonFXSimState backRightDriveSim = new TalonFXSimState(backRightDrive);
  private final TalonFXSimState backLeftDriveSim = new TalonFXSimState(backLeftDrive);
  private final TalonFXSimState frontRightSteerSim = new TalonFXSimState(frontRightSteer);
  private final TalonFXSimState frontLeftSteerSim = new TalonFXSimState(frontLeftSteer);
  private final TalonFXSimState backRightSteerSim = new TalonFXSimState(backRightSteer);
  private final TalonFXSimState backLeftSteerSim = new TalonFXSimState(backLeftSteer);

  private final Pigeon2SimState pigeonSim = new Pigeon2SimState(pigeon);

  private final CANcoderSimState CANcoderFrontRightSim = new CANcoderSimState(CANcoderFrontRight);
  private final CANcoderSimState CANcoderFrontLeftSim = new CANcoderSimState(CANcoderFrontLeft);
  private final CANcoderSimState CANcoderBackRightSim = new CANcoderSimState(CANcoderBackRight);
  private final CANcoderSimState CANcoderBackLeftSim = new CANcoderSimState(CANcoderBackLeft);


  public SimCTReSwerve() {
  }

  public void fakeSwerve() {
    fakeTalon(frontRightDriveSim);
    fakeTalon(frontLeftDriveSim);
    fakeTalon(backRightDriveSim);
    fakeTalon(backLeftDriveSim);
    fakeTalon(frontRightSteerSim);
    fakeTalon(frontLeftSteerSim);
    fakeTalon(backRightSteerSim);
    fakeTalon(backLeftSteerSim);

    pigeonSim.setSupplyVoltage(12.0);
    pigeonSim.setRawYaw(45.0);     // degrees
    pigeonSim.setPitch(5.0);    // degrees
    pigeonSim.setRoll(-2.0);    // degrees

    fakeCANcoder(CANcoderFrontRightSim);
    fakeCANcoder(CANcoderFrontLeftSim);
    fakeCANcoder(CANcoderBackRightSim);
    fakeCANcoder(CANcoderBackLeftSim);
  }

  private void fakeTalon(TalonFXSimState motor) {
    motor.setRawRotorPosition(5.0); // rotations
    motor.setSupplyVoltage(12.0);   // volts
    motor.setRotorVelocity(12.0);
  }

  private void fakeCANcoder(CANcoderSimState encoder) {
    encoder.setSupplyVoltage(12.0);
    encoder.setRawPosition(0.0); // degrees
  }
}
package frc.Glitch.Lib.BaseMechanisms;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.Motor;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.TestModules;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class RollerTest {
  private static final class TestRoller extends Roller {
    TestRoller(Motor motor) {
      super(motor);
    }
  }

  private TestModules.TestMotor motor;
  private Roller roller;

  @BeforeEach
  void setUp() {
    motor = new TestModules.TestMotor();
    roller = new TestRoller(motor);
  }

  @Test
  void setSpeedDutyCycleDelegatesToMotor() {
    roller.setSpeedDutyCycle(0.75);
    assertEquals(0.75, motor.duty, 1e-9);
  }

  @Test
  void setSpeedVelocityDelegatesToMotor() {
    roller.setSpeedVelocity(123.0);
    assertEquals(123.0, motor.velocity, 1e-9);
  }

  @Test
  void setPositionDelegatesAndUsesZeroFeedforward() {
    roller.setPosition(2.5);
    assertEquals(2.5, motor.position, 1e-9);
    assertEquals(0.0, motor.lastFeedforward, 1e-9);
  }

  @Test
  void gettersReflectMotorState() {
    motor.position = 5.25;
    motor.velocity = 60.0;
    motor.current = 2.5;
    motor.setForwardLimit(true);
    motor.setReverseLimit(false);

    assertEquals(5.25, roller.getPosition(), 1e-9);
    assertEquals(60.0, roller.getVelocity(), 1e-9);
    assertEquals(2.5, roller.getCurrent(), 1e-9);
    assertTrue(roller.getForwardLimitSwitch());
    assertFalse(roller.getReverseLimitSwitch());
  }

  @Test
  void periodicLogsTelemetry() {
    motor.position = 1.25;
    motor.velocity = 123.5;
    motor.current = 4.75;
    motor.setForwardLimit(true);
    motor.setReverseLimit(true);

    roller.periodic();

    var table = NetworkTableInstance.getDefault().getTable("TestRoller");
    assertEquals(1.25, table.getEntry("position").getDouble(Double.NaN), 1e-9);
    assertEquals(123.5, table.getEntry("velocity").getDouble(Double.NaN), 1e-9);
    assertEquals(4.75, table.getEntry("current").getDouble(Double.NaN), 1e-9);
    assertTrue(table.getEntry("forward limit switch").getBoolean(false));
    assertTrue(table.getEntry("reverse limit switch").getBoolean(false));
  }
}


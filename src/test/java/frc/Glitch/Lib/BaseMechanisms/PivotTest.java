package frc.Glitch.Lib.BaseMechanisms;

import Glitch.Lib.BaseMechanisms.Pivot;
import Glitch.Lib.Motors.Motor;
import frc.TestModules;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PivotTest {
    private static class TestPivot extends Pivot {
        public TestPivot(Motor motor) {
            super(
                motor,
                0.0,   // zeroed angle from horizontal (deg)
                180.0, // max velocity (deg/s)
                360.0, // max acceleration (deg/s^2)
                1.0    // allowed error (deg)
            );
        }
    }

    private TestModules.TestMotor motor;
    private TestPivot pivot;

    @BeforeEach
    void setUp() {
        motor = new TestModules.TestMotor();
        pivot = new TestPivot(motor);
    }



    @Test
    void forwardsDutyCycleToMotor() {
        pivot.setDutyCycle(0.25);
        assertEquals(0.25, motor.duty, 1e-9);
        pivot.setDutyCycle(-0.5);
        assertEquals(-0.5, motor.duty, 1e-9);
    }

    @Test
    void reachesSetpointOverTime() {
        // Start at 0 deg
        motor.position = 0.0;
        // Command 90 degrees
        pivot.setPosition(90.0);

        // Run "periodic" enough times to finish the trapezoid profile
        for (int i = 0; i < 200; i++) {
            pivot.periodic();
        }

        assertTrue(pivot.isAtSetpoint(), "Pivot should be within tolerance of setpoint");
        assertEquals(90.0, motor.getPosition() * 360.0, 1.0);
    }

    @Test
    void alreadyAtGoalIsInstantlyAtSetpoint() {
        // Start at 180 deg
        motor.position = 0.5; // 180 deg

        pivot.setPosition(180.0);
        // Without calling periodic, setpoint should initialize to current position
        assertTrue(pivot.isAtSetpoint());
        assertEquals(180.0, motor.getPosition() * 360.0, 1e-9);
    }

    @Test
    void isAtSetpointUsesAbsoluteError() {
        // Ensure setpoint is 0 deg
        motor.position = 0.0;
        pivot.setPosition(0.0);
        assertTrue(pivot.isAtSetpoint());

        // Simulate overshoot by +2 deg (outside allowed error of 1 deg)
        motor.position = 2.0 / 360.0;
        assertFalse(pivot.isAtSetpoint(), "Should be false when motor is +2 deg from setpoint");

        // Simulate within tolerance +0.5 deg
        motor.position = 0.5 / 360.0;
        assertTrue(pivot.isAtSetpoint(), "Should be true when within +0.5 deg tolerance");

        // Simulate overshoot by -2 deg (outside allowed error)
        motor.position = -2.0 / 360.0;
        assertFalse(pivot.isAtSetpoint(), "Should be false when motor is -2 deg from setpoint");

        // Simulate within tolerance -0.5 deg
        motor.position = -0.5 / 360.0;
        assertTrue(pivot.isAtSetpoint(), "Should be true when within -0.5 deg tolerance");
    }

    @Test
    void getPositionReflectsMotor() {
        motor.position = 1.25;
        assertEquals(1.25, pivot.getPosition(), 1e-9);
    }
 }

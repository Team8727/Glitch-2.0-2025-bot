// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class kConfigs {

    public static RobotConfig robotConfig; 
    static {
      try {
        robotConfig = RobotConfig.fromGUISettings();
      } catch (IOException | ParseException e) {
        e.printStackTrace();
        System.out.println("Failed to load robot config from GUI settings, using default values");
        // Default configuration if fromGUISettings fails or is not valid
        robotConfig = new RobotConfig(
            65, // TODO: find real value
            5.22, // TODO: find real value
            new ModuleConfig(
                .0371,
                4.55,
                1.4, // TODO: find real value
                new DCMotor(
                    12, 
                    3.6, 
                    211, 
                    3.6, 
                    710.42, // TODO: probably wrong
                    1),
                5.08,
                50,
                1),
            kSwerve.kinematics.getModules());
      }
    }

    public static final DCMotor neoMotor = 
      new DCMotor(
        12,
        2.6,
        105, 
        1.8, 
        594.4, 
        1);
  }

  public static int configurationSetRetries = 5;

  public static class kOI {
    public static double translationDeadzone = 0.08;
    public static double rotationDeadzone = 0.08;
  }

  // Swerve subsystem constants (module constants included)
  public static class kSwerve {
    // Chassis dimensions from wheel center to center (meters)
    public static double width = Units.inchesToMeters(22.52);
    public static double length = width;

    // Speed & accel limits (m/s, rad/s, m/s^2 & rad/s^2)
    // public static double maxTransSpeed = 5;//5
    // public static double maxAngSpeed = 3 * Math.PI;//3

    // public static double maxTransAccel = 1.35 * 9.81;//1.35
    // public static double maxAngAccel = 10 * 2 * Math.PI;//10

    // test speeds
    public static final double maxTransSpeed = 5; // 5
    public static final double maxAngSpeed = 3 * Math.PI; // 3

    public static final double maxTransAccel = 1.35 * 9.81; // 1.35
    public static final  double maxAngAccel = 10 * 2 * Math.PI; // 10

    public static class DriveSpeedScaling {
      public static final double minDriveSpeed = 0.5; // TODO: Set this for scaling
    }

    // Operator interface constants
    public static class Teleop {
      public static double translationGain = 1;
      public static double rotationGain = 0.7;

      public static boolean closedLoop = false;
    }

    // NavX
    public static boolean invertGyro = false;
    public static Port navxPort = Port.kMXP;

    // Swerve uses ccw+ angular quanities and a coordinate plane with 0,0 at the robot's center
    // , forward is +x, and a module order based on the quadrant system (front left is first)
    public static SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(length / 2, width / 2), // front left
            new Translation2d(length / 2, -width / 2), // front right
            new Translation2d(-length / 2, width / 2), // back left
            new Translation2d(-length / 2, -width / 2)); // back right

    public static SwerveDriveKinematics autoKinematics =
      new SwerveDriveKinematics(
          new Translation2d(length / 2, width / 2), // front left
          new Translation2d(length / 2, -width / 2), // front right
          new Translation2d(-length / 2, width / 2), // back left
          new Translation2d(-length / 2, -width / 2)); // back right

    // Module angular offsets (rad)
    public static class Offsets {
      public static double frontLeft = Math.PI / 2;
      public static double backLeft = -Math.PI;
      public static double backRight = -Math.PI / 2;
      public static double frontRight = 0;
    }

    // Controller PID values for x/y translation, and z rotation
    public static class Auton {
      public static final double angP = 4;
      public static final double angD = 0;

      public static final double maxAccel = 0.4;
      public static final double maxVel = 0.4;
      public static final double maxAngAccel = 0.4 * kSwerve.maxAngAccel;
      public static final double maxAngVel = 0.4 * kSwerve.maxAngSpeed;

      public static final double transP = 8;

      public static final double maxOnTheFlyVel = 2;
      public static final double maxOnTheFlyAcc = 2;

      public static final HolonomicDriveController controller =
          new HolonomicDriveController(
              new PIDController(Auton.transP, 0.0, 0.0),
              new PIDController(angP, 0.0, angD),
              new ProfiledPIDController(
                  0,
                  0,
                  0, // TODO: tune this also figure out what it is
                  new Constraints(maxVel, maxAccel)));

      public static final PathFollowingController pathFollowController =
          new PPHolonomicDriveController(
              new PIDConstants(Auton.transP, 0, 0), new PIDConstants(angP, 0, angD));

      public static final PathConstraints constraints =
          new PathConstraints(
              Auton.maxOnTheFlyVel, Auton.maxOnTheFlyAcc, Auton.maxAngVel, Auton.maxAngAccel);
      // public static final HolonomicPathFollowerConfig pathFollowConfig =
      //     new HolonomicPathFollowerConfig(
      //         new PIDConstants(Auton.transP, 0.0, 0), // Translation PID constants
      //         new PIDConstants(angP, 0.0, angD), // Rotation PID constants
      //         kModule.maxWheelSpeed, // Max module speed, in m/s
      //         Math.pow(
      //             Math.pow(kSwerve.width / 2, 2) + Math.pow(kSwerve.length / 2, 2),
      //             1 / 2), // Drive base radius in meters. Distance from robot center to furthest
      //         // module.
      //         new ReplanningConfig(false, false));
    }

    public static class kModule {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
      public static final int drivePinionTeeth = 14;
      public static final boolean invertSteerEncoder = true;

      // Controls Constants
      public static class kDrive {
        public static final double kP = 0.25;
        public static final double kD = 0.05;
        public static final double kS = 0.068841;
        public static final double kV = 2.4568;
        public static final double kA = 0.22524;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      public static class kSteer {
        public static final double kP = 2.5;
        public static final double kD = 0;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      // Motor configs
      public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode steeringMotorIdleMode = IdleMode.kBrake;

      public static final int driveSmartCurrentLimit = 50; // amps
      public static final int driveMaxCurrent = 80; // amps
      public static final int steerSmartCurrentLimit = 20; // amps
      public static final int steerMaxCurrent = 35; // amps

      // Physical dimensions/values
      public static final double wheelDiameter = 0.97 * Units.inchesToMeters(3);
      public static final double wheelCircumference = wheelDiameter * Math.PI; // meters
      public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
      public static final double steerMotorReduction = 9424.0 / 203.0;

      // Motor physics
      public static final double neoFreeSpeed = 5820.0 / 60; // rot/s
      public static final double neoFreeCurrent = 1.7; // amps
      public static final double neoStallTorque = 3.28; // Nm
      public static final double neoStallCurrent = 181; // amps

      public static final double neo550FreeSpeed = 11710.0 / 60;
      public static final double neo550FreeCurrent = 1.1;
      public static final double neo550StallTorque = 1.08;
      public static final double neo550StallCurrent = 111;

      public static final double maxWheelSpeed =
          (neoFreeSpeed / driveMotorReduction) * (wheelDiameter * Math.PI); // m/s

      // Encoders
      public static final double drivingEncoderPositionFactor =
          (wheelDiameter * Math.PI) / driveMotorReduction; // meters
      public static final double drivingEncoderVelocityFactor =
          ((wheelDiameter * Math.PI) / driveMotorReduction) / 60.0; // m/s

      public static final double steeringEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double steeringEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

      public static final double steeringEncoderPositionPIDMinInput = 0; // radians
      public static final double steeringEncoderPositionPIDMaxInput =
          steeringEncoderPositionFactor; // radians
    }

    // Motor CAN IDs
    public static class CANID {
      public static int frontLeftDrive = 9; //
      public static int frontLeftSteer = 8; //
      public static int backLeftDrive = 3; //
      public static int backLeftSteer = 2; //
      public static int backRightDrive = 5; //
      public static int backRightSteer = 4; //
      public static int frontRightDrive = 7; //
      public static int frontRightSteer = 6; //
    }
  }

  public static class kVision {
    // Vision

    public static class kPoses {
      public static final Pose2d blueFrontLeft =
          new Pose2d(6.05, 3.81, Rotation2d.fromDegrees(180)); // x,y in meters
    }

    public static final AprilTagFieldLayout aprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Translation2d fieldCenter = new Translation2d(8.770, 4.026); // meters
    // x   y     z
    // 8   7     22
    // -8  7     22
    // -8  6.625 19.5
    //  8  6.625 19.5
    public static final Transform3d camera1Position = // Right Rear
      new Transform3d(
        new Translation3d(Units.inchesToMeters(7), Units.inchesToMeters(8), Units.inchesToMeters(22)), 
        new Rotation3d(0, Math.toRadians(20), Math.toRadians(20)));
    public static final Transform3d camera2Position = // Upper
      new Transform3d(
        new Translation3d(Units.inchesToMeters(7), Units.inchesToMeters(-8), Units.inchesToMeters(22)),
        new Rotation3d(0, Math.toRadians(20), Math.toRadians(-20)));
    public static final Transform3d camera3Position = // Left Rear
      new Transform3d(
        new Translation3d(Units.inchesToMeters(6.625), Units.inchesToMeters(-8), Units.inchesToMeters(19.5)),
        new Rotation3d(0, Math.toRadians(25), Math.toRadians(30)));
    public static final Transform3d camera4Position = // Front
      new Transform3d(
        new Translation3d(Units.inchesToMeters(6.625), Units.inchesToMeters(8), Units.inchesToMeters(19.5)),
        new Rotation3d(0, Math.toRadians(25), Math.toRadians(-30)));

    public static final Matrix<N3, N1> stateStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.02, 0.02, 0.01);
    public static final Matrix<N3, N1> visionStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.03, 0.03, 0.25);
    public static final double visionScalingFactor =
        2.3; // scaling factor applied to the visionStdDevs per meter bigger means trust less at distance
        
  }
  public static class kAlgaeRemover {
    public static class kPivot {
      public static int removerPivotMotorCANID =12; // TODO: not set yet because remover is not built yet

      public enum RemoverPositions {
        RaisedL2(80), // TODO: SET WITH ACTUAL VALUES
        RaisedL3(50), // TODO: SET WITH ACTUAL VALUES
        Stowed(18); // TODO: SET WITH ACTUAL VALUES
      
        private final double degrees;
        
        private RemoverPositions(double degrees) {
          this.degrees = degrees;
        }
  
        public double getDegrees() {
          return degrees;
        }
      }
}

    public static class kRollers {
      public static int removerRollerMotorCANID =
          13; // TODO: not set yet because remover is not built yet
    }
  }

  public static class kAlgaeIntake {
    public static class kAlgaeIntakePivot {
      public static int intakePivotMotorCANID =
          17; // TODO: not set yet because intake is not built yet

      public enum IntakePosition {
          HOME(20), 
          SCORE(20), 
          DOWN(90); // TODO: SET WITH ACTUAL VALUES

          private final double degrees;
          private IntakePosition(double degrees) { this.degrees = degrees; }

          public double getIntakePositionDegrees() { 
            return degrees; 
          }
      }

      public TrapezoidProfile pivotMotionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(10, 20));

      public static int intakePivotEncoderChannelA =
          10; // TODO: not set yet because intake is not built yet
      public static int intakePivotEncoderChannelB =
          11; // TODO: not set yet because intake is not built yet
      public static double encoderPulsesPerRevolution = 2048; // Maybe?

      public static double gearRatio = 4; // TODO: This is probably what it is, but either set using document in Glitch Drive, or ask CAD.

      public static double idleAlgaeIntakeVoltage = 0; // TODO: set as needed when testing

      public static final int motorCurrentLimit = 35;
    }

    public static class kAlgaeIntakeRollers {
      public static int rollerMotorCANID = 16; // TODO: not set yet because intake is not built yet

      public static int sensorChannel = 0; // TODO: not set yet because intake is not built yet

      public static int intakeSpeed = 11; // TODO: not set yet because intake is not built yet
      public static int scoreVoltage = -11; // TODO: not final yet because intake is not built yet
      public static int outtakeSpeed = -5; // TODO: not set yet because intake is not built yet
    }
  }

  public static class kCoral {
    public static int intakeRollerMotorCANID =
        15; // TODO: not set yet because intake is not built yet
    public static int outtakeRollerMotorCANID =
        14; // TODO: not set yet because intake is not built yet

    public static double intakeSpeed = .2; // TODO: not set yet because intake is not built yet
    public static double outtakeSpeed = .75;// TODO: not set yet because intake is not built yet

    public static double coraldeploySpeedL1 = 0.5; // TODO: not set yet because intake is not built yet
    public static double coraldeploySpeedL2 = 0.5; // TODO: not set yet because intake is not built yet
    public static double coraldeploySpeedL3 = 0.5; // TODO: not set yet because intake is not built yet
    public static double coraldeploySpeedL4 = 0.5; // TODO: not set yet because intake is not built yet
  }

  public static class kElevator {
    // elevator calculations https://www.desmos.com/calculator/suqtj7vxc7
    public static int elevatorMotorRCANID = 10; // TODO: not set yet because elevator is not built yet
    public static int elevatorMotorLCANID = 11; // TODO: not set yet because elevator is not built yet

    public static int limitSwitchDIO = 3; // TODO: not set yet because elevator is not built yet
    public static double gearRatio = 5;

    public enum ElevatorPosition {
      HOME(0), // TODO: SET WITH ACTUAL VALUES
      L1(1.1), // TODO: SET WITH ACTUAL VALUES
      L2(8.23663),// TODO: SET WITH ACTUAL VALUES
      L3(20.43877), // TODO: SET WITH ACTUAL VALUES
      L4(39.53888), // TODO: SET WITH ACTUAL VALUES
      A2(13.5), // TODO: SET WITH ACTUAL VALUES
      A3(25.5); // TODO: SET WITH ACTUAL VALUES
    
      private final double rotations;
      
      private ElevatorPosition(double rotations) {
        this.rotations = rotations;
      }

      public double getOutputRotations() {
        return rotations;
      }
    }
  }
  public static class CustomCommands {
    public static Command waitCommand(double seconds, Runnable command) {
      return Commands.runOnce(() -> {
        new Thread(() -> {
          try {
            Thread.sleep((long) (seconds * 1000));
            new InstantCommand(command).schedule();
          } catch (InterruptedException e) {
            e.printStackTrace();
          } finally {
            Thread.currentThread().interrupt();
          }
        }).start();
      });
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeightCmd extends Command {
  private final Elevator m_elevator;
  private final ElevatorPosition m_scoreLevel;
  private final LEDSubsystem m_ledSubsystem;
  private final LEDPatterns m_ledPatterns;
  private final Coral m_coral;

  /** Creates a new SetEvevatorHeightCmd. */
  public SetElevatorHeightCmd(ElevatorPosition scoreLevel, Elevator elevator, Coral coral, LEDSubsystem ledSubsystem, LEDPatterns ledPatterns) {

    m_scoreLevel = scoreLevel;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    m_ledPatterns = ledPatterns;
    m_coral = coral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral, ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_coral.getBackCoralSensor()) {
      System.out.println("Setting elevator height to " + m_scoreLevel);
      m_elevator.setElevatorHeightMotionProfile(m_scoreLevel);
    } else {
      System.out.println("hey driver, are you trying to kill the elevator or something? please move the coral out of the way");
    }

    if (m_scoreLevel != ElevatorPosition.L1) {
      m_ledSubsystem.setPatternForDuration(m_ledPatterns.elevatorProgress, 0.5);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  @Override 
  public void end(boolean interrupted) {
    System.out.println("SetElevatorHeightCmd ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return m_elevator.m_setpoint.position == m_scoreLevel.getOutputRotations();
  }
}

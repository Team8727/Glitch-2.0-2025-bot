// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeightCmd extends Command {
  private final Elevator m_elevator;
  private final ElevatorPosition m_scoreLevel;
  private final LEDSubsystem m_ledSubsystem;
  private final Coral m_coral;
  private boolean endCmd = false; // flag to indicate when the command should end

  /** Creates a new SetEvevatorHeightCmd. */
  public SetElevatorHeightCmd(ElevatorPosition scoreLevel, Elevator elevator, Coral coral, LEDSubsystem ledSubsystem) {

    m_scoreLevel = scoreLevel;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
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
      m_ledSubsystem.setPattern(m_ledSubsystem.elevatorProgress);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_elevator.m_setpoint.position == m_scoreLevel.getOutputRotations()) {
      endCmd = true; // set the endCmd to true when the elevator reaches the desired height
    }
  }

  @Override 
  public void end(boolean interrupted) {
    System.out.println("SetElevatorHeightCmd ended");
    endCmd = false; // reset the endCmd flag when the command ends

    m_ledSubsystem.resetToDefaultPattern();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return endCmd;
  }
}

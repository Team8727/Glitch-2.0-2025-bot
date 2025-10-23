// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Coral.BackCoralRoller;
import frc.robot.subsystems.Elevator.Coral.FrontCoralRoller;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCmd extends Command {
  private final BackCoralRoller backCoralRoller;
  private final FrontCoralRoller frontCoralRoller;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;
  boolean sensedCoral = false;
  boolean end = false;

  /**
   * Creates a new IntakeCoral.
   */
  public IntakeCoralCmd(BackCoralRoller backCoralRoller, FrontCoralRoller frontCoralRoller, Elevator elevator, LEDSubsystem ledSubsystem) {
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;

    addRequirements(frontCoralRoller, backCoralRoller, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getElevatorSetPosition() != Elevator.ElevatorPosition.L1) {
      System.out.println("go to L1");
      this.end = true;
    } else {
      backCoralRoller.setSpeedDutyCycle(.4);
      m_ledSubsystem.setPattern(LEDPatterns.blinkyGreen);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (frontCoralRoller.getReverseLimitSwitch() && !sensedCoral) {
      backCoralRoller.setSpeedDutyCycle(.13);
      frontCoralRoller.setSpeedDutyCycle(.12);
      sensedCoral = true;
    }

    if (!frontCoralRoller.getReverseLimitSwitch() && frontCoralRoller.getForwardLimitSwitch() && sensedCoral) {
      backCoralRoller.setSpeedDutyCycle(0);
      frontCoralRoller.setSpeedDutyCycle(0);
      this.end = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sensedCoral = false;
    end = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
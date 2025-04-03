// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCmd extends Command {
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;
  boolean sensedCoral = false;
  boolean end = false;

  /** Creates a new IntakeCoral. */
  public IntakeCoralCmd(Coral coral, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;

    addRequirements(coral, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getElevatorSetPosition() != kElevator.ElevatorPosition.L1) {
      System.out.println("go to L1");
      this.cancel();
    } else {
      m_coral.setIntakeSpeedDuty(.4);
      // m_ledSubsystem.setPatternForDuration(m_ledSubsystem.green, 2);
      // Timer.delay(2);
      // m_ledSubsystem.setPatternForDuration(m_ledSubsystem.coralPickup, 2);
      // new Thread(() -> {
      //   // m_ledSubsystem.activateSecretPattern(!m_coral.getBackCoralSensor());
      //   // Thread.currentThread().interrupt();
      // }).start();
      m_ledSubsystem.setPattern(LEDPatterns.blinkyGreen);;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_coral.getBackCoralSensor() && !sensedCoral) {
      m_coral.setIntakeSpeedDuty(.13);
      m_coral.setOuttakeSpeedDuty(.12);
      sensedCoral = true;
    } 

    if (!m_coral.getBackCoralSensor() && sensedCoral) {
      m_coral.setIntakeSpeedDuty(0);
      m_coral.holdPosition();
      sensedCoral = false;
      end = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.setIntakeSpeedDuty(0);
    m_coral.setOuttakeSpeedDuty(0);
    end = false;
    m_ledSubsystem.setPatternForDuration(LEDSubsystem.defaultPattern, 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}

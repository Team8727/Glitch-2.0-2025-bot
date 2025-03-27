// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReindexCoralCmd extends Command {
  private final Coral m_coral;
  private final Elevator m_Elevator;
  private final LEDSubsystem m_LedSubsystem;

  /** Creates a new ReindexCoralCmd. */
  public ReindexCoralCmd(Coral coral, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_coral = coral;
    m_Elevator = elevator;
    m_LedSubsystem = ledSubsystem;

    addRequirements(coral, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Thread(() -> {
      for (int n = 1; n <= 3; n++){
        m_coral.setIntakeSpeedDuty(.15);
        try {
          Thread.sleep(200);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          e.printStackTrace();
        }
        m_coral.setIntakeSpeedDuty(-.15);
        try {
          Thread.sleep(200);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          e.printStackTrace();
        }
      }
      new IntakeCoralCmd(m_coral, m_Elevator, m_LedSubsystem).schedule();
      this.cancel();
      Thread.currentThread().interrupt();
    }).start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

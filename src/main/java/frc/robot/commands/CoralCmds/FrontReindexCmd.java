// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Coral.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FrontReindexCmd extends Command {
  private final Coral m_Coral;
  private boolean pastBackSensor;
  /** Creates a new FrontReingdx. */
  public FrontReindexCmd(Coral coral) {
    m_Coral = coral;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Coral.setIntakeSpeedDuty(.2);
    m_Coral.setOuttakeSpeedDuty(.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Coral.getBackCoralSensor()) {
      pastBackSensor = true;
      m_Coral.setIntakeSpeedDuty(-.2);
      m_Coral.setOuttakeSpeedDuty(-.2);
    }
    if (m_Coral.getBackCoralSensor() && pastBackSensor) {
      new Thread(() -> {
        m_Coral.setIntakeSpeedDuty(.1);
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          e.printStackTrace();
        }
        m_Coral.setIntakeSpeedDuty(0);
        m_Coral.holdPosition();
        this.cancel();
        Thread.currentThread().interrupt();
      }).start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Coral.BackCoralRoller;
import frc.robot.subsystems.Elevator.Coral.FrontCoralRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RejectCoralCmd extends Command {
  private final FrontCoralRoller frontCoralRoller;
  private final BackCoralRoller backCoralRoller;

  /** Creates a new ReindexCoralCmd. */
  public RejectCoralCmd(BackCoralRoller backCoralRoller, FrontCoralRoller frontCoralRoller) {
    this.frontCoralRoller = frontCoralRoller;
    this.backCoralRoller = backCoralRoller;

    addRequirements(frontCoralRoller, backCoralRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Thread(() -> {
      backCoralRoller.setSpeedDutyCycle(-1);
      frontCoralRoller.setSpeedDutyCycle(1);
      try {
        Thread.sleep(500);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        e.printStackTrace();
      }
      backCoralRoller.setSpeedDutyCycle(0);
      frontCoralRoller.setSpeedDutyCycle(0);
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

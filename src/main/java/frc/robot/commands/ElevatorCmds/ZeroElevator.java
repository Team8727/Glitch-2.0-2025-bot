// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Elevator.Elevator;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroElevator extends Command {
  private final Elevator m_elevator;


  /** Creates a new SetEvevatorHeightCmd. */
  public ZeroElevator(Elevator elevator) {

    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    // this.beforeStarting(() -> {
    //   m_coral.elevatorUp = true;
    //   new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsystem);
    // }, m_coral);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SequentialCommandGroup(
      new InstantCommand(() -> m_elevator.isHoming = true),
      new WaitCommand(.1),
      new InstantCommand(() -> m_elevator.setDutyCycle(-0.1)),
      new WaitCommand(0.3),
      new WaitUntilCommand(() -> m_elevator.getCurrentDrawAmps() > 35),
      new InstantCommand(() -> m_elevator.resetElevatorEncoders()),
      new InstantCommand(() -> m_elevator.isHoming = false),
      new RunCommand(() -> this.cancel())).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return false;
  }
}

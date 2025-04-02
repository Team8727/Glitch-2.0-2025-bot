// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorAlgaeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAlgaeRemover.kPivot.RemoverPositions;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgaeCmd extends Command {
  private final AlgaeRemoverPivot m_pivot;
  private final AlgaeRemoverRollers m_rollers;
  private final ElevatorPosition m_setPos;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;

  /** Creates a new removeAlgae. */
  public RemoveAlgaeCmd(AlgaeRemoverPivot algaeRemoverPivot, AlgaeRemoverRollers algaeRemoverRollers, ElevatorPosition setPos, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_pivot = algaeRemoverPivot;
    m_rollers = algaeRemoverRollers;
    m_setPos = setPos;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;

    addRequirements(algaeRemoverPivot, algaeRemoverRollers, elevator); // Add the required subsystems here
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getElevatorSetPosition() == m_setPos) {
      m_pivot.setPositionTrapazoidal(RemoverPositions.RaisedL2); // TODO: set positions
      m_rollers.setRemoverRollerSpeed(.5); // TODO: set speed
    } else {
      m_elevator.setElevatorHeightMotionProfile(m_setPos);
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setPositionTrapazoidal(RemoverPositions.Stowed);
    m_rollers.setRemoverRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

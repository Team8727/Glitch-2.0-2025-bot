// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralCmds;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployCoralCmd extends Command {
  private final Coral m_coral;
  private final LEDSubsystem m_ledSubsytem;
  private ElevatorPosition m_elevatorPosition;
  private boolean isFinished = false;

  /** Creates a new coralDeployer. */
  public DeployCoralCmd(Coral coral, LEDSubsystem ledSubsystem, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies
    m_coral = coral;
    m_ledSubsytem = ledSubsystem;
    m_elevatorPosition = elevator.getElevatorSetPosition();

    addRequirements(coral, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coral.elevatorUp = true;
    m_coral.setOuttakeSpeedDuty(.2);
    m_ledSubsytem.setPatternForDuration(m_ledSubsytem.coralPickup.reversed(), 2);
  }

  // Called every time the scheduler runs while the command is scheduled

  @Override
  public void execute() {
    if (!m_coral.getFrontCoralSensor()) {
      m_coral.stopDeployer();
      m_coral.elevatorUp = false;
      isFinished = true;
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

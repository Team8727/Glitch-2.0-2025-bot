// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.AlgaeIntake.TestPivot;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCmd extends Command {
  /** Creates a new GroundIntakeAlgae. */
  private final AlgaeIntakePivot m_algaeIntakePivot;
  private final AlgaeIntakeRollers m_algaeIntakeRollers;
  private final LEDSubsystem m_ledSubsystem;

  public IntakeAlgaeCmd(
    AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeRemoverPivot, LEDSubsystem ledSubsystem) {
    m_algaeIntakePivot = algaeIntakePivot;
    m_algaeIntakeRollers = algaeRemoverPivot;
    m_ledSubsystem = ledSubsystem;

    addRequirements(algaeIntakePivot, algaeRemoverPivot); // Add the required subsystems here
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("moving");
    m_algaeIntakeRollers.isMoving = true;
    m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.DOWN);
    m_algaeIntakeRollers.setRollerSpeedDuty(.8);
    m_ledSubsystem.setPatternForDuration(LEDSubsystem.algaePickup, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_algaeIntakeRollers.getAlgaeCheck()) {
      m_algaeIntakeRollers.isMoving = false;
      m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.HOME);
      
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.HOME);
    m_algaeIntakeRollers.setRollerSpeedDuty(0);
    // Go back to home position and stop rollers
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Finish when algae is detected
    return false;
  }
}

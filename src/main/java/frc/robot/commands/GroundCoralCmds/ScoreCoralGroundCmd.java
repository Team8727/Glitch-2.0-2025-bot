// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundCoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.LEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoralGroundCmd extends Command {
  /** Creates a new ScoreAlgaeProcessorCmd. */
  private final LEDSubsystem m_ledSubsystem;

  GroundIntakePivot intakePivot;
  GroundIntakeRollers intakeRollers;

  public ScoreCoralGroundCmd(
      GroundIntakePivot intakePivot, GroundIntakeRollers intakeRollers, LEDSubsystem ledSubsystem) {
    this.intakeRollers = intakeRollers;
    this.intakePivot = intakePivot;
    m_ledSubsystem = ledSubsystem;

    addRequirements(intakePivot, intakeRollers); // Add the required subsystems here
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the intake to score position, score the algae by running rollers, and then set the intake to home position.
    m_ledSubsystem.activateRandomNoise(LEDPatterns.coralPickup);
      intakePivot.setPositionCommand(12)
        .andThen(new WaitCommand(.2))
        .andThen(() -> intakeRollers.setSpeedDutyCycle(.3))
        .andThen(new WaitCommand(0.2))
        .andThen(this::cancel).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Go to home position (in robot) after scoring
    intakePivot.setPosition(0);
    intakeRollers.setSpeedDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_ledSubsystem.setPatternForDuration(LEDPatterns.coralPickup.reversed(), 0.5);
    return false;
  }
}

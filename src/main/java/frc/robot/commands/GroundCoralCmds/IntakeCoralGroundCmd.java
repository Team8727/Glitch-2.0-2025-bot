// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundCoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake.GroundIntakePivot;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.LEDs.GlitchLEDPatterns;
import frc.robot.subsystems.LEDs.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralGroundCmd extends Command {
  /** Creates a new GroundIntakeAlgae. */
  private final GroundIntakePivot intakePivot;
  private final GroundIntakeRollers intakeRollers;
  private final LEDSubsystem ledSubsystem;

  public IntakeCoralGroundCmd(
    GroundIntakeRollers intakeRollers, GroundIntakePivot intakePivot, LEDSubsystem ledSubsystem) {
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
    this.ledSubsystem = ledSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePivot.setPosition(110);
    intakeRollers.setSpeedDutyCycle(-.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setPositionCommand(5)
      .andThen(() -> intakeRollers.setSpeedDutyCycle(-0.1))
    .schedule();
//    // Go back to home position and stop rollers
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    ledSubsystem.setPatternForDuration(GlitchLEDPatterns.coralPickup, 0.5);;
    return false;
    // Finish when algae is detected
//    return intakeRollers.getCurrent() > 30;
  }
}

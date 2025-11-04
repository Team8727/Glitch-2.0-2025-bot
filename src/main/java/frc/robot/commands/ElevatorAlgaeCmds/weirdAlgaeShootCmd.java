//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.commands.ElevatorAlgaeCmds;
//
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.ElevatorCmds.SetElevatorHeightCmd;
//import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
//import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
//import frc.robot.subsystems.Elevator.Coral.Coral;
//import frc.robot.subsystems.Elevator.Elevator;
//import frc.robot.subsystems.LEDs.LEDPatterns;
//import frc.robot.subsystems.LEDs.LEDSubsystem;
//
///* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
//public class weirdAlgaeShootCmd extends SequentialCommandGroup {
//  /** Creates a new removeAlgae. */
//  public weirdAlgaeShootCmd(AlgaeRemoverPivot algaeRemoverPivot, AlgaeRemoverRollers algaeRemoverRollers, Elevator elevator, LEDSubsystem ledSubsystem, LEDPatterns ledPatterns, Coral coral) {
//    addRequirements(algaeRemoverPivot, algaeRemoverRollers, elevator); // Add the required subsystems here
//
//    addCommands(
//      new SetElevatorHeightCmd(Elevator.ElevatorPosition.L3, elevator, coral, ledSubsystem, ledPatterns),
//      new InstantCommand(() -> elevator.isHoming = true), // TODO: set positions
//      new InstantCommand(() -> elevator.setDutyCycle(.5)),
//      new ParallelCommandGroup(
//        new InstantCommand(() -> algaeRemoverPivot.setPositionTrapazoidal(AlgaeRemoverPivot.RemoverPositions.Fling)), // TODO: set positions
//        new InstantCommand(() -> algaeRemoverRollers.setRemoverRollerSpeed(.2)), // TODO: set speed
//        new InstantCommand(() -> coral.setFrontSpeedDuty(-.5))),
//      new WaitCommand(.5),
//      new InstantCommand(() -> elevator.isHoming = false) // Stop homing after the command is done
//      );
//  }
//}

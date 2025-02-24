// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotState;
import frc.robot.StateMachine;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OperatorStore;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCommand extends SequentialCommandGroup {
  Drive drive;
  ActuatorSubsystem actuator;
  ElevatorSubsystem elevator;
  OperatorStore buffer;
  Supplier<BotState> bufferedState;
  LimelightSubsystem limelight;
  IntakeSubsystem intake;
  StateMachine stateMachine;
  /** Creates a new AutoScoreCommand. */
  public AutoScoreCommand(
      Drive d,
      ActuatorSubsystem a,
      ElevatorSubsystem e,
      IntakeSubsystem i,
      OperatorStore b,
      Supplier<BotState> tState,
      LimelightSubsystem l,
      StateMachine s) {
    drive = d;
    actuator = a;
    elevator = e;
    buffer = b;
    bufferedState = tState;
    limelight = l;
    stateMachine = s;
    intake = i;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new StateMachineCommand(elevator, actuator, stateMachine, bufferedState.get()),
        new DriveToReefCommand(drive, limelight, buffer, () -> elevator.getSlowDownMult()),
        new InstantCommand()
        /*intake.setSpeed(0.75).withTimeout(0.25)*/ );
  }
}

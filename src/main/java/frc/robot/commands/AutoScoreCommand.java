// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateMachine;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCommand extends SequentialCommandGroup {
  Drive drive;
  ActuatorSubsystem actuator;
  ElevatorSubsystem elevator;
  AutoScoreSelection storedState;
  LimelightSubsystem lLimelight, rLimelight;
  ScoringSubsystem scoring;
  StateMachine stateMachine;
  /** Creates a new AutoScoreCommand. */
  public AutoScoreCommand(
      Drive d,
      ActuatorSubsystem a,
      ElevatorSubsystem e,
      ScoringSubsystem s,
      AutoScoreSelection b,
      LimelightSubsystem ll,
      LimelightSubsystem rl,
      StateMachine sm) {
    drive = d;
    actuator = a;
    elevator = e;
    storedState = b;
    lLimelight = ll;
    rLimelight = rl;
    stateMachine = sm;
    scoring = s;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new BufferedStateMachineCommand(elevator, actuator, stateMachine, storedState),
        new DriveToReefCommand(
            drive, lLimelight, rLimelight, storedState, () -> elevator.getSlowDownMult()),
        scoring.setSpeedAndState(-0.75, false).withTimeout(0.5));
  }
}

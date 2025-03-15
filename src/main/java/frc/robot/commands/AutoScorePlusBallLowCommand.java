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
public class AutoScorePlusBallLowCommand extends SequentialCommandGroup {
  Drive drive;
  ActuatorSubsystem actuator;
  ElevatorSubsystem elevator;
  AutoScoreSelection storedState;
  LimelightSubsystem lLimelight, rLimelight;
  ScoringSubsystem score;
  StateMachine stateMachine;

  /** Creates a new AutoScoreCommand. */
  public AutoScorePlusBallLowCommand(
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
    score = s;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new BufferedStateMachineCommand(elevator, actuator, stateMachine, storedState),
        // actuator.setGoal(20).withTimeout(0.2),
        // elevator.setGoal(38.5).withTimeout(0.2),
        // new AutoAlignCommand(drive, lLimelight, 0, LimelightConstants.leftLimelightName)
        //     .withTimeout(1.5),
        // new AutoAlignTeleop(drive, lLimelight, storedState).withTimeout(3),
        score.setSpeedAndState(-0.75, false).withTimeout(0.5),
        actuator.setGoal(15).withTimeout(0.2),
        elevator.setGoal(10).withTimeout(0.5),
        actuator.setGoal(34).withTimeout(0.5),
        score.setSpeedAndState(-1, true).withTimeout(0.5),
        actuator.setGoal(19).withTimeout(0.5),
        elevator.setGoal(0.5).withTimeout(0.5),
        score.setSpeedAndState(-0.25, true));
  }
}

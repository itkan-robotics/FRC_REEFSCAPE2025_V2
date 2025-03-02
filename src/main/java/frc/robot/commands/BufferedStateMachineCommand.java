// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.BotState;
import frc.robot.StateMachine;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.AutoScoreSelection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BufferedStateMachineCommand extends Command {
  ActuatorSubsystem pivot;
  ElevatorSubsystem elevator;
  AutoScoreSelection storedState;
  StateMachine stateMachine;
  BotState currentState;

  /**
   * Creates a new BufferedStateMachineCommand.
   *
   * <p>This is an alternative StateMachine Command that takes in an AutoScoreSelection instead of a
   * target state, hopefully fixing the issue of the target state being unable to be read.
   */
  public BufferedStateMachineCommand(
      ElevatorSubsystem e, ActuatorSubsystem p, StateMachine sMachine, AutoScoreSelection b) {
    this.elevator = e;
    this.pivot = p;
    this.storedState = b;
    this.stateMachine = sMachine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = Constants.toBotState(storedState.getBotStateInt());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSetpoint(currentState.getElevatorSetpoint());
    pivot.setSetpoint(currentState.getActuatorSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return ((elevator.getPosition() - currentState.getElevatorSetpoint()) < 0.1)
    //     && ((pivot.getAvgPosition() - currentState.getActuatorSetpoint()) < 0.1);
    return (Math.abs(elevator.getPosition() - currentState.getElevatorSetpoint()) < 15.0)
        && (Math.abs(pivot.getAvgPosition() - currentState.getActuatorSetpoint()) < 5.0);
  }
}

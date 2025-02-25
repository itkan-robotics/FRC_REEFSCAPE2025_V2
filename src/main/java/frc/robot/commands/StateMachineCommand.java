// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.BotState;
import frc.robot.StateMachine;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateMachineCommand extends Command {
  ActuatorSubsystem pivot;
  ElevatorSubsystem elevator;
  BotState targetState;
  StateMachine stateMachine;

  /** Creates a new StateMachineCommand. */
  public StateMachineCommand(
      ElevatorSubsystem e, ActuatorSubsystem p, StateMachine stateMachine, BotState targetState) {
    this.elevator = e;
    this.pivot = p;
    this.targetState = targetState;
    this.stateMachine = stateMachine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("TargetState", Constants.toString(targetState));
    elevator.setSetpoint(targetState.getElevatorSetpoint());
    pivot.setSetpoint(targetState.getActuatorSetpoint());

    if (elevator.getPosition() < targetState.getElevatorSetpoint()) {
      elevator.setSetpoint(targetState.getElevatorSetpoint());
      if (elevator.setpointReached()) {
        pivot.setSetpoint(targetState.getActuatorSetpoint());
      }
    } else {
      pivot.setSetpoint(targetState.getActuatorSetpoint());
      if (pivot.setpointReached()) {
        elevator.setSetpoint(targetState.getElevatorSetpoint());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.setpointReached() && pivot.setpointReached();
  }
}

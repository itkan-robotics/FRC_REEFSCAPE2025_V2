// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.cert.Extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.util.MachineStates.BotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartArmCommand extends Command {
  /** Creates a new SmartArmCommand. */
  private ShoulderSubsystem shoulder;
  private ExtensionSubsystem extension;
  private BotState state;
  private double coaxialRatio = 5/0.3;
  private boolean isFinished = false; 

  public SmartArmCommand(ShoulderSubsystem shoulder, ExtensionSubsystem extension, BotState state) {
    this.shoulder = shoulder;
    this.extension = extension;
    this.state = state;
    addRequirements(this.shoulder, this.extension);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setSetpoint(state.getShoulderSetpoint());
    extension.setSetpoint(state.getShoulderSetpoint() * coaxialRatio);
    if(shoulder.setpointReached()){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

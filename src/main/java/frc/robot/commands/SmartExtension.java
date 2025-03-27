// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.util.MachineStates.BotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartExtension extends Command {
  /** Creates a new SmartExtension. */
  private ExtensionSubsystem extension;
  private WristSubsystem wrist;
  private BotState state;
  public SmartExtension( ExtensionSubsystem extension, WristSubsystem wrist, BotState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extension = extension;
    this.wrist = wrist;
    this.state = state;
    addRequirements(this.extension, this.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extension.setSetpoint(state.getExtensionSetpoint());
    if(extension.getPosition() < 10){
      wrist.setSetpoint(MathUtil.clamp(state.getWristSetpoint(), -.1, 0.2));
    }
    else{
      wrist.setSetpoint(state.getWristSetpoint());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

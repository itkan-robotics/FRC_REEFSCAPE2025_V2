// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import static frc.robot.util.MachineStates.HOME;
// import static frc.robot.util.MachineStates.INTAKE;

// import static frc.robot.util.MachineStates.HOME;
import static frc.robot.util.MachineStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FullArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartIntake extends Command {
  /** Creates a new SmartIntake. */
  IntakeSubsystem intake;

  FullArmSubsystem fullArm;
  boolean m_end;

  /**
   * Start intaking and move the arm to the {@link frc.robot.util.MachineStates#INTAKE INTAKE}
   * position until a coral is detected, at which point move the arm to the {@link
   * frc.robot.util.MachineStates#HOME HOME} position
   */
  public SmartIntake(IntakeSubsystem i, FullArmSubsystem f) {
    intake = i;
    fullArm = f;
    addRequirements(intake, fullArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.isIntaked()) {
      fullArm.setGoalVoid(HOME, true);
    } else {
      fullArm.setGoalVoid(INTAKE, false);
      intake.setIntake(1);
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

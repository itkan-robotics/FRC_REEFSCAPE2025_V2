// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DisabledInstantCommand extends InstantCommand {
  public DisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

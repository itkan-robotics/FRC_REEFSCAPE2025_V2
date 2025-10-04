// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Superstructure extends SubsystemBase {

  public enum CurrentSuperState {
    HOME,
    GROUND_CORAL_INTAKE,
    GROUND_ALGAE_INTAKE,
    STATION_INTAKE,
    L1,
    L2,
    L3,
    L4,
    PRE_CLIMB,
    CLIMB,
    PROCESSOR,
    BARGE,
    HIGH_ALGAE,
    LOW_ALGAE
  }

  public enum WantedSuperState {
    HOME,
    GROUND_CORAL_INTAKE,
    GROUND_ALGAE_INTAKE,
    STATION_INTAKE,
    L1,
    L2,
    L3,
    L4,
    PRE_CLIMB,
    CLIMB,
    PROCESSOR,
    BARGE,
    HIGH_ALGAE,
    LOW_ALGAE
  }
  /** Creates a new Superstructure. */
  public Superstructure(
    Drive drive,
    FullArmSubsystem fullArm,
    IntakeSubsystem intake,
    ClimbSubsystem climb
  ) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  private CurrentSuperState handleStateTransitions() {
    return CurrentSuperState.HOME;
  }

  private CurrentSuperState applyStates() {
    return CurrentSuperState.HOME;
  }
}

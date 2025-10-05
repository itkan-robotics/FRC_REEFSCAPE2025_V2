// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  WantedSuperState wantedSuperState = WantedSuperState.HOME;
  CurrentSuperState currentSuperState = CurrentSuperState.HOME;
  CurrentSuperState previouSuperState = CurrentSuperState.HOME;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, FullArmSubsystem fullArm, IntakeSubsystem intake, ClimbSubsystem climb) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    applyStates();
  }

  public void setWantedSuperState(WantedSuperState wantedState) {}

  private void tryState(WantedSuperState desiredState) {
    switch (desiredState) {
      default:
        tryState(WantedSuperState.HOME);
        break;
      case HOME:
        switch (currentSuperState) {
          case CLIMB:
            currentSuperState = CurrentSuperState.PRE_CLIMB;
            break;
          default:
            currentSuperState = CurrentSuperState.HOME;
            break;
        }
        break;
      case PRE_CLIMB:
        switch (currentSuperState) {
          case CLIMB:
          case HOME:
            currentSuperState = CurrentSuperState.PRE_CLIMB;
            break;
          default:
            currentSuperState = CurrentSuperState.HOME;
        }
      case CLIMB:
        switch (currentSuperState) {
          case PRE_CLIMB:
            currentSuperState = CurrentSuperState.CLIMB;
            break;
          default:
            currentSuperState = CurrentSuperState.PRE_CLIMB;
        }
    }
  }

  private void applyStates() {
    // TO-DO: 2910-style methods
  }

  private boolean doesMatchState() {
    // TO-DO: Make doesMatchState() for each subsystem
    return true;
  }
}

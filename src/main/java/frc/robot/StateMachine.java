// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.BotState.*;

import frc.robot.Constants.BotState;

/** Add your docs here. */
public class StateMachine {
  BotState currState;

  public void setState(BotState state) {
    currState = state;
  }

  public BotState getState() {
    return currState;
  }

  public BotState tryState(BotState state) {
    switch (state) {
      case HOME:
        setState(state);
        return HOME;
      case L1:
        switch (currState) {
          case HOME:
            setState(state);
            return L1;
          default:
            return currState;
        }
      case L2:
        switch (currState) {
          case HOME:
            setState(state);
            return L2;
          default:
            return currState;
        }
      case L3:
        switch (currState) {
          case HOME:
            setState(state);
            return L3;
          default:
            return currState;
        }
      case L4:
        switch (currState) {
          case HOME:
            setState(state);
            return L4;
          default:
            return currState;
        }
      case CLIMB:
        switch (currState) {
          case HOME:
            setState(state);
            return CLIMB;
          default:
            return currState;
        }
      case RESET:
        switch (currState) {
          case HOME:
          case L1:
          case L2:
          case L3:
          case L4:
            setState(state);
            return RESET;
          default:
            return currState;
        }
    }
    // If all else fails, stay at the same state
    return currState;
  }
}

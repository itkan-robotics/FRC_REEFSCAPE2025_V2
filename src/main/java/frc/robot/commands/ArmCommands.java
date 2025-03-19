package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.util.MachineStates.BotState;

public class ArmCommands {

  private ArmCommands() {}

  private static BotState clampArmSetpoints(
      double shoulderPos, double extensionPos, double wristPos) {
    return new BotState(
        "CLAMPED",
        MathUtil.clamp(
            shoulderPos,
            ArmConstants.ShoulderConstants.MIN_SHOULDER_ROTATION_POS,
            ArmConstants.ShoulderConstants.MAX_SHOULDER_ROTATION_POS),
        MathUtil.clamp(
            extensionPos,
            ArmConstants.ExtensionConstants.MIN_EXTENSION_POS,
            ArmConstants.ExtensionConstants.MAX_EXTENSION_POS),
        MathUtil.clamp(
            wristPos,
            ArmConstants.WristConstants.MIN_WRIST_ROTATION_POS,
            ArmConstants.WristConstants.MAX_WRIST_ROTATION_POS),
        100);
  }

  private static boolean validState(BotState stateA, BotState stateB) {
    boolean isValid = true;

    // Logic for determining whether the robot can go from state A to state B
    return isValid;
  }

  public static Command setArmGoal(
      ShoulderSubsystem shoulder,
      ExtensionSubsystem extension,
      WristSubsystem wrist,
      BotState currentState,
      BotState requestedState) {
    Subsystem[] requirements = new Subsystem[] {shoulder, extension, wrist};

    return Commands.run(
        () -> {
          if (validState(currentState, requestedState)) {
            BotState clampedBotState = requestedState;
            // clampArmSetpoints(
            //     requestedState.getShoulderSetpoint(),
            //     requestedState.getExtensionSetpoint(),
            //     requestedState.getWristSetpoint());

            shoulder.setGoal(clampedBotState.getShoulderSetpoint());
            extension.setGoal(clampedBotState.getExtensionSetpoint());
            wrist.setGoal(clampedBotState.getWristSetpoint());
          }
        },
        requirements);
  }
}

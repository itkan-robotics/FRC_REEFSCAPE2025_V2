package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.util.MachineStates.BotState;
import frc.robot.util.Util;

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

  public static Command setScoringPosition(
      ShoulderSubsystem shoulder, ExtensionSubsystem extension, BotState requestedState) {
    return Commands.run(
        () -> {
          double initialShoulderPosition = shoulder.getPosition();
          SmartDashboard.putNumber("armCommands/initialShoulderPosition", initialShoulderPosition);
          SmartDashboard.putNumber(
              "armCommands/requestedShoulderPosition", requestedState.getShoulderSetpoint());
          SmartDashboard.putString("armCommands/currentPos", "0");

          if (initialShoulderPosition < requestedState.getShoulderSetpoint()) {
            SmartDashboard.putString("armCommands/currentPos1", "1");
            shoulder.setSetpoint(requestedState.getShoulderSetpoint());
            while (!shoulder.setpointReached()) {
              SmartDashboard.putString("armCommands/currentPos2", "2");
            }
            SmartDashboard.putString("armCommands/currentPos3", "3");
            extension.setSetpoint(requestedState.getExtensionSetpoint());
          }

          SmartDashboard.putString("armCommands/currentPos4", "4");
        });
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

            // If the requested position's extension is higher than the current state, pivot >>
            // extend
            if (requestedState.getExtensionSetpoint() > currentState.getExtensionSetpoint()) {

              System.out.println("before the first while loop");
              shoulder.setSetpoint(requestedState.getShoulderSetpoint());

              while (!shoulder.setpointReached()) {
                // Wait for shoulder to get within tolerance
                System.out.println("inside the first while loop");
              }

              System.out.println("after the first while loop");
              extension.setSetpoint(clampedBotState.getExtensionSetpoint());
              // wrist.setGoal(clampedBotState.getWristSetpoint());
            } else {

              System.out.println("before the second while loop");
              extension.setSetpoint(clampedBotState.getExtensionSetpoint());
              wrist.setSetpoint(clampedBotState.getWristSetpoint());
              while (Util.withinTolerance(
                  extension.getPosition(), clampedBotState.getExtensionSetpoint(), 1.0)) {
                // Wait for shoulder to get within tolerance
                System.out.println("inside the second while loop");
              }
              System.out.println("after the second while loop");
              shoulder.setSetpoint(clampedBotState.getShoulderSetpoint());
            }
          }
        }); // ,requirements
  }
}

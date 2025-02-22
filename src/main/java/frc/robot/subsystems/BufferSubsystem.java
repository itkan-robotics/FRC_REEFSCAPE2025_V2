// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BotState;
import frc.robot.Constants.LimelightConstants.OffsetPipelines;
import frc.robot.util.LimelightHelpers;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class BufferSubsystem extends SubsystemBase {
  private static BotState operatorState = BotState.RESET;
  private static OffsetPipelines operatorBranch = OffsetPipelines.CENTER;
  private static double operatorAngle = 0.0;
  private HashMap<Double, Integer> reefAngles = new HashMap<Double, Integer>();

  /** Creates a new BuffsetSubsystem. */
  public BufferSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Buffer/BotState", Constants.getBotStateAsString(operatorState));
    SmartDashboard.putNumber(
        "Buffer/Pipeline", LimelightHelpers.getCurrentPipelineIndex("limelight"));
    SmartDashboard.putNumber("Buffer/TargetAngle", operatorAngle);
  }

  public void createReefHashMap() {
    int blueAllianceTags = !Constants.isRedAlliance() ? 11 : 0;
    reefAngles.put(-1.0, -1);
    reefAngles.put(-60.0, 6 + blueAllianceTags); // 17
    reefAngles.put(0.0, 7 + blueAllianceTags); // 18
    reefAngles.put(60.0, 8 + blueAllianceTags); // 19
    reefAngles.put(120.0, 9 + blueAllianceTags); // 20
    reefAngles.put(180.0, 10 + blueAllianceTags); // 21
    reefAngles.put(-120.0, 11 + blueAllianceTags); // 22
  }

  public int getReefAprilTag(double angle) {
    int ID = (reefAngles.get(angle) == null) ? 0 : reefAngles.get(angle);
    return ID;
  }

  public Command setBotStateCommand(BotState cState) {
    return run(
        () -> {
          setBotState(cState);
        });
  }
  ;

  public Command setOffsetPipelineCommand(OffsetPipelines cPipeline) {
    return run(
        () -> {
          setOffsetPipeLine(cPipeline);
        });
  }
  ;

  public Command setOperatorAngleCommand(DoubleSupplier rxSupplier, DoubleSupplier rySupplier) {
    return run(
        () -> {
          setOperatorAngle(rxSupplier, rySupplier);
        });
  }
  ;

  public void setBotState(BotState state) {
    operatorState = state;
  }

  public void setOffsetPipeLine(OffsetPipelines pipeline) {
    operatorBranch = pipeline;
  }

  public void setOperatorAngle(DoubleSupplier rJoystickX, DoubleSupplier rJoystickY) {
    double DEADBAND = 0.1;
    double xDeadband = MathUtil.applyDeadband(rJoystickX.getAsDouble(), DEADBAND);
    double yDeadband = MathUtil.applyDeadband(rJoystickY.getAsDouble(), DEADBAND);
    double opAngle = Math.toDegrees(Math.atan2(xDeadband, yDeadband)) - 90;

    // Round to nearest 60 degrees
    operatorAngle = Math.round(opAngle / 60.0) * 60.0;
  }

  public BotState getTargetBotState() {
    return operatorState;
  }

  public OffsetPipelines getTargetBranch() {
    return operatorBranch;
  }

  public double getTargetReefAngle() {
    return operatorAngle;
  }

  /**
   * @return The target AprilTag given the target angle. An angle that isn't in the HashMap will
   *     return an AprilTag ID of 0
   */
  public int getTargetAprilTag() {
    return getReefAprilTag(operatorAngle);
  }
}

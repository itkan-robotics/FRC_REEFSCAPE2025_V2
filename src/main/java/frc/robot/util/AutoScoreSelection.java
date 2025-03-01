// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.BotState;
import frc.robot.Constants.LimelightConstants;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class AutoScoreSelection {
  private static int operatorStateInt = -1;
  private static int operatorLimelight = 0;
  private static double operatorReefAngle = 0.0;
  private static boolean baseAutoTurn = false;
  private HashMap<Double, Integer> reefAnglesToIDs = new HashMap<Double, Integer>();
  boolean working;

  /** Creates a new AutoScoreSelection Class. */
  public AutoScoreSelection() {
    createReefAnglestoIDsHashMap();
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("storedState/RobotState", "L" + operatorStateInt);
    SmartDashboard.putNumber("storedState/Pipeline", operatorLimelight);
    SmartDashboard.putNumber("storedState/TargetAngle", operatorReefAngle);
    SmartDashboard.putBoolean("storedState/ShouldAutoTurn", baseAutoTurn);
  }

  /**
   * Create a hash map that takes in an angle and returns the corresponding AprilTag. Used when
   * determining whether the primary seen ATag is the correct one during Auto Score
   */
  public void createReefAnglestoIDsHashMap() {
    reefAnglesToIDs.put(-1.0, -1);
    reefAnglesToIDs.put(-60.0, 6); // 17
    reefAnglesToIDs.put(0.0, 7); // 18
    reefAnglesToIDs.put(60.0, 8); // 19
    reefAnglesToIDs.put(120.0, 9); // 20
    reefAnglesToIDs.put(180.0, 10); // 21
    reefAnglesToIDs.put(-120.0, 11); // 22
  }

  /** Set the pipeline the limelight should use during auto-align */
  public void setLimelightPipeLine(int pipeline) {
    operatorLimelight = pipeline;
  }

  /**
   * Alternative method of setting a pipeline that takes in a string based on what branch the robot
   * should go to
   */
  public void setLimelightPipeLine(String placement) {
    int cPipeline = 0;
    switch (placement.toLowerCase()) {
      case "left":
        cPipeline = LimelightConstants.LEFT_BRANCH_PIPELINE;
        break;

      case "right":
        cPipeline = LimelightConstants.RIGHT_BRANCH_PIPELINE;
        break;

      default:
        cPipeline = LimelightConstants.RIGHT_BRANCH_PIPELINE;
        break;
    }
    operatorLimelight = cPipeline;
  }

  /**
   * Method that takes in a joystick's x and y values and stores the recorded angle of the
   * joysticks. If joystick is not moving (i.e. within deadband range) the resulting angle will be
   * 0.0
   */
  public void setOperatorReefAngle(DoubleSupplier rJoystickX, DoubleSupplier rJoystickY) {
    double DEADBAND = 0.1;
    double xDeadband = MathUtil.applyDeadband(rJoystickX.getAsDouble(), DEADBAND);
    double yDeadband = MathUtil.applyDeadband(rJoystickY.getAsDouble(), DEADBAND);

    // If joystick input is likely caused by deadband, set the angle to 0.0
    if (Math.abs(xDeadband + yDeadband) <= 0.05) {
      operatorReefAngle = 0.0;
    } else {
      // Else, set the angle to the calculated angle, rounded to the nearest 60 degrees
      double opAngle = Math.toDegrees(Math.atan2(xDeadband, yDeadband)) - 90;
      operatorReefAngle = Math.round(opAngle / 60.0) * 60.0;
    }
  }

  /**
   * @return The pipeline the operator has inputted from NetworkTables. Defaults to the center
   *     pipeline.
   */
  public int getLimelightTargetPipeline() {
    return operatorLimelight; // (int) SmartDashboard.getNumber("storedState/Pipeline", 0);
  }

  public void setAutoTurn(boolean shouldTurn) {
    baseAutoTurn = shouldTurn;
  }

  public void invertAutoTurn() {
    baseAutoTurn = !baseAutoTurn;
  }

  public boolean getAutoTurn() {
    return baseAutoTurn;
  }

  /**
   * Set the stored bot state using an integer value, where the number corresponds to the level.
   * More values will be added in the future
   */
  public void setBotStateInt(int stateInt) {
    operatorStateInt = stateInt;
  }

  /**
   * @return The store state as an integer, with a default value of -1. Corresponding values can be
   *     found in {@link frc.robot.Constants#toBotState(int)}
   */
  public int getBotStateInt() {
    return operatorStateInt; // (int) SmartDashboard.getNumber("storedState/RobotState", -1);
  }

  /**
   * @return the store state as a BotState enum, with a default value of RESET.
   */
  public BotState getBotState() {
    return Constants.toBotState(operatorStateInt); // Constants.toBotState((int)
    // SmartDashboard.getNumber("storedState/RobotState", -1));
  }

  /**
   * @return The stored target angle of the reef
   */
  public double getTargetReefAngle() {
    return operatorReefAngle;
  }

  /** Takes in an angle and returns the corresponding AprilTag ID, relative to the alliance */
  public int getReefAprilTag(double angle) {
    int ID = (reefAnglesToIDs.get(angle) == null) ? 0 : reefAnglesToIDs.get(angle);
    return ID;
  }

  /**
   * @return The target AprilTag given the target angle. An angle that isn't in the HashMap will
   *     return an AprilTag ID of 0
   */
  public int getTargetAprilTag() {
    return getReefAprilTag(operatorReefAngle);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.LimelightConstants.LEFT_BRANCH_PIPELINE;
import static frc.robot.Constants.LimelightConstants.leftLimelightName;
import static frc.robot.Constants.LimelightConstants.rightLimelightName;
import static frc.robot.util.MachineStates.INTAKE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.MachineStates.BotState;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

/**
 * Class that stores various information used in the {@link frc.robot.commands.SmartAlignProfiledPID
 * SmartAlignProfiledPID} command, including:
 *
 * <ul>
 *   <li>L1-4 state
 *   <li>Left/Right Branch
 *   <li>Reef Angle
 *   <li>Boolean for turning automatically to the desired reef face
 * </ul>
 */
public class AutoScoreSelection {
  private static int operatorLimelight = 0;
  private static double operatorReefAngle = 0.0;
  private static BotState desiredState = INTAKE;
  private static boolean baseAutoTurn = false;
  private HashMap<Double, Integer> reefAnglesToIDs = new HashMap<Double, Integer>();
  boolean working;
  private static double lastUpdatedFPGA = 0.0;

  /** Creates a new AutoScoreSelection Class. */
  public AutoScoreSelection() {
    createReefAnglestoIDsHashMap();
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("storedState/RobotState", "L" + desiredState.getId());
    SmartDashboard.putNumber("storedState/Pipeline", getLimelightTargetPipeline());
    SmartDashboard.putNumber("storedState/TargetAngle", getTargetReefAngle());
    SmartDashboard.putBoolean("storedState/ShouldAutoTurn", getAutoTurn());
  }

  public void refreshLastUpdated() {
    lastUpdatedFPGA = Timer.getFPGATimestamp();
  }

  public double getLastUpdated() {
    return lastUpdatedFPGA;
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
    reefAnglesToIDs.put(-240.0, 9); // 20
    reefAnglesToIDs.put(-180.0, 10); // 21
    reefAnglesToIDs.put(-120.0, 11); // 22
  }

  /** Set the pipeline the limelight should use during auto-align */
  public void setLimelightPipeLine(int pipeline) {
    operatorLimelight = pipeline;
    refreshLastUpdated();
  }

  /**
   * Alternative method of setting a pipeline that takes in a string based on what branch the robot
   * should go to
   *
   * @param placement What branch the robot should go to
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
    refreshLastUpdated();
  }

  /**
   * Method that takes in a joystick's x and y values and stores the recorded angle of the
   * joysticks. If joystick is not moving (i.e. within deadband range) the resulting angle will be
   * 0.0
   */
  public void setOperatorReefAngle(DoubleSupplier rJoystickX, DoubleSupplier rJoystickY) {
    double DEADBAND = 0.2;
    double xDeadband = MathUtil.applyDeadband(rJoystickX.getAsDouble(), DEADBAND);
    double yDeadband = MathUtil.applyDeadband(rJoystickY.getAsDouble(), DEADBAND);

    // If joystick input is likely caused by deadband, set the angle to 0.0
    if (xDeadband == 0.0 && yDeadband == 0.0) {
      // operatorReefAngle = 0.0;
    } else {
      // Else, set the angle to the calculated angle, rounded to the nearest 60 degrees
      double opAngle = Math.toDegrees(Math.atan2(xDeadband, yDeadband)) - 90;
      operatorReefAngle = Math.round(opAngle / 60.0) * 60.0;
    }
    refreshLastUpdated();
  }

  /**
   * @return The pipeline the operator has inputted from NetworkTables. Defaults to the center
   *     pipeline.
   */
  public int getLimelightTargetPipeline() {
    return operatorLimelight; // (int) SmartDashboard.getNumber("storedState/Pipeline", 0);
  }

  /**
   * Getter method that returns the target limelight's ID name
   *
   * <ul>
   *   <li>Left Limelight Name: {@value frc.robot.Constants.LimelightConstants#leftLimelightName}
   *   <li>Right Limelight Name: {@value frc.robot.Constants.LimelightConstants#rightLimelightName}
   *       *
   * </ul>
   */
  public String getTargetLimelight() {
    return operatorLimelight == LEFT_BRANCH_PIPELINE ? rightLimelightName : leftLimelightName;
  }

  /**
   * @param shouldTurn Sets the {@link #baseAutoTurn} boolean to true or false
   */
  public void setAutoTurn(boolean shouldTurn) {
    baseAutoTurn = shouldTurn;
    refreshLastUpdated();
  }

  /** Inverts the {@link #baseAutoTurn} boolean */
  public void invertAutoTurn() {
    baseAutoTurn = !baseAutoTurn;
    refreshLastUpdated();
  }

  /**
   * @return {@link #baseAutoTurn}
   */
  public boolean getAutoTurn() {
    return baseAutoTurn;
  }

  /**
   * @param dState What {@link #desiredState} should be set to
   */
  public void setBotState(BotState dState) {
    desiredState = dState;
    refreshLastUpdated();
  }

  /**
   * @return the store state as a {@link frc.robot.util.MachineStates.BotState BotState} with a
   *     default value of {@link frc.robot.util.MachineStates#RESET RESET}.
   */
  public BotState getBotState() {
    return desiredState; // Constants.toBotState((int)
    // SmartDashboard.getNumber("storedState/RobotState", -1));
  }

  /**
   * @return The stored target angle of the reef
   */
  public double getTargetReefAngle() {
    return (operatorReefAngle == -240) ? 120 : operatorReefAngle;
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TagOffsets;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive;

import java.util.HashMap;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  Timer lastTargetTime = new Timer();
  public boolean targetSeen = false;
  private final double imageWidth = 320;
  private final double imageHeight = 240;
  private final double fovHorizontalDegrees = 59.6;
  private final double fovVerticalDegrees = 49.7;
  private final double areaMultiplier = 1.5;
  public boolean limelightHeadingGood = true;
  private HashMap<Integer, Double> reefAngles = new HashMap<Integer, Double>();
  public double[] tagPose = {0, 0, 0, 0};

  public LimelightSubsystem() {
    createReefHashMap();
  }

  public Command setLimelight() {
    return run(
        () -> {
          limelightHeadingGood = true;
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AprilTag ID", getID());
    if (table.getEntry("tv").getDouble(0.0) == 1) {
      lastTargetTime.restart();
      targetSeen = true;
    }
    if (lastTargetTime.get() > .1) {
      targetSeen = false;
    }
  }

  /***************************************************************************************
   * Function that gets the target's position relative to the robot
   * (Based on 6328 Mechanical Advantage's idea in <a href=https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85>this</a> CD post)
   *  <p> Last Updated by Abdullah Khaled, 2/1/2025
   * @return
   *************************************************************************************/
  @SuppressWarnings("unused")
  public Pose2d getTagEstimatedPosition(Drive drive) {
    double[] targetPose =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("targetpose_robotspace")
            .getDoubleArray(new double[6]);
    ;

    double targetTX = targetPose[0];
    double targetTY = targetPose[1];
    double targetTZ = targetPose[2];
    Rotation2d tAngleToRobot = Rotation2d.fromRadians(Math.atan2(targetTX, targetTZ));
    // System.out.println("targetRotation: " + tAngleToRobot.getDegrees());
    double distanceToTarget =
        getPrimaryFiducial(LimelightHelpers.getRawFiducials("limelight")).distToRobot;
    double absRotation =
        -1.0
                * drive.getHeadingDegrees()
            - getReefAngle();

    // System.out.println("absRotation: " + absRotation);
    Pose2d targetPose2d = new Pose2d(targetTZ, -targetTX, Rotation2d.fromDegrees(absRotation));
    return targetPose2d == null ? new Pose2d(-1, -1, Rotation2d.fromDegrees(0)) : targetPose2d;
  }

  // Helper Methods

  /***************************************************************************************
   * Creates the hashMap for the reef AprilTags based on alliance;
   * if blue alliance, adds 11 to the AprilTag keys to account for different IDs
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   **************************************************************************************/

  public void createReefHashMap() {
    int blueAllianceTags = !isRedAlliance() ? 11 : 0;
    reefAngles.put(-1, -1.0);
    reefAngles.put(6 + blueAllianceTags, -60.0); // 17
    reefAngles.put(7 + blueAllianceTags, 0.0); // 18
    reefAngles.put(8 + blueAllianceTags, 60.0); // 19
    reefAngles.put(9 + blueAllianceTags, 120.0); // 20
    reefAngles.put(10 + blueAllianceTags, 180.0); // 21
    reefAngles.put(11 + blueAllianceTags, -120.0); // 22
  }

  public void setAprilTagOffset(TagOffsets offset) {
    LimelightHelpers.setFiducial3DOffset(
        "limelight", 0.0, Units.inchesToMeters(offset.getHorizontalOffsetInches()), 0.0);
  }

  public double getReefAngle() {
    double angle = (reefAngles.get(getID()) == null) ? -1.0 : reefAngles.get(getID());
    // System.out.println("id: " + getID());
    return angle;
  }

  /***************************************************************************************
   * Get the current alliance as specific in the Driver Station.
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   * @return The current alliance, where red is true and blue is false
   **************************************************************************************/

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        return true;
      } else if (alliance.get() == DriverStation.Alliance.Blue) {
        return false;
      }
    }
    return false;
  }

  public double getX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getLatency() {
    return table.getEntry("tl").getDouble(0.0) + table.getEntry("cl").getDouble(0.0);
  }

  public double getY() {
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getArea() {
    return table.getEntry("ta").getDouble(0.0);
  }

  public boolean hasTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  public int getID() {
    return (int) table.getEntry("tid").getDouble(0.0);
  }

  public static RawFiducial getPrimaryFiducial(RawFiducial[] fiducials) {
    RawFiducial primaryFiducial = new RawFiducial(-1, 0, 0, 0, 0, 0, 0);
    int primaryID = (int) table.getEntry("tid").getDouble(0.0);
    if (primaryID != -1) {
      for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == primaryID) primaryFiducial = fiducial;
      }
    }
    return primaryFiducial;
  }

  public boolean canSeeTarget() {
    return targetSeen;
  }

  public void dynamicCropping() {

    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    double xCenter = imageWidth / 2 + (tx / fovHorizontalDegrees * imageWidth);
    double yCenter = imageHeight / 2 + (ty / fovVerticalDegrees * imageHeight);

    double cropSize = Math.sqrt(ta / 100 * imageWidth * imageHeight) * areaMultiplier;
    double left = Math.max(0, xCenter - cropSize / 2);
    double right = Math.min(imageWidth, xCenter + cropSize / 2);
    double top = Math.max(0, yCenter - cropSize / 2);
    double bottom = Math.min(imageHeight, yCenter + cropSize / 2);

    double[] normalizedCropValues = normalizeCropValues(left, right, top, bottom);

    table.getEntry("crop").setDoubleArray(normalizedCropValues);
  }

  private double[] normalizeCropValues(double left, double right, double top, double bottom) {
    double normalizedLeft = (left / (imageWidth / 2)) - 1;
    double normalizedRight = (right / (imageWidth / 2)) - 1;
    double normalizedTop = (top / (imageHeight / 2)) - 1;
    double normalizedBottom = (bottom / (imageHeight / 2)) - 1;

    return new double[] {normalizedLeft, normalizedRight, normalizedTop, normalizedBottom};
  }
}

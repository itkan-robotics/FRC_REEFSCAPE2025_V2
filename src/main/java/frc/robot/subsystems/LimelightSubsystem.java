// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.HashMap;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public static NetworkTable table;

  Timer lastTargetTime = new Timer();
  public boolean targetSeen = false;
  private final String limelightName;
  private final double imageWidth = 320;
  private final double imageHeight = 240;
  private final double fovHorizontalDegrees = 59.6;
  private final double fovVerticalDegrees = 49.7;
  private final double areaMultiplier = 1.5;
  public boolean limelightHeadingGood = true;
  private PIDController m_aTagSpeedContoller;
  private PIDController m_aTagDirController;
  private HashMap<Integer, Double> reefIDsToAngles = new HashMap<Integer, Double>();
  public double[] tagPose = {0, 0, 0, 0};

  public LimelightSubsystem(String name) {
    limelightName = name;
    table = NetworkTableInstance.getDefault().getTable(limelightName);
    createReefIDsToAnglesHashMap();
  }

  public Command setLimelight() {
    return run(
        () -> {
          limelightHeadingGood = true;
        });
  }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable(leftLimelightName);
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("tv", table.getEntry("tv").getDouble(0.0));
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
   * <p> Last Updated by Abdullah Khaled, 2/9/2025
   * <p> Update (2/9): Added compatibility with TagOffsets so the tag's pose is automatically offset
   * when requested
   * @return
   *************************************************************************************/
  @SuppressWarnings("unused")
  public Pose2d getTagEstimatedPosition(Drive drive) {
    double[] targetPose =
        NetworkTableInstance.getDefault()
            .getTable(leftLimelightName)
            .getEntry("targetpose_robotspace")
            .getDoubleArray(new double[6]);
    ;

    double targetTX = targetPose[0] /*+ ((offset.getPipeline() == 1) ? -8.0 : 8.0)*/;
    double targetTY = targetPose[1];
    double targetTZ = targetPose[2];
    Rotation2d tAngleToRobot = Rotation2d.fromRadians(Math.atan2(targetTX, targetTZ));
    // System.out.println("targetRotation: " + tAngleToRobot.getDegrees());
    double distanceToTarget =
        getPrimaryFiducial(LimelightHelpers.getRawFiducials(leftLimelightName)).distToRobot;
    double absRotation = -1.0 * drive.getHeadingDegrees() - getReefAngle();

    // System.out.println("absRotation: " + absRotation);
    Pose2d targetPose2d = new Pose2d(targetTZ, -targetTX, Rotation2d.fromDegrees(absRotation));
    return targetPose2d == null ? new Pose2d(-1, -1, Rotation2d.fromDegrees(0)) : targetPose2d;
  }

  /**
   * @param limelightNames Array of names of Limelights
   * @return The name of the limelight with the "best" view of the AprilTag (the one with the lowest
   *     ambiguity)
   */
  public static String getPrimaryLimelight(ArrayList<String> limelightNames) {
    String primaryLimelight = leftLimelightName;
    double minAmbiguity = Double.POSITIVE_INFINITY;
    for (int i = 0; i < limelightNames.size(); i++) {
      RawFiducial primaryFiducial =
          getPrimaryFiducial(LimelightHelpers.getRawFiducials(limelightNames.get(i)));
      if (primaryFiducial.ambiguity < minAmbiguity) {
        minAmbiguity = primaryFiducial.ambiguity;
        primaryLimelight = limelightNames.get(i);
      }
    }
    return primaryLimelight;
  }

  public static String getPrimaryLimelight() {
    if (multipleLimelights) return singleLimelightName;

    ArrayList<String> limelightNames = new ArrayList<String>();
    limelightNames.add(leftLimelightName);
    limelightNames.add(rightLimelightName);
    return getPrimaryLimelight(limelightNames);
  }

  // Helper Methods

  /***************************************************************************************
   * Creates the hashMap for the reef AprilTags based on alliance;
   * if blue alliance, adds 11 to the AprilTag keys to account for different IDs
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   **************************************************************************************/

  public void createReefIDsToAnglesHashMap() {
    reefIDsToAngles.put(-1, -1.0);
    reefIDsToAngles.put(0, -1.0);
    reefIDsToAngles.put(6, -60.0); // 17
    reefIDsToAngles.put(7, 0.0); // 18
    reefIDsToAngles.put(8, 60.0); // 19
    reefIDsToAngles.put(9, 120.0); // 20
    reefIDsToAngles.put(10, 180.0); // 21
    reefIDsToAngles.put(11, -120.0); // 22
  }

  public double getReefAngle() {
    double angle = (reefIDsToAngles.get(getID()) == null) ? -1.0 : reefIDsToAngles.get(getID());
    // System.out.println("id: " + getID());
    return angle;
  }

  /*******************************************************
   * Function to get angle of target AprilTag based on its ID.
   *
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   * @return Angle of the target AprilTag in degrees
   *******************************************************/

  public double getLLReefAngle() {
    return reefIDsToAngles.get(getID(getPrimaryLimelight()));
  }

  public double getLLReefAngle(String limelightName) {
    return reefIDsToAngles.get((int) LimelightHelpers.getFiducialID(limelightName));
  }

  /***************************************************************************************
   * Gets the magnitude and direction the robot should drive in based on AprilTag data.
   * The method uses ta to calculate magnitude and tx to calculate direction, and an exponential
   * interpolation equation to find the kP value the speed controller should use based on ta
   * <p>Last Updated by Abdullah Khaled, 1/19/2025
   *
   * @param offset Offset for left and right branches
   * @return The linear velocity of the robot as a Translation2d
   **************************************************************************************/
  public Translation2d getAprilTagVelocity(
      double alignkP,
      double alignkD,
      int pipeline,
      boolean overTurned,
      double reefAngle,
      String limelightName) {

    // if (!multipleLimelights) LimelightHelpers.setPipelineIndex(limelightName, pipeline);

    m_aTagSpeedContoller = new PIDController(kPExpInterpolation(MAX_AREA), 0.0, 0.0);
    if (!overTurned) {
      double targetArea =
          LimelightHelpers.getTA(limelightName) != 0.0
              ? LimelightHelpers.getTA(limelightName)
              : MAX_AREA;

      m_aTagSpeedContoller = new PIDController(kPExpInterpolation(targetArea), 0.0, 0.0);
    }

    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            // Calculate speed based on ta
            m_aTagSpeedContoller.calculate(LimelightHelpers.getTA(limelightName), MAX_AREA)
                + ALIGN_KS,
            VELOCITY_DEADBAND);

    // Calculate direction based on tx
    m_aTagDirController = new PIDController(alignkP, 0.0, alignkD);
    Rotation2d linearDirection =
        new Rotation2d(
            MathUtil.applyDeadband(
                    m_aTagDirController.calculate(
                        Math.toRadians(LimelightHelpers.getTX(limelightName))),
                    VELOCITY_DEADBAND)
                + Math.toRadians(reefAngle));
    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public boolean withinTolerance(double toleranceX, double toleranceA, String limelight) {
    return ((Math.abs(LimelightHelpers.getTX(limelight)) <= toleranceX)
        && (Math.abs(LimelightHelpers.getTA(limelight) - MAX_AREA) <= toleranceA));
  }

  // Helper Methods

  /***************************************************************************************
   * Finds the kP of the speed controller using a linear interpolation equation as created by
   * ChatGPT
   * <p> Last Updated by Abdullah Khaled, 1/19/2025
   * @param ta The area of the limelight's FOV the target fills.
   * @return The interpolated value
   **************************************************************************************/

  public double kPExpInterpolation(double ta) {

    double area = MathUtil.clamp(ta, MIN_AREA, MAX_AREA);
    double[] pair0 = {MIN_AREA, MAX_KP.getAsDouble()};
    double[] pair1 = {MAX_AREA, MIN_KP.getAsDouble()};

    double k = -Math.log(pair1[1] / pair0[1]) / (pair1[0] - pair0[0]);
    double A = pair0[1] * Math.exp(k * pair0[0]);
    double interpolatedVal = A * Math.exp(-k * area);
    return interpolatedVal;
  }

  public double getX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getX(String limelightName) {
    return getLimelightTable(limelightName).getEntry("tx").getDouble(0.0);
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

  public double getArea(String limelightName) {
    return table.getEntry("ta").getDouble(0.0);
  }

  public boolean hasTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  public double getTV() {
    return table.getEntry("tv").getDouble(0.0);
  }

  public double getTV(String limelightName) {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tv").getDouble(0.0);
  }

  public int getID() {
    return (int) table.getEntry("tid").getDouble(0.0);
  }

  public int getID(String limelightName) {
    return (int) getLimelightTable(limelightName).getEntry("tid").getDouble(0.0);
  }

  public NetworkTable getPrimaryLimelightTable() {
    return NetworkTableInstance.getDefault().getTable(getPrimaryLimelight());
  }

  public NetworkTable getLimelightTable(String llName) {
    return NetworkTableInstance.getDefault().getTable(getPrimaryLimelight());
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

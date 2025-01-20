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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private PIDController m_aTagSpeedContoller;
  private PIDController m_aTagDirController;
  private HashMap<Integer, Double> reefAngles = new HashMap<Integer, Double>();

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

  /*******************************************************
   * Function to get angle of target AprilTag based on its ID.
   *
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   * @return Angle of the target AprilTag in degrees
   *******************************************************/

  public double getLLReefAngle() {
    return reefAngles.get(getID());
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

  public Translation2d getAprilTagVelocity(double offset, boolean overTurned, double reefAngle) {

    m_aTagSpeedContoller = new PIDController(kPExpInterpolation(MAX_AREA), 0.0, 0.0);
    if (!overTurned) {
      double targetArea = getArea() != 0.0 ? getArea() : MAX_AREA;

      m_aTagSpeedContoller = new PIDController(kPExpInterpolation(targetArea), 0.0, 0.0);
    }

    // Once speed controller tuned, I'd like to tune direction as well so != linear
    // PIDContoller m_aTagDirController = new PIDContoller(
    // kP, kI, kD);

    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            // Calculate speed based on ta
            m_aTagSpeedContoller.calculate(getArea(), MAX_AREA) + SPEED_KS, 0.025);

    // Calculate direction based on tx
    m_aTagDirController = new PIDController(1.695, 0.0, 0.001);
    Rotation2d linearDirection =
        new Rotation2d(
            MathUtil.applyDeadband(
                    m_aTagDirController.calculate(Math.toRadians(getX() + offset)), 0.01)
                + Math.toRadians(reefAngle));
    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
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
    double[] pair0 = {MIN_AREA, 0.12};
    double[] pair1 = {MAX_AREA, 0.015};

    double k = -Math.log(pair1[1] / pair0[1]) / (pair1[0] - pair0[0]);
    double A = pair0[1] * Math.exp(k * pair0[0]);
    double interpolatedVal = A * Math.exp(-k * area);
    return interpolatedVal;
  }

  /***************************************************************************************
   * Creates the hashMap for the reef AprilTags based on alliance;
   * if blue alliance, adds 11 to the AprilTag keys to account for different IDs
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   **************************************************************************************/

  public void createReefHashMap() {
    int blueAllianceOffset = !isRedAlliance() ? 11 : 0;
    reefAngles.put(-1, -1.0);

    reefAngles.put(6 + blueAllianceOffset, 120.0);
    reefAngles.put(7 + blueAllianceOffset, 180.0);
    reefAngles.put(8 + blueAllianceOffset, -120.0);
    reefAngles.put(9 + blueAllianceOffset, -60.0);
    reefAngles.put(10 + blueAllianceOffset, 0.0);
    reefAngles.put(11 + blueAllianceOffset, 60.0);
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

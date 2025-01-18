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
import frc.robot.LimelightHelpers;
import java.util.HashMap;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  Timer lastTargetTime = new Timer();
  double detectedTargetDistance = -1;
  double detectedYaw = -1;
  public boolean targetSeen = false;
  private final double imageWidth = 320;
  private final double imageHeight = 240;
  private final double fovHorizontalDegrees = 59.6;
  private final double fovVerticalDegrees = 49.7;
  private final double areaMultiplier = 1.5;
  public double distance;
  public boolean limelightHeadingGood = true;
  private PIDController m_moveController = new PIDController(0, 0, 0);
  private HashMap<Integer, Double> reefAngles = new HashMap<Integer, Double>();

  public LimelightSubsystem() {
    createReefHashMap();
    //// SmartDashboard.putNumber("tx - new", table.getEntry("tx").getDouble(0.0));
  }

  public Command setLimelight() {
    return run(
        () -> {
          limelightHeadingGood = true;
        });
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Limelight Range", detectedTargetDistance);
    if (table.getEntry("tv").getDouble(0.0) == 1) {
      lastTargetTime.restart();
      // SmartDashboard.putNumber("Photon Yaw", target.getYaw());
      //   System.out.println("Skew: " + getSkew());
      //   System.out.println("Tx" + table.getEntry("tx").getDouble(0.0));
      // final double camera_height =
      //     Units.inchesToMeters(Constants.LimelightConstants.limelightLensHeightInches);
      // final double target_height =
      //     Units.inchesToMeters(Constants.LimelightConstants.goalHeightInches);
      // final double camera_pitch =
      //     Units.degreesToRadians(Constants.LimelightConstants.limelightMountAngleDegrees);

      // double range =
      //     PhotonUtils.calculateDistanceToTargetMeters(
      //         camera_height,
      //         target_height,
      //         camera_pitch,
      //         Units.degreesToRadians(table.getEntry("ty").getDouble(0.0)));
      // detectedTargetDistance =
      //     Units.metersToInches(range) - Constants.LimelightConstants.cameraToReefDistance;
      detectedYaw = table.getEntry("tx").getDouble(0.0);
      targetSeen = true;
      //// SmartDashboard.putNumber("Limelight Range", detectedTargetDistance);

    }
    if (lastTargetTime.get() < .1) {
    } else {
      targetSeen = false;
      detectedYaw = 0;
      detectedTargetDistance = -1;
    }
    // SmartDashboard.putNumber("Limelight Range", detectedTargetDistance);
    // This method will be called once per scheduler run
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

  public double getDistance() {
    return detectedTargetDistance;
  }

  public double getYaw() {
    return detectedYaw;
  }

  public boolean canSeeTarget() {
    return targetSeen;
  }

  public double getSkew() {
    return LimelightHelpers.getT2DArray("limelight")[16];
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

  public void createReefHashMap() {
    int blueAllianceOffset = !isRedAlliance() ? 11 : 0;

    reefAngles.put(1, 0.0); // Test Value b/c ID 10 no esta T_T

    reefAngles.put(6 + blueAllianceOffset, -120.0);
    reefAngles.put(7 + blueAllianceOffset, 180.0);
    reefAngles.put(8 + blueAllianceOffset, 120.0);
    reefAngles.put(9 + blueAllianceOffset, 60.0);
    reefAngles.put(10 + blueAllianceOffset, 0.0);
    reefAngles.put(11 + blueAllianceOffset, -60.0);
  }

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

  /**
   * Function to get angle of target AprilTag based on ID
   *
   * @return Angle of target AprilTag in degrees
   */
  public double getLLReefAngle() {
    return reefAngles.get(getID());
  }

  /**
   * Gets the magnitude and direction the robot should drive in based on AprilTag data.
   *
   * <p>The method uses ta to calculate magnitude and tx to calculate direction
   *
   * @param offset offset for left and right branches
   * @return the linear velocity of the robot as a Translation2d
   */
  public Translation2d getAprilTagVelocity(double offset, double kP, double kI, double kD) {

    m_moveController = new PIDController(kP, kI, kD);

    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            // Calculate speed based on ta
            m_moveController.calculate(getArea(), maxArea), 0.025);

    // Calculate direction based on tx (*-1 b/c when we are to the left, we want robot to go right)
    Rotation2d linearDirection = new Rotation2d(Math.toRadians((getX() + offset) * -1.0));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}

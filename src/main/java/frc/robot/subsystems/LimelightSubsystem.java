// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

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
  private PIDController m_thetaController = new PIDController(0, 0, 0);
  private PIDController m_moveController = new PIDController(0, 0, 0);

  public LimelightSubsystem() {
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

  /**
   * Get rotation value (in m/s) robot needs to rotate to be parallel with AprilTag, given the robot
   * heading. Uses getTargetSkew() to align parallel to the AprilTag.
   *
   * @param robotHeading
   * @return Returns direction and magnitude of robot rotation between -2 and 2 m/s.
   */
  public double getAprilTagParallelRot(double kP, double kI, double kD, double robotHeading) {
    m_thetaController = new PIDController(kP, kI, kD);
    m_thetaController.enableContinuousInput(-180, 180);
    double targetAngle = -getSkew();

    // Rest is the same as old code for angling
    m_thetaController.setSetpoint(targetAngle);
    double omega =
        m_thetaController.calculate(
            (MathUtil.inputModulus(robotHeading, -180, 180)), m_thetaController.getSetpoint());

    return MathUtil.clamp(omega, -2, 2);
  }

  /**
   * Gets how fast and at what angle the robot should drive towards the AprilTag.
   *
   * @return Returns how fast the robot should drive (in m/s) and in what direction
   */
  public Translation2d getAprilTagDrive(double kP, double kI, double kD) {
    m_moveController = new PIDController(kP, kI, kD);

    // Calculate forwards speed
    double x = m_moveController.calculate(LimelightHelpers.getTY("limelight"));
    x *= -1.0;

    // Calculate sideways speed
    double y = m_moveController.calculate(LimelightHelpers.getTX("limelight"));
    y *= 1.0;

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the
  // "tx" value from the Limelight.
  public double limelight_aim_proportional(double kP) {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control
    // loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for
  // target ranging rather than "ty"
  public double limelight_range_proportional(double kP) {
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  public double getLLReefAngle() {
    double id = LimelightHelpers.getFiducialID("limelight");

    if (id == 1) {
      return 60;
    }

    return -1;
  }
}

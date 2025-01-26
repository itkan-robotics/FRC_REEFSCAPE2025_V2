// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.MAX_AREA;
import static frc.robot.Constants.LimelightConstants.MAX_KP;
import static frc.robot.Constants.LimelightConstants.MIN_AREA;
import static frc.robot.Constants.LimelightConstants.MIN_KP;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.List;

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
    maReefAlignment(null);
  }

  // Mechanical Advantage reef alignment
  // Get the tag's position relative to the robot (distToRobot)
  //
  public Pose2d maReefAlignment(Drive drive) {
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
                * NetworkTableInstance.getDefault()
                    .getTable("SmartDashboard")
                    .getEntry("Heading")
                    .getDouble(0.0)
            - getReefAngle();

    // System.out.println("absRotation: " + absRotation);
    Pose2d targetPose2d = new Pose2d(targetTZ, -targetTX, Rotation2d.fromDegrees(absRotation));
    return targetPose2d == null ? new Pose2d(-1, -1, Rotation2d.fromDegrees(0)) : targetPose2d;
  }

  public PathPlannerPath reefAlignmentPath(
      Drive drive, double frontOffsetInches, Constants.TagOffsets offset) {

    setPipeline(offset.getPipeline());
    // Get the current pose of the robot.
    Pose3d robotPose = drive.getRobotPose3d();

    // Get the current heading of the robot
    Rotation2d robotHeading = drive.getRotation();

    // Convert the target's robot relative pose into a "field-relative" pose
    Pose2d targetPose = robotPose.toPose2d();

    if (hasTarget())
      targetPose =
          robotPose
              .transformBy(
                  new Transform3d(
                      new Pose3d(), LimelightHelpers.getTargetPose3d_RobotSpace("limelight")))
              .transformBy(
                  new Transform3d(
                      new Translation3d(Units.inchesToMeters(frontOffsetInches), 0, 0),
                      new Rotation3d(0, 0, Math.PI)))
              .toPose2d();

    // Get the desired direction of travel by subtracting the target's heading from the robot's
    // heading. This way, the robot travels in a straight line to the apriltag.
    Rotation2d directionOfTravel = targetPose.getRotation().minus(robotHeading);

    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(targetPose.getTranslation(), directionOfTravel),
            new Pose2d(
                targetPose.getTranslation(),
                directionOfTravel.plus(Rotation2d.fromRadians(Math.PI))));

    PathConstraints constraints =
        new PathConstraints(
            drive.getMaxLinearSpeedFeetPerSec(),
            drive.getMaxLinearSpeedFeetPerSec() * 0.5,
            drive.getMaxAngularSpeedRadPerSec(),
            drive.getMaxAngularSpeedRadPerSec() * 2); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                targetPose
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a
            // differential drivetrain, the rotation will have no effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return path;
  }

  // Helper Methods

  public Transform3d getTargetPoseRobotRelative3d() {
    /* "3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
    [tx, ty, tz, pitch, yaw, roll] (meters, degrees)"
    https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data*/
    double[] targetpose_robotspace = LimelightHelpers.getTargetPose_RobotSpace("limelight");

    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
    double tagAmbiguity = Double.POSITIVE_INFINITY;
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id == getID()) tagAmbiguity = fiducial.ambiguity;
    }
    Transform3d targetPoseRobotRelative = new Transform3d();
    if (tagAmbiguity < 0.5) {
      targetPoseRobotRelative =
          new Transform3d(
              targetpose_robotspace[0],
              targetpose_robotspace[1],
              targetpose_robotspace[2],
              new Rotation3d(
                  Math.toRadians(targetpose_robotspace[5]),
                  Math.toRadians(targetpose_robotspace[3]),
                  Math.toRadians(targetpose_robotspace[4])));
    }

    return targetPoseRobotRelative;
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

  /***************************************************************************************
   * Finds the kP of the speed controller using a linear interpolation equation as created by
   * ChatGPT
   * <p> Last Updated by Abdullah Khaled, 1/19/2025
   * @param ta The area of the limelight's FOV the target fills.
   * @return The interpolated value
   **************************************************************************************/
  public double kPExpInterpolation(double ta) {

    double area = MathUtil.clamp(ta, MIN_AREA, MAX_AREA);
    double[] pair0 = {MIN_AREA, MAX_KP};
    double[] pair1 = {MAX_AREA, MIN_KP};

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
    reefAngles.put(6 + blueAllianceOffset, -60.0); // 17
    reefAngles.put(7 + blueAllianceOffset, 0.0); // 18
    reefAngles.put(8 + blueAllianceOffset, 60.0); // 19
    reefAngles.put(9 + blueAllianceOffset, 120.0); // 20
    reefAngles.put(10 + blueAllianceOffset, 180.0); // 21
    reefAngles.put(11 + blueAllianceOffset, -120.0); // 22
  }

  public double getReefAngle() {
    double angle = (reefAngles.get(getID()) == null) ? -1.0 : reefAngles.get(getID());
    System.out.println("id: " + getID());
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

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex("limelight", pipeline);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import java.util.Set;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Code obtained from
// https://www.chiefdelphi.com/t/need-help-with-integrating-pose-estimation-with-apriltags-and-pathplanner-trajectories-in-auto-teleop/455287
public class limelightReefAlignment extends SequentialCommandGroup {
  Drive m_drive;
  LimelightSubsystem m_limelight;
  LimelightHelpers.RawFiducial m_primaryTarget;

  Transform3d TAG_TO_GOAL;
  Pose3d targetPoseEst;
  Pose2d goalPose;
  PIDController translationController;
  PIDController thetaController;

  /***************************************************************************************************************************************************************
   * SequentialCommandGroup that uses Pathplanner AutoBuilder to create a path from the robot's
   * current position to the primary AprilTag's estimated position.
   * <p>(Base code obtained from
   * <a href=https://www.chiefdelphi.com/t/need-help-with-integrating-pose-estimation-with-apriltags-and-pathplanner-trajectories-in-auto-teleop/455287>HERE</a>.
   * Modifications include making it use LimelightHelpers methods instead of PhotonVision)
   * <p> Last Updated by Abdullah Khaled, 1/25/2025
   * @param drive Swerve subsystem
   * @param limelight Limelight subsystem
   * @param frontOffsetInches Offset in inches; a negative value results in an offset AWAY from the
   * AprilTag. If for some reason you want the robot to go behind the AprilTag, make the value positive.
   * @param offset Where we want to align (i.e. Left, Center, Right). Each enum has a value
   * associated with it that corresponds to the Limelight pipeline the camera should use.
   ***************************************************************************************************************************************************************/
  public limelightReefAlignment(
      Drive drive,
      LimelightSubsystem limelight,
      double frontOffsetInches,
      Constants.TagOffsets offset) {

    this.m_limelight = limelight;
    m_limelight.setAprilTagOffset(offset);
    this.m_drive = drive;
    this.TAG_TO_GOAL =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(frontOffsetInches), 0, 0),
            new Rotation3d(0, 0, 0.0));

    addRequirements(drive);
    addCommands(
        new DeferredCommand(() -> macroPath(), Set.of(drive)),
        new DeferredCommand(() -> microPath(), Set.of(drive)),
        new InstantCommand(() -> drive.stopWithX()));
  }

  public Command macroPath() {

    if (m_limelight.canSeeTarget() == false) {
      return new InstantCommand();
    } else {
      // Transform the tag's pose to set our goal
      targetPoseEst = new Pose3d(m_limelight.getTagEstimatedPosition(m_drive));

      var robotPose2d = m_drive.getPoseLatencyCompensation(m_limelight.getLatency());

      var robotPose3d =
          new Pose3d(
              robotPose2d.getX(),
              robotPose2d.getY(),
              0,
              new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

      goalPose =
          targetPoseEst.getX() <= 0
              ? robotPose3d.toPose2d()
              : robotPose3d
                  .transformBy(new Transform3d(new Pose3d(), targetPoseEst))
                  .transformBy(TAG_TO_GOAL)
                  .toPose2d();

      return AutoBuilder.pathfindToPose(
          goalPose,
          new PathConstraints(
              15.0, // Throttled at 15 ft/s
              10.0,
              Units.degreesToRadians(180),
              Units.degreesToRadians(75)),
          0);
    }
  }

  public Command microPath() {
    var robotPose2d = m_drive.getPoseLatencyCompensation(m_limelight.getLatency());

    var robotPose3d =
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0,
            new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

    if (m_limelight.canSeeTarget() == true) {
      // Transform the tag's pose to set our goal; otherwise, just use last estimated position
      targetPoseEst = new Pose3d(m_limelight.getTagEstimatedPosition(m_drive));
    }
    goalPose =
        targetPoseEst.getX() <= 0
            ? robotPose3d.toPose2d()
            : robotPose3d
                .transformBy(new Transform3d(new Pose3d(), targetPoseEst))
                .transformBy(TAG_TO_GOAL)
                .toPose2d();
    translationController = new PIDController(50.0, 0, 0);
    thetaController = new PIDController(0.5, 0, 0);

    return Commands.run(
        () -> {
          double xSpeed = translationController.calculate(robotPose2d.getX(), goalPose.getX());
          double ySpeed = translationController.calculate(robotPose2d.getY(), goalPose.getY());
          double omega =
              thetaController.calculate(
                  m_drive.getHeadingDegrees(), goalPose.getRotation().getDegrees());

          while (xSpeed < 0.03 && ySpeed < 0.03 && omega < 0.1) {
            xSpeed = translationController.calculate(robotPose2d.getX(), goalPose.getX());
            ySpeed = translationController.calculate(robotPose2d.getY(), goalPose.getY());
            omega =
                thetaController.calculate(
                    m_drive.getHeadingDegrees(), goalPose.getRotation().getDegrees());
            // Convert to field relative speeds & send command
            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    xSpeed * m_drive.getMaxLinearSpeedFeetPerSec(),
                    ySpeed * m_drive.getMaxLinearSpeedFeetPerSec(),
                    omega);
            boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            m_drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    isFlipped
                        ? m_drive.getRotation().plus(new Rotation2d(Math.PI))
                        : m_drive.getRotation()));
          }
        },
        m_drive);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
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
        new DeferredCommand(() -> getCommand(), Set.of(drive)),
        new InstantCommand(() -> drive.stop()));
  }

  public Command getCommand() {

    var robotPose2d = m_drive.getPoseLatencyCompensation(m_limelight.getLatency());

    var robotPose3d =
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0,
            new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

    if (m_limelight.canSeeTarget() == false) {
      return new InstantCommand();
    } else {
      // Transform the tag's pose to set our goal

      var targetPoseEst = new Pose3d(m_limelight.getTagEstimatedPosition());
      var goalPose =
          targetPoseEst.getX() <= 0
              ? robotPose3d.toPose2d()
              : robotPose3d
                  .transformBy(new Transform3d(new Pose3d(), targetPoseEst))
                  .transformBy(TAG_TO_GOAL)
                  .toPose2d();

      return AutoBuilder.pathfindToPose(
          goalPose,
          new PathConstraints(
              m_drive.getMaxLinearSpeedFeetPerSec(),
              30,
              Units.degreesToRadians(720),
              Units.degreesToRadians(1440)),
          0);
    }
  }
}

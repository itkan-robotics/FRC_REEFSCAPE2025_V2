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
import frc.robot.LimelightHelpers.RawFiducial;
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

  /** Creates a new SequentialChaseTagCmd. */
  public limelightReefAlignment(
      Drive drive,
      LimelightSubsystem limelight,
      double frontOffsetInches,
      Constants.TagOffsets offset) {

    limelight.setPipeline(offset.getPipeline());

    this.m_limelight = limelight;
    this.m_drive = drive;
    this.TAG_TO_GOAL =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(frontOffsetInches), 0, 0),
            new Rotation3d(0, 0, Math.PI));

    addRequirements(drive);
    addCommands(
        new DeferredCommand(() -> getCommand(), Set.of(drive)),
        new InstantCommand(() -> drive.stop()));
  }

  public Command getCommand() {

    var robotPose2d = m_drive.getPose();

    var robotPose3d =
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0,
            new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

    if (m_limelight.hasTarget() == false) {
      return new InstantCommand();
    } else {
      try {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
        m_primaryTarget = fiducials[0];
        for (RawFiducial fiducial : fiducials) {
          // .ta, .distToCamera, .distToRobot, .ambiguity
          if (fiducial.id == m_limelight.getID()) m_primaryTarget = fiducial;
        }
        // this doesn't really do anything unless "always do single target estimation is
        // checked -> by default, returns -1"
        if (m_primaryTarget.ambiguity >= 0.2) {
          return new InstantCommand();
        }

        // System.out.println("ID: " + targetToUse.getFiducialId() + " ambig = "
        // + targetToUse.getPoseAmbiguity());
        // Get the transformation from the camera to the tag
        // double[] targetpose_robotspace = LimelightHelpers.getTargetPose_RobotSpace("limelight");

        // var targetPoseRobotRelative =
        //     new Transform3d(
        //         targetpose_robotspace[0],
        //         targetpose_robotspace[1],
        //         targetpose_robotspace[2],
        //         new Rotation3d(
        //             Math.toRadians(targetpose_robotspace[5]),
        //             Math.toRadians(targetpose_robotspace[3]),
        //             Math.toRadians(targetpose_robotspace[4])));

        var targetPoseRobotRelative =
            new Transform3d(new Pose3d(), LimelightHelpers.getTargetPose3d_RobotSpace("limelight"));

        // Transform the robot's pose to find the tag's pose
        var targetPose = robotPose3d.transformBy(targetPoseRobotRelative);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        return AutoBuilder.pathfindToPose(
            goalPose,
            new PathConstraints(3.0, 2, Units.degreesToRadians(540), Units.degreesToRadians(720)),
            0);

      } catch (NullPointerException ex) {
        ex.printStackTrace();
        return new InstantCommand();
      }
    }
  }
}
